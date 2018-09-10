/**
  *@file Driver_app.c
  *@date 2018-1-17
  *@author 天涯浪子
  *@brief 遥控校正、遥控解码、控制电机控制标志位
  */
	
#include "Task_tim.h"
#include <stdio.h>  
#include "Motor_Shoot.h"

uint32_t Timetick1ms = 0;
 extern int16_t ElectricShoot;
 
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
uint8_t contro_flag=0,start_flag=0;
uint8_t Head_flag=0;
int16_t DeadShoot=0;
extern uint8_t TimesFlag;
extern uint16_t  TimesGo;

uint8_t un_control=0;   //监控程序里面检测是否失控
Test_ResultTypedef sTestResult = {0, 0, 0, 1, 0, 0};

uint8_t DBUSCheak=CHEAKSTART;   //遥控校正标志位  0x00:未校正过     0x40:开启校正  0x80:结束校正
extern int16_t RealSpeedLF,RealSpeedRF,RealSpeedLB,RealSpeedRB;  //底盘电机的反馈速度
//extern AngleTypedef ChassisAngle;
//extern int16_t moto_ctr[];
extern CatTypedef CatStruct; 
/*total task used to check all,it will be used in timer 6 interrupt*/
//任务分配函数,建议用于改变信号量
//被调用TIM6中断函数调用,TIM6(基本定时器,每1ms中断一次)
void test_task(void)
{
	static uint8_t DBUSCheaktime=0;
  Timetick1ms++;
	//unsigned char i=0;		
	if(DBUSCheak==CHEAKING)  //开启校正 
	    DBUSCheaktime++;
	
	if(DBUSCheaktime>=5)  //校正完成
	 {
		  DBUSCheak=CHEAKFINISH;   //结束了校正
		  DBUSCheaktime=0;
		  MX_USART1_UART_Init();  //遥控器串口接收配置  DBUS RX: PB7
		  HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18);
	 }
	
    if(ShootNumber.Number>0)
   {
       if((ShootNumber.LastFinish==0x00)&&(ShootNumber.AllowShoot==0x00))  //如果上次发弹结束  
       {
             ShootNumber.LastFinish=0x01;
             ShootNumber.Number--;
             StartShoot();   //开启下一次的发弹    
       }
   }
   
   if(TimesFlag==1)
   {
      TimesGo++;  
   }
   
	//电机5ms控制一次
  if(Timetick1ms % 5 == 0)
  {
		//电机控制标志位置一，在进行控制并清零
		contro_flag = 1;
     
    if(ElectricShoot>PluckMotor.DeadElectric)    //电压值等于死区电压
		{
      
		     	if(DeadShoot>=80)
					{
						PluckMotor.Stop=1;  //电机已经停转
					}
					else if(DeadShoot<95)
						DeadShoot++;
		}
		else 
		{
			if(PluckMotor.Stop==1)
			{
				if(DeadShoot>0)
						DeadShoot-=3;
				else
        {
          PluckMotor.Stop=0;        //解除卡弹
          ShootNumber.LastFinish=0X00;
        }
		  }
      else
				 DeadShoot=0;
	  }
   get_referee_data(); 
    
  }
   if(Timetick1ms % 4 == 0)
  {
		//云台电机控制标志位
		Head_flag = 1;
  }
  
 
	 //遥控接收完毕
	if(start_flag == 1)
	{
		if(DBUSCheak==CHEAKFINISH)   //遥控已经校正过
			MotorTargetChange();     //Chassis_control();	
		else
		{
			DBUSCheak=CHEAKING;       //开启校正	
			DBUS_Init();     //接收区清空
		}
			start_flag=0;
	}

  //转速监控程序	
	if((Timetick1ms % 100 == 0)&&(CatStruct.mode==0))
  {
		//防止发生转速超过失控
		if(abs(RealSpeedLF)>5000||abs(RealSpeedRF)>5000||abs(RealSpeedRB)>5000||abs(RealSpeedLB)>5000)
		{
			un_control+=5;
		}
		
		//防止目标为0，转速持续不为0的失控
		if(((ChassisPidLF.target==0)&&(RealSpeedLF!=0))||((ChassisPidRF.target==0)&&(RealSpeedRF!=0))||\
			((ChassisPidRB.target==0)&&(RealSpeedRB!=0))||((ChassisPidLB.target==0)&&(RealSpeedLB!=0)))
		{
		     un_control++;
		}
		else
			  un_control=0;
  } 
  
  if(CatStruct.mode==0x01)
  {
        if(CatStruct.GyTime>900)
        {
           CatStruct.GyTime=0;
           if(CatStruct.dir==0)
           {
             
             CatStruct.dir=1;
             ChassisPidLF.target=2000;
             ChassisPidRF.target=2000;
             ChassisPidLB.target=2000;
             ChassisPidRB.target=2000;
           }
           else
           {
             CatStruct.dir=0;
             ChassisPidLF.target=-2000;
             ChassisPidRF.target=-2000;
             ChassisPidLB.target=-2000;
             ChassisPidRB.target=-2000;
           }
        }
        else
          CatStruct.GyTime++;
  }
  
 //是否开启电子罗盘
//  #if 0
//  
//  if(Timetick1ms%20==0)
//  {
//      if(ChassisAngle.mode==1)
//	       ChassisAuto(moto_ctr);
//  }
//    
//	if(Timetick1ms % 102 == 0)
//	{
//		  HAL_UART_Receive_IT(&huart3, uart3_rx_buff, 8);
//		  USART3->DR=0x31; 
//	}
//	#endif	
  
   
	//每1s进行一次数据校正,以及数据清零
  if(Timetick1ms % 1000 == 0)
  {
		Timetick1ms  = 0;
		DBUSCheak=CHEAKSTART;
  }
	
	
}
