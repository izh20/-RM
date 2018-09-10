/**
  *@file Driver_app.c
  *@date 2018-1-17
  *@author ��������
  *@brief ң��У����ң�ؽ��롢���Ƶ�����Ʊ�־λ
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

uint8_t un_control=0;   //��س����������Ƿ�ʧ��
Test_ResultTypedef sTestResult = {0, 0, 0, 1, 0, 0};

uint8_t DBUSCheak=CHEAKSTART;   //ң��У����־λ  0x00:δУ����     0x40:����У��  0x80:����У��
extern int16_t RealSpeedLF,RealSpeedRF,RealSpeedLB,RealSpeedRB;  //���̵���ķ����ٶ�
//extern AngleTypedef ChassisAngle;
//extern int16_t moto_ctr[];
extern CatTypedef CatStruct; 
/*total task used to check all,it will be used in timer 6 interrupt*/
//������亯��,�������ڸı��ź���
//������TIM6�жϺ�������,TIM6(������ʱ��,ÿ1ms�ж�һ��)
void test_task(void)
{
	static uint8_t DBUSCheaktime=0;
  Timetick1ms++;
	//unsigned char i=0;		
	if(DBUSCheak==CHEAKING)  //����У�� 
	    DBUSCheaktime++;
	
	if(DBUSCheaktime>=5)  //У�����
	 {
		  DBUSCheak=CHEAKFINISH;   //������У��
		  DBUSCheaktime=0;
		  MX_USART1_UART_Init();  //ң�������ڽ�������  DBUS RX: PB7
		  HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18);
	 }
	
    if(ShootNumber.Number>0)
   {
       if((ShootNumber.LastFinish==0x00)&&(ShootNumber.AllowShoot==0x00))  //����ϴη�������  
       {
             ShootNumber.LastFinish=0x01;
             ShootNumber.Number--;
             StartShoot();   //������һ�εķ���    
       }
   }
   
   if(TimesFlag==1)
   {
      TimesGo++;  
   }
   
	//���5ms����һ��
  if(Timetick1ms % 5 == 0)
  {
		//������Ʊ�־λ��һ���ڽ��п��Ʋ�����
		contro_flag = 1;
     
    if(ElectricShoot>PluckMotor.DeadElectric)    //��ѹֵ����������ѹ
		{
      
		     	if(DeadShoot>=80)
					{
						PluckMotor.Stop=1;  //����Ѿ�ͣת
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
          PluckMotor.Stop=0;        //�������
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
		//��̨������Ʊ�־λ
		Head_flag = 1;
  }
  
 
	 //ң�ؽ������
	if(start_flag == 1)
	{
		if(DBUSCheak==CHEAKFINISH)   //ң���Ѿ�У����
			MotorTargetChange();     //Chassis_control();	
		else
		{
			DBUSCheak=CHEAKING;       //����У��	
			DBUS_Init();     //���������
		}
			start_flag=0;
	}

  //ת�ټ�س���	
	if((Timetick1ms % 100 == 0)&&(CatStruct.mode==0))
  {
		//��ֹ����ת�ٳ���ʧ��
		if(abs(RealSpeedLF)>5000||abs(RealSpeedRF)>5000||abs(RealSpeedRB)>5000||abs(RealSpeedLB)>5000)
		{
			un_control+=5;
		}
		
		//��ֹĿ��Ϊ0��ת�ٳ�����Ϊ0��ʧ��
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
  
 //�Ƿ�����������
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
  
   
	//ÿ1s����һ������У��,�Լ���������
  if(Timetick1ms % 1000 == 0)
  {
		Timetick1ms  = 0;
		DBUSCheak=CHEAKSTART;
  }
	
	
}
