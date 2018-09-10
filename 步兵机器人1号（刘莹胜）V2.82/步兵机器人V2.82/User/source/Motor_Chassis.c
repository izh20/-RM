
//底盘电机闭环控制程序
//天涯浪子
//2018/1/12
//

#include "Motor_Chassis.h"
#include <stdio.h>  
#include "Motor_Shoot.h"


//PidTypeDef ChassisAngle={0};   //底盘姿态PID
//AngleTypedef  ChassisAngle={0};
PidTypeDef ChassisPidLF={0},ChassisPidRF={0},ChassisPidRB={0},ChassisPidLB={0};

//PidTypeDef YawOPID={0};   //航偏角        { 2000, 1.6, 0.05, 10, {0,0,0,0,0}, 0}; 


/**
  * @brief  PID初始化函数
  * @param  void
  * @retval void
  */

const double KP=7,KI=0.3,KD=5;  //底盘电机PID
//int16_t ChassisAngleNow=0;  //电子罗盘角度值

//extern int16_t SPEEDTARGET;
int16_t SPEEDTARGET=0;
int16_t AngleMid=0;

void PIDInit(void)
{
//    ChassisAngle.mode=0;  
//    ChassisAngle.AngleMax=450;    //向右边45度
//    ChassisAngle.AngleMin=-450;   //向左边45度
//    ChassisAngle.AngleAdd=5;      //每次偏移0.5度 
//    ChassisAngle.dir=0;
//  
//    ChassisAngle.ChassisPid.Kp=10;
//    ChassisAngle.ChassisPid.Ki=0;
//	  ChassisAngle.ChassisPid.Kd=0;
//	  ChassisAngle.ChassisPid.Istart=20;
//	  ChassisAngle.ChassisPid.OutMax=2500;
//	  ChassisAngle.ChassisPid.SumMax=20000;
//  	ChassisAngle.ChassisPid.target=600; 

  
	//  char i=0;
	//底盘电机LF
	  ChassisPidLF.Kp=KP;
    ChassisPidLF.Ki=KI;
	  ChassisPidLF.Kd=KD;
	 // ChassisPidLF.AddMax=1500;
	  ChassisPidLF.Istart=200;
	  ChassisPidLF.OutMax=CHASSISMAX;
	  ChassisPidLF.SumMax=20000;
  	ChassisPidLF.target=SPEEDTARGET; 

	
		//底盘电机RF
	  ChassisPidRF.Kp=KP;
    ChassisPidRF.Ki=KI;
	  ChassisPidRF.Kd=KD;
	  ///ChassisPidRF.AddMax=1500;
	  ChassisPidRF.Istart=200;
	  ChassisPidRF.OutMax=CHASSISMAX;
	  ChassisPidRF.SumMax=20000;
  	ChassisPidRF.target=SPEEDTARGET; 

	
	//底盘电机RB
	  ChassisPidRB.Kp=KP;
    ChassisPidRB.Ki=KI;
	  ChassisPidRB.Kd=KD;
		//ChassisPidRB.AddMax=1500;
		ChassisPidRB.Istart=200;
	  ChassisPidRB.OutMax=CHASSISMAX;
	  ChassisPidRB.SumMax=20000;
  	ChassisPidRB.target=SPEEDTARGET; 

			
//底盘电机LB			
	  ChassisPidLB.Kp=KP;
    ChassisPidLB.Ki=KI;
	  ChassisPidLB.Kd=KD;
		//ChassisPidLB.AddMax=1500;
		ChassisPidLB.Istart=200;
	  ChassisPidLB.OutMax=CHASSISMAX;
	  ChassisPidLB.SumMax=20000;
  	ChassisPidLB.target=SPEEDTARGET; 	
	
}

//姿态校正
//中间值 680  
//
//int16_t mid_dir=500;  //中心角度值
//int16_t SpeedCorrect=0;


//void CorrectDirect(int16_t *moto_ctr)  //前进方向校正
//{
//  SpeedCorrect=-PIDCalc(&ChassisAngle.ChassisPid,ChassisAngleNow);  
//  moto_ctr[0]=SpeedCorrect;  
//  moto_ctr[1]=SpeedCorrect;  
//  moto_ctr[2]=SpeedCorrect;  
//  moto_ctr[3]=SpeedCorrect;  
//}

//进入诡步模式
//void ChassisAuto(int16_t *moto_ctr)  
//{
//  if(ChassisAngle.mode==1)  //如果开启了诡步模式
//  {
//      if(ChassisAngle.dir==0)
//      {
//          ChassisAngle.AngleMove+=ChassisAngle.AngleAdd;
//          if(ChassisAngle.AngleMove>=450)
//          {
//             ChassisAngle.dir=1;
//          }
//      }
//      else //if(ChassisAngle.dir==0)
//      {
//          ChassisAngle.AngleMove-=ChassisAngle.AngleAdd;
//          if(ChassisAngle.AngleMove<=ChassisAngle.AngleMin)
//          {
//             ChassisAngle.dir=0;
//          }
//      }
//   ChassisAngle.ChassisPid.target=ChassisAngle.AngleMid+ChassisAngle.AngleMove; 
//   CorrectDirect(moto_ctr);
// }
//   //ChassisAngle.
//}

int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 
extern uint8_t un_control;
extern char sysflag;    //串口掉线标志位
extern refDataStruct  refSysData;      //裁判数据结构体，其中每个变量代表什么我已经在.h文件里面说明

#define  INFPOWER    78   //步兵限制功率值
#define  HEROPOWER   118   //英雄限制功率值

float  PowerCof;
/** 
  * @note   modified
  * @brief  底盘角度PID
  * @param  
  * @retval void
  */
 extern ShootNumTypedef ShootNumber;

void Control_ChassisPID(void)
{
	ElectricOutLF = PIDCalc(&ChassisPidLF,RealSpeedLF);
	ElectricOutRF = PIDCalc(&ChassisPidRF,RealSpeedRF);
	ElectricOutRB = PIDCalc(&ChassisPidRB,RealSpeedRB);
	ElectricOutLB = PIDCalc(&ChassisPidLB,RealSpeedLB);
	
	if(sysflag == 1)          //  如果接受到裁判系统的功率数据，步兵底盘功率限制  
	{ 
		sysflag=0;
		
	if(refSysData.PowerHeatData_t.chassisPower > INFPOWER)
	 {
		 if((refSysData.PowerHeatData_t.chassisPowerBuffer>40)&&(refSysData.PowerHeatData_t.chassisPowerBuffer<60))
		 {                                                                 //缓冲热量在40到60之间  速度稍减  0.9
		
			ElectricOutLF *= 0.95;
			ElectricOutRF *= 0.95;
			ElectricOutRB *= 0.95;
			ElectricOutLB *= 0.95;
		 }
		 
		if((refSysData.PowerHeatData_t.chassisPowerBuffer>20)&&(refSysData.PowerHeatData_t.chassisPowerBuffer<40))
		 {                                                                 //缓冲热量在20到40之间   将速度减小些 0.8		
			ElectricOutLF *= 0.85;
			ElectricOutRF *= 0.85;
			ElectricOutRB *= 0.85;
			ElectricOutLB *= 0.85;
		 }
     
		 	if(refSysData.PowerHeatData_t.chassisPowerBuffer<20)
		 {         //缓冲热量在0到20之间   直接把功率限制在80
	   	  PowerCof= INFPOWER / refSysData.PowerHeatData_t.chassisPower;
       
        PowerCof=(PowerCof>1)?1:PowerCof;
				ElectricOutLF *= PowerCof;
				ElectricOutRF *= PowerCof;
				ElectricOutRB *= PowerCof;
				ElectricOutLB *= PowerCof;
		 }
	  }

		 switch (refSysData.GameRobotState_t.maxHP)    //根据不同的热量的上限值来确定步兵的等级
			  {
				 case 750:                                     //一级步兵
					  if(refSysData.PowerHeatData_t.ShooterHeat_17mm<1600-400)
						{
               ShootNumber.AllowShoot=0X00; 
						}
            else
            {
                ShootNumber.AllowShoot=0X01;
            }
            break;		
						
				 case 1000:                                    //二级步兵
					  if(refSysData.PowerHeatData_t.ShooterHeat_17mm<1600)
						{
              ShootNumber.AllowShoot=0X00;
			      }
            else
            {
                ShootNumber.AllowShoot=0X01;
            }
            break;
						
				 case 1500:                                    //三级步兵
					  if(refSysData.PowerHeatData_t.ShooterHeat_17mm<1600)
						{
                ShootNumber.AllowShoot=0X00;
			      }
            else
            {
                ShootNumber.AllowShoot=0X01;
            }
            break;
						
				   default:break;
			   } 	
        
		}

	//将底盘电机的速度解码发送
	MotorTxData[0] = (uint8_t)((ElectricOutLF>>8)&0xFF);
	MotorTxData[1] = (uint8_t)(ElectricOutLF&0xFF); 
	MotorTxData[2] = (uint8_t)((ElectricOutRF>>8)&0xFF);
	MotorTxData[3] = (uint8_t)(ElectricOutRF&0xFF); 
	MotorTxData[4] = (uint8_t)((ElectricOutRB>>8)&0xFF);
	MotorTxData[5] = (uint8_t)(ElectricOutRB&0xFF); 
	MotorTxData[6] = (uint8_t)((ElectricOutLB>>8)&0xFF);
	MotorTxData[7] = (uint8_t)(ElectricOutLB&0xFF); 

	  if(un_control>=12)    //0.3s持续疯转，或者回复状态1.2s仍然没有达到目标速度0，则强制电机输出电流为0
	{	   		
		for(char i=0;i<8;i++)
	       MotorTxData[i]=0; 		 			
	}
  	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
}





