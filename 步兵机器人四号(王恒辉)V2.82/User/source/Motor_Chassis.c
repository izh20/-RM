
//���̵���ջ����Ƴ���
//��������
//2018/1/12
//

#include "Motor_Chassis.h"
#include <stdio.h>  
#include "Motor_Shoot.h"


//PidTypeDef ChassisAngle={0};   //������̬PID
//AngleTypedef  ChassisAngle={0};
PidTypeDef ChassisPidLF={0},ChassisPidRF={0},ChassisPidRB={0},ChassisPidLB={0};

//PidTypeDef YawOPID={0};   //��ƫ��        { 2000, 1.6, 0.05, 10, {0,0,0,0,0}, 0}; 


/**
  * @brief  PID��ʼ������
  * @param  void
  * @retval void
  */

const double KP=7,KI=0.3,KD=5;  //���̵��PID
//int16_t ChassisAngleNow=0;  //�������̽Ƕ�ֵ

//extern int16_t SPEEDTARGET;
int16_t SPEEDTARGET=0;
int16_t AngleMid=0;

void PIDInit(void)
{
//    ChassisAngle.mode=0;  
//    ChassisAngle.AngleMax=450;    //���ұ�45��
//    ChassisAngle.AngleMin=-450;   //�����45��
//    ChassisAngle.AngleAdd=5;      //ÿ��ƫ��0.5�� 
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
	//���̵��LF
	  ChassisPidLF.Kp=KP;
    ChassisPidLF.Ki=KI;
	  ChassisPidLF.Kd=KD;
	 // ChassisPidLF.AddMax=1500;
	  ChassisPidLF.Istart=200;
	  ChassisPidLF.OutMax=CHASSISMAX;
	  ChassisPidLF.SumMax=20000;
  	ChassisPidLF.target=SPEEDTARGET; 

	
		//���̵��RF
	  ChassisPidRF.Kp=KP;
    ChassisPidRF.Ki=KI;
	  ChassisPidRF.Kd=KD;
	  ///ChassisPidRF.AddMax=1500;
	  ChassisPidRF.Istart=200;
	  ChassisPidRF.OutMax=CHASSISMAX;
	  ChassisPidRF.SumMax=20000;
  	ChassisPidRF.target=SPEEDTARGET; 

	
	//���̵��RB
	  ChassisPidRB.Kp=KP;
    ChassisPidRB.Ki=KI;
	  ChassisPidRB.Kd=KD;
		//ChassisPidRB.AddMax=1500;
		ChassisPidRB.Istart=200;
	  ChassisPidRB.OutMax=CHASSISMAX;
	  ChassisPidRB.SumMax=20000;
  	ChassisPidRB.target=SPEEDTARGET; 

			
//���̵��LB			
	  ChassisPidLB.Kp=KP;
    ChassisPidLB.Ki=KI;
	  ChassisPidLB.Kd=KD;
		//ChassisPidLB.AddMax=1500;
		ChassisPidLB.Istart=200;
	  ChassisPidLB.OutMax=CHASSISMAX;
	  ChassisPidLB.SumMax=20000;
  	ChassisPidLB.target=SPEEDTARGET; 	
	
}

//��̬У��
//�м�ֵ 680  
//
//int16_t mid_dir=500;  //���ĽǶ�ֵ
//int16_t SpeedCorrect=0;


//void CorrectDirect(int16_t *moto_ctr)  //ǰ������У��
//{
//  SpeedCorrect=-PIDCalc(&ChassisAngle.ChassisPid,ChassisAngleNow);  
//  moto_ctr[0]=SpeedCorrect;  
//  moto_ctr[1]=SpeedCorrect;  
//  moto_ctr[2]=SpeedCorrect;  
//  moto_ctr[3]=SpeedCorrect;  
//}

//����ģʽ
//void ChassisAuto(int16_t *moto_ctr)  
//{
//  if(ChassisAngle.mode==1)  //��������˹ģʽ
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
extern char sysflag;    //���ڵ��߱�־λ
extern refDataStruct  refSysData;      //�������ݽṹ�壬����ÿ����������ʲô���Ѿ���.h�ļ�����˵��

#define  INFPOWER    78   //�������ƹ���ֵ
#define  HEROPOWER   118   //Ӣ�����ƹ���ֵ

float  PowerCof;
/** 
  * @note   modified
  * @brief  ���̽Ƕ�PID
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
	
	if(sysflag == 1)          //  ������ܵ�����ϵͳ�Ĺ������ݣ��������̹�������  
	{ 
		sysflag=0;
		
	if(refSysData.PowerHeatData_t.chassisPower > INFPOWER)
	 {
		 if((refSysData.PowerHeatData_t.chassisPowerBuffer>40)&&(refSysData.PowerHeatData_t.chassisPowerBuffer<60))
		 {                                                                 //����������40��60֮��  �ٶ��Լ�  0.9
		
			ElectricOutLF *= 0.95;
			ElectricOutRF *= 0.95;
			ElectricOutRB *= 0.95;
			ElectricOutLB *= 0.95;
		 }
		 
		if((refSysData.PowerHeatData_t.chassisPowerBuffer>20)&&(refSysData.PowerHeatData_t.chassisPowerBuffer<40))
		 {                                                                 //����������20��40֮��   ���ٶȼ�СЩ 0.8		
			ElectricOutLF *= 0.85;
			ElectricOutRF *= 0.85;
			ElectricOutRB *= 0.85;
			ElectricOutLB *= 0.85;
		 }
     
		 	if(refSysData.PowerHeatData_t.chassisPowerBuffer<20)
		 {         //����������0��20֮��   ֱ�Ӱѹ���������80
	   	  PowerCof= INFPOWER / refSysData.PowerHeatData_t.chassisPower;
       
        PowerCof=(PowerCof>1)?1:PowerCof;
				ElectricOutLF *= PowerCof;
				ElectricOutRF *= PowerCof;
				ElectricOutRB *= PowerCof;
				ElectricOutLB *= PowerCof;
		 }
	  }

		 switch (refSysData.GameRobotState_t.maxHP)    //���ݲ�ͬ������������ֵ��ȷ�������ĵȼ�
			  {
				 case 750:                                     //һ������
					  if(refSysData.PowerHeatData_t.ShooterHeat_17mm<1600-400)
						{
               ShootNumber.AllowShoot=0X00; 
						}
            else
            {
                ShootNumber.AllowShoot=0X01;
            }
            break;		
						
				 case 1000:                                    //��������
					  if(refSysData.PowerHeatData_t.ShooterHeat_17mm<1600)
						{
              ShootNumber.AllowShoot=0X00;
			      }
            else
            {
                ShootNumber.AllowShoot=0X01;
            }
            break;
						
				 case 1500:                                    //��������
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

	//�����̵�����ٶȽ��뷢��
	MotorTxData[0] = (uint8_t)((ElectricOutLF>>8)&0xFF);
	MotorTxData[1] = (uint8_t)(ElectricOutLF&0xFF); 
	MotorTxData[2] = (uint8_t)((ElectricOutRF>>8)&0xFF);
	MotorTxData[3] = (uint8_t)(ElectricOutRF&0xFF); 
	MotorTxData[4] = (uint8_t)((ElectricOutRB>>8)&0xFF);
	MotorTxData[5] = (uint8_t)(ElectricOutRB&0xFF); 
	MotorTxData[6] = (uint8_t)((ElectricOutLB>>8)&0xFF);
	MotorTxData[7] = (uint8_t)(ElectricOutLB&0xFF); 

	  if(un_control>=12)    //0.3s������ת�����߻ظ�״̬1.2s��Ȼû�дﵽĿ���ٶ�0����ǿ�Ƶ���������Ϊ0
	{	   		
		for(char i=0;i<8;i++)
	       MotorTxData[i]=0; 		 			
	}
  	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
}





