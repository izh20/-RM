
//���̵���ջ����Ƴ���
//��������
//2018/1/12
//

#include "Motor_Chassis.h"
#include <stdio.h>  


PidTypeDef ChassisPidLF={0},ChassisPidRF={0},ChassisPidRB={0},ChassisPidLB={0};
PidTypeDef PluckPID={0};   //���������
PidTypeDef RotatePID={0};
extern uint16_t RealSpeedPLUCK,RealSpeedRotate;
//PidTypeDef YawOPID={0};   //��ƫ��        { 2000, 1.6, 0.05, 10, {0,0,0,0,0}, 0}; 

uint8_t HeadTxData[8] = {0,0,0,0,0,0,0,0};     //�����������ת���

/**
  * @brief  PID��ʼ������
  * @param  void
  * @retval void
  */

const double KP=7,KI=0.3,KD=0;  //���̵��PID
int16_t ANGLE=0;

//extern int16_t SPEEDTARGET;
int16_t SPEEDTARGET=0;
void PIDInit(void)
{
	//  char i=0;
	//���̵��LF
	  ChassisPidLF.Kp=KP;
    ChassisPidLF.Ki=KI;
	  ChassisPidLF.Kd=KD;
	 // ChassisPidLF.AddMax=1500;
	  ChassisPidLF.Istart=200;
	  ChassisPidLF.OutMax=2000;
	  ChassisPidLF.SumMax=20000;
  	ChassisPidLF.target=SPEEDTARGET; 

	
		//���̵��RF
	  ChassisPidRF.Kp=KP;
    ChassisPidRF.Ki=KI;
	  ChassisPidRF.Kd=KD;
	  ///ChassisPidRF.AddMax=1500;
	  ChassisPidRF.Istart=200;
	  ChassisPidRF.OutMax=2000;
	  ChassisPidRF.SumMax=20000;
  	ChassisPidRF.target=SPEEDTARGET; 

	
	//���̵��RB
	  ChassisPidRB.Kp=KP;
    ChassisPidRB.Ki=KI;
	  ChassisPidRB.Kd=KD;
		//ChassisPidRB.AddMax=1500;
		ChassisPidRB.Istart=200;
	  ChassisPidRB.OutMax=2000;
	  ChassisPidRB.SumMax=20000;
  	ChassisPidRB.target=SPEEDTARGET; 

			
//���̵��LB			
	  ChassisPidLB.Kp=KP;
    ChassisPidLB.Ki=KI;
	  ChassisPidLB.Kd=KD;
		//ChassisPidLB.AddMax=1500;
		ChassisPidLB.Istart=200;
	  ChassisPidLB.OutMax=2000;
	  ChassisPidLB.SumMax=20000;
  	ChassisPidLB.target=SPEEDTARGET; 	
	
}

void Pluckmotor_PIDInit(void)
{
   
		 PluckPID.Kp=7;
     PluckPID.Ki=0.3;
	   PluckPID.Kd=0;
	   PluckPID.Istart=200;
	   PluckPID.OutMax=2000;
	   PluckPID.SumMax=20000;
  	 PluckPID.target=0; 
	
		 RotatePID.Kp=7;
     RotatePID.Ki=0.3;
	   RotatePID.Kd=0;
	   RotatePID.Istart=200;
	   RotatePID.OutMax=2000;
	   RotatePID.SumMax=20000;
  	 RotatePID.target=0; 
	
}



int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 
extern uint8_t un_control;
/** 
  * @note   modified
  * @brief  ���̽Ƕ�PID
  * @param  
  * @retval void
  */
void Control_ChassisPID(void)
{
	ElectricOutLF = PIDCalc(&ChassisPidLF,RealSpeedLF);
	ElectricOutRF = PIDCalc(&ChassisPidRF,RealSpeedRF);
	ElectricOutRB = PIDCalc(&ChassisPidRB,RealSpeedRB);
	ElectricOutLB = PIDCalc(&ChassisPidLB,RealSpeedLB);

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



uint16_t ElectricPluck207,ElectricRotate205;
void Control_PluckPID(void)
{
	
//	  imu_data.gx=Slide(&data_x,imu_data.gx);
//	  PitchIPID.target=PIDCalc(&PitchOPID,RealAnglePITCH);  //λ���⻷ ��������Ϊ�����ڴ˵�����Ҫȡ��
//	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gx);    //�ٶ��ڻ�
	
	   ElectricRotate205=PIDCalc(&RotatePID,RealSpeedRotate);
     ElectricPluck207=PIDCalc(&PluckPID,RealSpeedPLUCK);
	
	   HeadTxData[4]=(uint8_t)((ElectricPluck207>>8)&0xFF);
   	 HeadTxData[5]=(uint8_t)(ElectricPluck207&0xFF);           //207  Pluck
	  
	  HeadTxData[0]=(uint8_t)((ElectricRotate205>>8)&0xFF);
   	HeadTxData[1]=(uint8_t)(ElectricRotate205&0xFF);           //205  Rotate 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}


