#include "Motor_Pitch.h"
#include "Algorithm_pid.h"
#include "Motor_Shoot.h"

int16_t ElectricYaw205,ElectricPitch206; //205 206�������ֵ
static int16_t Slide(ImuDataTypedef *imudata,int16_t temp);
extern uint16_t RealAngleYAW,RealAnglePITCH;   //205 206 ��ʵ�ĽǶ�ֵ
extern ShootNumTypedef ShootNumber;   

//��ʼ��ֵ
PidTypeDef PitchOPID={0};   //������λ���⻷
PidTypeDef PitchIPID={0};   //�������ٶ��ڻ�
PidTypeDef HeadTargets={0};   //Ŀ���⻷PID

PidTypeDef YawOPID={0};   //��ƫ��λ���⻷
PidTypeDef YawIPID={0};   //��ƫ���ٶ��ڻ�

ImuDataTypedef data_x={0},data_z={0};
CatTypedef CatStruct;   //

//pitch 3650-4750  yaw  2100-5400


  void PitchPIDInit(void)  
{
  
  
//	//λ���⻷
	  PitchOPID.Kp=0.6;
	  PitchOPID.Ki=0.0;
	  PitchOPID.Kd=1.0;
	  PitchOPID.Istart=1000;    //���ַ��룬���ֿ�ʼ��ֵ
	  PitchOPID.SumMax=2500;   //�����޷�
	  PitchOPID.OutMax=3000;   //����������
	  PitchOPID.target=PITCHANGLEMID;   //Ŀ��Ƕ�
	
	  //�ٶ��ڻ�
	  PitchIPID.Kp=20.0;
	  PitchIPID.Ki=6.5;
	  PitchIPID.Kd=0.0;
	  PitchIPID.Istart=1000;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=1000;  


}

void YawPIDInit(void)  //����
{
 
  
	  YawOPID.Kp=5;  //12
	  YawOPID.Ki=0;
	  YawOPID.Kd=0;
	  YawOPID.Istart=1000;    //���ַ��룬���ֿ�ʼ��ֵ
	  YawOPID.SumMax=2000;   //�����޷�
	  YawOPID.OutMax=3000;   //����������
	  YawOPID.target=YAWANGLEMID;   //�м�λ��	
	  

    YawIPID.Kp=1.6;
	  YawIPID.Ki=0;
	  YawIPID.Kd=0;
	  YawIPID.Istart=1000;
	  YawIPID.OutMax=4000;
	  YawIPID.SumMax=1000;
	  
}


extern IMUDataTypedef imu_data;


void Control_PitchPID(void)
{
		//IMU���ݶ�ȡ
     IMU_Get_Data();
             
		YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //��ƫ��� λ���⻷
  //	YawIPID.target=0;
	  ElectricYaw205=PIDCalc(&YawIPID,imu_data.gz);    //�ٶ��ڻ�
	
    imu_data.gy=imu_data.gy*0.1;
		PitchIPID.target=-PIDCalc(&PitchOPID,RealAnglePITCH);  //λ���⻷ ��������Ϊ�����ڴ˵�����Ҫȡ��
  //	PitchIPID.target=0;
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gy);    //�ٶ��ڻ�
	
	
	if(refSysData.PowerHeatData_t.ShooterHeat_17mm<350)  //  350 �ֲ�Ϊ360; 
		 ShootNumber.AllowShoot = 0x00;   //���Է���
	else
		 ShootNumber.AllowShoot = 0x01;

  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}




////�����˲� ��ȡƽ��ֵ
const int N=2;
static int16_t Slide(ImuDataTypedef *imudata,int16_t temp)
{
	uint8_t i=0;
	int32_t result=0; 
	
  if(imudata->point==N)
	{
		imudata->point=0;
	}
	imudata->imu_num[imudata->point]=temp;
	imudata->point++;
  
	for(i=0;i<N;i++)
     result+=imudata->imu_num[i];
  
	return result/N;
}
