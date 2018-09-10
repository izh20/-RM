
//��̨˫��
//

#include "Motor_Pitch.h"
#include "Algorithm_pid.h"

int16_t ElectricYaw205,ElectricPitch206; //205 206�������ֵ
static int16_t Slide(ImuDataTypedef *imudata,int16_t temp);
extern uint16_t RealAngleYAW,RealAnglePITCH;   //205 206 ��ʵ�ĽǶ�ֵ

//��ʼ��ֵ
PidTypeDef PitchOPID={0};   //������λ���⻷
PidTypeDef PitchIPID={0};   //�������ٶ��ڻ�
PidTypeDef HeadTargets={0};   //Ŀ���⻷PID

PidTypeDef YawOPID={0};   //��ƫ��λ���⻷
PidTypeDef YawIPID={0};   //��ƫ���ٶ��ڻ�
//PidTypeDef CatStruct.GyHeadPID={0}; 

ImuDataTypedef data_x={0},data_z={0};
CatTypedef CatStruct;   //


//�� 3660-4940 ��-��
//������ 3300-4400  ��->��

 // ����Ӣ�۵Ĳ���    Pitch�� 1500��2600   �м�ֵ1900
//  Yaw:1200��0   8180��4800     // 4700   7160  9520


//#define __LAST     1

#ifdef ROBOT1
  void PitchPIDInit(void)  //����
{
  
  
//	//λ���⻷
	  PitchOPID.Kp=20;//12 14
	  PitchOPID.Ki=0.2;//0.3  0.2
	  PitchOPID.Kd=0;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=50;    //���ַ��룬���ֿ�ʼ��ֵ
	  PitchOPID.SumMax=2500;   //�����޷�
	  PitchOPID.OutMax=3000;   //����������
	  PitchOPID.target=PITCHANGLEMID;   //Ŀ��Ƕ�
	
	  //�ٶ��ڻ�
	  PitchIPID.Kp=6;//	2.5
	  PitchIPID.Ki=0;//  0
	  PitchIPID.Kd=7;//  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=2000;  

}

//�� 6500 �ұ� 2000  �м䣺4300
void YawPIDInit(void)  //����
{
  
//      CatStruct.mode=0;
//      CatStruct.dir=0;       
//      CatStruct.AngleMid=0;
//      CatStruct.GyTime=0;
//  
//      CatStruct.GyHeadPID.Kp=9;
//      CatStruct.GyHeadPID.Ki=0.4;
//      CatStruct.GyHeadPID.Kd=10;
//      CatStruct.GyHeadPID.Istart=100;
//      CatStruct.GyHeadPID.SumMax=4000;
//      CatStruct.GyHeadPID.OutMax=5000;  
//      CatStruct.GyHeadPID.target=0;         //Ŀ��Ƕ�ֵ
 
     
  
//   HeadTargets.Kp=2;
//   HeadTargets.Ki=0;
//   HeadTargets.Kd=0;
//   HeadTargets.Istart=100;
//   HeadTargets.SumMax=2000;
//   HeadTargets.OutMax=7000;
//   HeadTargets.target=YAWANGLEMID;
//  
  
	  YawOPID.Kp=25;  //12
	  YawOPID.Ki=0.15;
	  YawOPID.Kd=0;
	//  YawOPID.AddMax=30;
	  YawOPID.Istart=70;    //���ַ��룬���ֿ�ʼ��ֵ
	  YawOPID.SumMax=2000;   //�����޷�
	  YawOPID.OutMax=3000;   //����������
	  YawOPID.target=YAWANGLEMID;   //�м�λ��	
	  

    YawIPID.Kp=7;
	  YawIPID.Ki=0;
	  YawIPID.Kd=4;
	  //YawIPID.AddMax=30;
	  YawIPID.Istart=80;
	  YawIPID.OutMax=4000;
	  YawIPID.SumMax=2000;
	  YawIPID.target=0;
}

#elif ROBOT2      //Ӣ��

   void PitchPIDInit(void)  //����
 {
//	//λ���⻷
	  PitchOPID.Kp=8;//12 14
	  PitchOPID.Ki=0;//0.3  0.2
	  PitchOPID.Kd=6;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=100;    //���ַ��룬���ֿ�ʼ��ֵ
	  PitchOPID.SumMax=3000;   //�����޷�
	  PitchOPID.OutMax=4000;   //����������
	  PitchOPID.target=PITCHANGLEMID;   //Ŀ��Ƕ�
	
	  //�ٶ��ڻ�
	  PitchIPID.Kp=3;//2.5;//3  2.5
	  PitchIPID.Ki=0;//0.1  0
	  PitchIPID.Kd=12;//10  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=2000;

}

void YawPIDInit(void)  //����
{
	//λ���⻷
	  YawOPID.Kp=5;
	  YawOPID.Ki=0.2;
	  YawOPID.Kd=0;
	//  YawOPID.AddMax=30;
	  YawOPID.Istart=100;    //���ַ��룬���ֿ�ʼ��ֵ
	  YawOPID.SumMax=2000;   //�����޷�
	  YawOPID.OutMax=500;   //����������
	  YawOPID.target=YAWANGLEMID;   //�м�λ��	
	  
	  //�ٶ��ڻ�
	  YawIPID.Kp=4.0;
	  YawIPID.Ki=0;
	  YawIPID.Kd=0;
	  //YawIPID.AddMax=30;
	  YawIPID.Istart=80;
	  YawIPID.OutMax=4000;
	  YawIPID.SumMax=2000;
	  YawIPID.target=0;
}

#elif ROBOT3

void PitchPIDInit(void)  //����
{
//	//λ���⻷
	  PitchOPID.Kp=8;//12 14
	  PitchOPID.Ki=0;//0.3  0.2
	  PitchOPID.Kd=0;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=100;    //���ַ��룬���ֿ�ʼ��ֵ
	  PitchOPID.SumMax=2000;   //�����޷�
	  PitchOPID.OutMax=3000;   //����������
	  PitchOPID.target=PITCHANGLEMID;   //Ŀ��Ƕ�
	
	  //�ٶ��ڻ�
	  PitchIPID.Kp=2;//2.5;//3  2.5
	  PitchIPID.Ki=0;//0.1  0
	  PitchIPID.Kd=7;//10  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=3000;
	  PitchIPID.SumMax=2000;
}

void YawPIDInit(void)  //����
{
	//λ���⻷
	  YawOPID.Kp=0;
	  YawOPID.Ki=0;
	  YawOPID.Kd=0;
	//  YawOPID.AddMax=30;
	  YawOPID.Istart=100;    //���ַ��룬���ֿ�ʼ��ֵ
	  YawOPID.SumMax=2000;   //�����޷�
	  YawOPID.OutMax=500;   //����������
	  YawOPID.target=YAWANGLEMID;   //�м�λ��	
	  
	  //�ٶ��ڻ�
	  YawIPID.Kp=0;
	  YawIPID.Ki=0;
	  YawIPID.Kd=0;
	  //YawIPID.AddMax=30;
	  YawIPID.Istart=80;
	  YawIPID.OutMax=4000;
	  YawIPID.SumMax=2000;
	  YawIPID.target=0;
}



#endif
extern IMUDataTypedef imu_data;

#ifdef ROBOT1
 
void Control_PitchPID(void)
{
		//IMU���ݶ�ȡ
     IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //���������ݻ����˲�
   
 
	  YawOPID.target=(YawOPID.target>YAWANGLEMAX)?YAWANGLEMAX:YawOPID.target;
    YawOPID.target=(YawOPID.target<YAWANGLEMIN)?YAWANGLEMIN:YawOPID.target; 
             
    YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //��ƫ��� λ���⻷
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //�ٶ��ڻ�
	
	  imu_data.gx=Slide(&data_x,imu_data.gx);
	  PitchIPID.target=PIDCalc(&PitchOPID,RealAnglePITCH);  //λ���⻷ ��������Ϊ�����ڴ˵�����Ҫȡ��
	  ElectricPitch206=PIDCalc(&PitchIPID,imu_data.gx);    //�ٶ��ڻ�
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}


#elif ROBOT2

void Control_PitchPID(void)
{
		//IMU���ݶ�ȡ
     IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //���������ݻ����˲�
	  YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //��ƫ��� λ���⻷
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //�ٶ��ڻ�
	
	  imu_data.gx=Slide(&data_x,imu_data.gx)+15;
	  PitchIPID.target=-PIDCalc(&PitchOPID,RealAnglePITCH);  //λ���⻷ ��������Ϊ�����ڴ˵�����Ҫȡ��
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gx);    //�ٶ��ڻ�
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	 HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}

#elif  ROBOT3
 
void Control_PitchPID(void)
{
		//IMU���ݶ�ȡ
      IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //���������ݻ����˲�
	  YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //��ƫ��� λ���⻷
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //�ٶ��ڻ�
	
	  imu_data.gx=Slide(&data_x,imu_data.gx);
	  PitchIPID.target=PIDCalc(&PitchOPID,RealAnglePITCH);  //λ���⻷ ��������Ϊ�����ڴ˵�����Ҫȡ��
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gx);    //�ٶ��ڻ�
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	 HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}


#endif


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
