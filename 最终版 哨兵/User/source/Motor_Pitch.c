#include "Motor_Pitch.h"
#include "Algorithm_pid.h"
#include "Motor_Shoot.h"

int16_t ElectricYaw205,ElectricPitch206; //205 206输出电流值
static int16_t Slide(ImuDataTypedef *imudata,int16_t temp);
extern uint16_t RealAngleYAW,RealAnglePITCH;   //205 206 真实的角度值
extern ShootNumTypedef ShootNumber;   

//初始化值
PidTypeDef PitchOPID={0};   //俯仰角位置外环
PidTypeDef PitchIPID={0};   //俯仰角速度内环
PidTypeDef HeadTargets={0};   //目标外环PID

PidTypeDef YawOPID={0};   //航偏角位置外环
PidTypeDef YawIPID={0};   //航偏角速度内环

ImuDataTypedef data_x={0},data_z={0};
CatTypedef CatStruct;   //

//pitch 3650-4750  yaw  2100-5400


  void PitchPIDInit(void)  
{
  
  
//	//位置外环
	  PitchOPID.Kp=0.6;
	  PitchOPID.Ki=0.0;
	  PitchOPID.Kd=1.0;
	  PitchOPID.Istart=1000;    //积分分离，积分开始的值
	  PitchOPID.SumMax=2500;   //积分限幅
	  PitchOPID.OutMax=3000;   //最大输出电流
	  PitchOPID.target=PITCHANGLEMID;   //目标角度
	
	  //速度内环
	  PitchIPID.Kp=20.0;
	  PitchIPID.Ki=6.5;
	  PitchIPID.Kd=0.0;
	  PitchIPID.Istart=1000;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=1000;  


}

void YawPIDInit(void)  //左右
{
 
  
	  YawOPID.Kp=5;  //12
	  YawOPID.Ki=0;
	  YawOPID.Kd=0;
	  YawOPID.Istart=1000;    //积分分离，积分开始的值
	  YawOPID.SumMax=2000;   //积分限幅
	  YawOPID.OutMax=3000;   //最大输出电流
	  YawOPID.target=YAWANGLEMID;   //中间位置	
	  

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
		//IMU数据读取
     IMU_Get_Data();
             
		YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //航偏电机 位置外环
  //	YawIPID.target=0;
	  ElectricYaw205=PIDCalc(&YawIPID,imu_data.gz);    //速度内环
	
    imu_data.gy=imu_data.gy*0.1;
		PitchIPID.target=-PIDCalc(&PitchOPID,RealAnglePITCH);  //位置外环 由于向下为增大，在此电流需要取反
  //	PitchIPID.target=0;
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gy);    //速度内环
	
	
	if(refSysData.PowerHeatData_t.ShooterHeat_17mm<350)  //  350 手册为360; 
		 ShootNumber.AllowShoot = 0x00;   //可以发弹
	else
		 ShootNumber.AllowShoot = 0x01;

  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}




////滑动滤波 求取平均值
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
