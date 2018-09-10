
//云台双环
//

#include "Motor_Pitch.h"
#include "Algorithm_pid.h"

int16_t ElectricYaw205,ElectricPitch206; //205 206输出电流值
static int16_t Slide(ImuDataTypedef *imudata,int16_t temp);
extern uint16_t RealAngleYAW,RealAnglePITCH;   //205 206 真实的角度值

//初始化值
PidTypeDef PitchOPID={0};   //俯仰角位置外环
PidTypeDef PitchIPID={0};   //俯仰角速度内环
PidTypeDef HeadTargets={0};   //目标外环PID

PidTypeDef YawOPID={0};   //航偏角位置外环
PidTypeDef YawIPID={0};   //航偏角速度内环
//PidTypeDef CatStruct.GyHeadPID={0}; 

ImuDataTypedef data_x={0},data_z={0};
CatTypedef CatStruct;   //


//兵 3660-4940 上-下
//步兵： 3300-4400  下->上

 // 代替英雄的步兵    Pitch： 1500到2600   中间值1900
//  Yaw:1200到0   8180到4800     // 4700   7160  9520


//#define __LAST     1

#ifdef ROBOT1
  void PitchPIDInit(void)  //上下
{
  
  
//	//位置外环
	  PitchOPID.Kp=20;//12 14
	  PitchOPID.Ki=0.2;//0.3  0.2
	  PitchOPID.Kd=0;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=50;    //积分分离，积分开始的值
	  PitchOPID.SumMax=2500;   //积分限幅
	  PitchOPID.OutMax=3000;   //最大输出电流
	  PitchOPID.target=PITCHANGLEMID;   //目标角度
	
	  //速度内环
	  PitchIPID.Kp=6;//	2.5
	  PitchIPID.Ki=0;//  0
	  PitchIPID.Kd=7;//  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=2000;  

}

//左 6500 右边 2000  中间：4300
void YawPIDInit(void)  //左右
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
//      CatStruct.GyHeadPID.target=0;         //目标角度值
 
     
  
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
	  YawOPID.Istart=70;    //积分分离，积分开始的值
	  YawOPID.SumMax=2000;   //积分限幅
	  YawOPID.OutMax=3000;   //最大输出电流
	  YawOPID.target=YAWANGLEMID;   //中间位置	
	  

    YawIPID.Kp=7;
	  YawIPID.Ki=0;
	  YawIPID.Kd=4;
	  //YawIPID.AddMax=30;
	  YawIPID.Istart=80;
	  YawIPID.OutMax=4000;
	  YawIPID.SumMax=2000;
	  YawIPID.target=0;
}

#elif ROBOT2      //英雄

   void PitchPIDInit(void)  //上下
 {
//	//位置外环
	  PitchOPID.Kp=8;//12 14
	  PitchOPID.Ki=0;//0.3  0.2
	  PitchOPID.Kd=6;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=100;    //积分分离，积分开始的值
	  PitchOPID.SumMax=3000;   //积分限幅
	  PitchOPID.OutMax=4000;   //最大输出电流
	  PitchOPID.target=PITCHANGLEMID;   //目标角度
	
	  //速度内环
	  PitchIPID.Kp=3;//2.5;//3  2.5
	  PitchIPID.Ki=0;//0.1  0
	  PitchIPID.Kd=12;//10  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=4000;
	  PitchIPID.SumMax=2000;

}

void YawPIDInit(void)  //左右
{
	//位置外环
	  YawOPID.Kp=5;
	  YawOPID.Ki=0.2;
	  YawOPID.Kd=0;
	//  YawOPID.AddMax=30;
	  YawOPID.Istart=100;    //积分分离，积分开始的值
	  YawOPID.SumMax=2000;   //积分限幅
	  YawOPID.OutMax=500;   //最大输出电流
	  YawOPID.target=YAWANGLEMID;   //中间位置	
	  
	  //速度内环
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

void PitchPIDInit(void)  //上下
{
//	//位置外环
	  PitchOPID.Kp=8;//12 14
	  PitchOPID.Ki=0;//0.3  0.2
	  PitchOPID.Kd=0;//6  0
	  //PitchOPID.AddMDax=30;
	  PitchOPID.Istart=100;    //积分分离，积分开始的值
	  PitchOPID.SumMax=2000;   //积分限幅
	  PitchOPID.OutMax=3000;   //最大输出电流
	  PitchOPID.target=PITCHANGLEMID;   //目标角度
	
	  //速度内环
	  PitchIPID.Kp=2;//2.5;//3  2.5
	  PitchIPID.Ki=0;//0.1  0
	  PitchIPID.Kd=7;//10  7
	  //PitchIPID.AddMax=30;
	  PitchIPID.Istart=100;
	  PitchIPID.OutMax=3000;
	  PitchIPID.SumMax=2000;
}

void YawPIDInit(void)  //左右
{
	//位置外环
	  YawOPID.Kp=0;
	  YawOPID.Ki=0;
	  YawOPID.Kd=0;
	//  YawOPID.AddMax=30;
	  YawOPID.Istart=100;    //积分分离，积分开始的值
	  YawOPID.SumMax=2000;   //积分限幅
	  YawOPID.OutMax=500;   //最大输出电流
	  YawOPID.target=YAWANGLEMID;   //中间位置	
	  
	  //速度内环
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
		//IMU数据读取
     IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //陀螺仪数据滑动滤波
   
 
	  YawOPID.target=(YawOPID.target>YAWANGLEMAX)?YAWANGLEMAX:YawOPID.target;
    YawOPID.target=(YawOPID.target<YAWANGLEMIN)?YAWANGLEMIN:YawOPID.target; 
             
    YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //航偏电机 位置外环
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //速度内环
	
	  imu_data.gx=Slide(&data_x,imu_data.gx);
	  PitchIPID.target=PIDCalc(&PitchOPID,RealAnglePITCH);  //位置外环 由于向下为增大，在此电流需要取反
	  ElectricPitch206=PIDCalc(&PitchIPID,imu_data.gx);    //速度内环
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}


#elif ROBOT2

void Control_PitchPID(void)
{
		//IMU数据读取
     IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //陀螺仪数据滑动滤波
	  YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //航偏电机 位置外环
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //速度内环
	
	  imu_data.gx=Slide(&data_x,imu_data.gx)+15;
	  PitchIPID.target=-PIDCalc(&PitchOPID,RealAnglePITCH);  //位置外环 由于向下为增大，在此电流需要取反
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gx);    //速度内环
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	 HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}

#elif  ROBOT3
 
void Control_PitchPID(void)
{
		//IMU数据读取
      IMU_Get_Data();
	
	  imu_data.gz=Slide(&data_z,imu_data.gz);   //陀螺仪数据滑动滤波
	  YawIPID.target=-PIDCalc(&YawOPID,RealAngleYAW);  //航偏电机 位置外环
	  ElectricYaw205=-PIDCalc(&YawIPID,imu_data.gz);    //速度内环
	
	  imu_data.gx=Slide(&data_x,imu_data.gx);
	  PitchIPID.target=PIDCalc(&PitchOPID,RealAnglePITCH);  //位置外环 由于向下为增大，在此电流需要取反
	  ElectricPitch206=-PIDCalc(&PitchIPID,imu_data.gx);    //速度内环
  
	  HeadTxData[0]=(uint8_t)((ElectricYaw205>>8)&0xFF);
   	 HeadTxData[1]=(uint8_t)(ElectricYaw205&0xFF);           //205   
	  
	  HeadTxData[2]=(uint8_t)((ElectricPitch206>>8)&0xFF);
   	HeadTxData[3]=(uint8_t)(ElectricPitch206&0xFF);           //206  Pitch 
	
	  CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);
}


#endif


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
