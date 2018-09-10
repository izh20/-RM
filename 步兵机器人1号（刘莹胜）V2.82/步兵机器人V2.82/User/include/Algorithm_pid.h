#ifndef PID_X_H
#define PID_X_H

#include "common.h"
#include "main.h"
#include "config.h"

//#define _Inum 5 //0.1秒内的i

typedef struct{
	float Kp;
	float Ki; 
	float Kd;
	
	float Pout;
	float Iout;
	float Dout;

	int16_t PIDout;//[20];
	//int16_t AddMax;
  int16_t Istart;	
	int16_t target;
	int16_t OutMax;
	
  float lasterror;
  float error;
	float derror;

	float sum; 
	float SumMax; 
	
}PidTypeDef;


typedef struct
{
   PidTypeDef GyHeadPID;   //上部陀螺仪
   int8_t  mode;          //0:手动模式   1:猫步模式
   int8_t   dir;          //0:底盘正向   1:底盘反向
   int16_t AngleMid;      //中间角度值
   int16_t GyTime;
}CatTypedef;

int16_t PIDCalc( PidTypeDef *PID, int16_t Real);

extern short var_2,var_3;

#endif
