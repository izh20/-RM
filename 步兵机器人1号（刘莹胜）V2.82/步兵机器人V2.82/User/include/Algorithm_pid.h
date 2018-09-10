#ifndef PID_X_H
#define PID_X_H

#include "common.h"
#include "main.h"
#include "config.h"

//#define _Inum 5 //0.1���ڵ�i

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
   PidTypeDef GyHeadPID;   //�ϲ�������
   int8_t  mode;          //0:�ֶ�ģʽ   1:è��ģʽ
   int8_t   dir;          //0:��������   1:���̷���
   int16_t AngleMid;      //�м�Ƕ�ֵ
   int16_t GyTime;
}CatTypedef;

int16_t PIDCalc( PidTypeDef *PID, int16_t Real);

extern short var_2,var_3;

#endif
