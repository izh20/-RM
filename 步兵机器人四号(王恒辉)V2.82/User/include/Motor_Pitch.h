#ifndef _MOTOR_PITCH_H
#define _MOTOR_PITCH_H

#include "common.h"

void PitchPIDInit(void);
void YawPIDInit(void);
void Control_PitchPID(void);

typedef struct 
{
	int16_t imu_num[20];
	int8_t  point;
}ImuDataTypedef;

//extern uint16_t GimAngle206,Angle206_real,GimAngle205,Angle205_real;
//extern int16_t GimContro206,GimContro205;

//void Yuntai_contro(uint16_t angle_206,uint16_t angle_205);
//void Control_ChassisPID(void);
//void PIDInit(void);

#endif

