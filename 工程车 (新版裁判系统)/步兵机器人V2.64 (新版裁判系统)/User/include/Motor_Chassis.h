#ifndef YUNTAI_X_H
#define YUNTAI_X_H

#include "common.h"


//extern uint16_t GimAngle206,Angle206_real,GimAngle205,Angle205_real;
//extern int16_t GimContro206,GimContro205;

//void Yuntai_contro(uint16_t angle_206,uint16_t angle_205);
void Control_ChassisPID(void);
void PIDInit(void);
void Pluckmotor_PIDInit(void);
void Control_PluckPID(void);
#endif

