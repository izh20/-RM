#ifndef _Driver__TIM_H
#define _Driver__TIM_H

#include "stm32f4xx_HAL.h"
#include "common.h"

void TIM_SetTIM12Compare(uint16_t compare1,uint16_t compare2);
void TIM_SetTIM2Compare(uint16_t bo_compare1,uint16_t bo_compare2);
//void mo_init(void);
void RM2312_Init(void);
//void mocha_shoot(void);
//int16_t StepChange(uint16_t NowAngle);
void ShootControl(uint16_t NowAngle);
void StartShoot(void);

typedef struct 
{
	int8_t   dir;
	int8_t   StartFlag;     //��־λ
	int8_t   Stop;          //�ж��Ƿ񿨵�
	int8_t   Num;            //ת����Ȧ��
	int16_t  StartThreshold;  //�ʼ�ĽǶ�ֵ
	int16_t  Threshold;     //�Ƚϵ���ֵ��ÿ�ο�ʼ�ĽǶ�ֵ	
	int16_t  DeadElectric;  //������ѹ  
	float    Circle;      //1�β��ӵ���Ȧ��
	float    Circles;     //7�ε�Ȧ��
}Shoot;

typedef struct
{
   uint8_t AllowShoot;
   uint16_t Number;        //��Ҫ����Ĵ���
   uint8_t LastFinish;    //��һ�β����Ƿ����
}ShootNumTypedef;

extern ShootNumTypedef ShootNumber;
extern Shoot PluckMotor;



void PluckPIDInit(void);
#endif
