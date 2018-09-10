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
	int8_t   StartFlag;     //标志位
	int8_t   Stop;          //判断是否卡弹
	int8_t   Num;            //转过的圈数
	int16_t  StartThreshold;  //最开始的角度值
	int16_t  Threshold;     //比较的阈值，每次开始的角度值	
	int16_t  DeadElectric;  //死区电压  
	float    Circle;      //1次拨子弹的圈数
	float    Circles;     //7次的圈数
}Shoot;

typedef struct
{
   uint8_t AllowShoot;
   uint16_t Number;        //需要射击的次数
   uint8_t LastFinish;    //上一次拨弹是否结束
}ShootNumTypedef;

extern ShootNumTypedef ShootNumber;
extern Shoot PluckMotor;



void PluckPIDInit(void);
#endif
