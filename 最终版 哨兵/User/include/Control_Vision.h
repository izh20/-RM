
#ifndef _CONTROL_VISON_H
#define _CONTROL_VISON_H

//#include "common.h"

#include "stm32f4xx_HAL.h"

  
void ScanDataVision(uint8_t *str); //解码数据

typedef struct
{
   int16_t PitchErr;
   int16_t YawErr;
  
   int16_t PitchMissInfance;   //需要瞄准步兵装甲  基准值 由于像素中心点不是需要的中心点
   int16_t YawMissInfance;   
  
   int8_t   robots;    //0:英雄   1：步兵
   int8_t   mode;  //0：无效  1：近距离模式 2：远距离模式
   int8_t   num;   //九宫格数字
   int8_t   flags;   //0;目标未更新  1：目标更新

}ScanMove;



void ScanDataInit(void);


#endif
