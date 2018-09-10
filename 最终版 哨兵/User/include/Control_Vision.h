
#ifndef _CONTROL_VISON_H
#define _CONTROL_VISON_H

//#include "common.h"

#include "stm32f4xx_HAL.h"

  
void ScanDataVision(uint8_t *str); //��������

typedef struct
{
   int16_t PitchErr;
   int16_t YawErr;
  
   int16_t PitchMissInfance;   //��Ҫ��׼����װ��  ��׼ֵ �����������ĵ㲻����Ҫ�����ĵ�
   int16_t YawMissInfance;   
  
   int8_t   robots;    //0:Ӣ��   1������
   int8_t   mode;  //0����Ч  1��������ģʽ 2��Զ����ģʽ
   int8_t   num;   //�Ź�������
   int8_t   flags;   //0;Ŀ��δ����  1��Ŀ�����

}ScanMove;



void ScanDataInit(void);


#endif
