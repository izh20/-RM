
#ifndef _CONTROL_VISON_H
#define _CONTROL_VISON_H

//#include "common.h"

#include "stm32f4xx_HAL.h"

  
void ScanDataVision(unsigned char *str); //解码数据

typedef struct
{
   int16_t PitchErr;
   int16_t YawErr;
  
   int16_t PitchMissHero;   //需要瞄准英雄装甲  基准值 由于像素中心点不是需要的中心点
   int16_t YawMissHero;   
   int16_t PitchMissInfance;   //需要瞄准步兵装甲  基准值 由于像素中心点不是需要的中心点
   int16_t YawMissInfance;   
  
   int8_t   robots;    //0:英雄   1：步兵
   int8_t   mode;  //0：无效  1：近距离模式 2：远距离模式
   int8_t   num;   //九宫格数字
   int8_t   flags;   //0;目标未更新  1：目标更新

}ScanMove;



void ScanDataInit(void);

//extern uint8_t can1_rx_data[8];
//extern uint8_t can2_rx_data[8];
//extern uint8_t MotorTxData[8];
//extern uint8_t HeadTxData[8];
//extern uint8_t TestTxData[8];

//extern uint16_t motor_angle_0x201,motor_angle_0x202,motor_angle_0x203,motor_angle_0x204;
//extern int16_t RealSpeedLF,RealSpeedRF,RealSpeedLB,RealSpeedRB;  //底盘电机的反馈速度

//void CanFilter_Init(CAN_HandleTypeDef* hcan);
//void HAL_CAN_Get(CAN_HandleTypeDef* hcan);
//void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id, uint8_t len);

//extern int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB;

#endif
