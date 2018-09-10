
#ifndef _CONTROL_VISON_H
#define _CONTROL_VISON_H

//#include "common.h"

#include "stm32f4xx_HAL.h"

  
void ScanDataVision(char *str); //��������

typedef struct
{
   int16_t PitchErr;
   int16_t YawErr;
   float   pitch_move;
   float   yaw_move; 
   int8_t   mode;  //0����Ч  1��������ģʽ 2��Զ����ģʽ
   int8_t   num;   //�Ź�������
}ScanMove;

//extern uint8_t can1_rx_data[8];
//extern uint8_t can2_rx_data[8];
//extern uint8_t MotorTxData[8];
//extern uint8_t HeadTxData[8];
//extern uint8_t TestTxData[8];

//extern uint16_t motor_angle_0x201,motor_angle_0x202,motor_angle_0x203,motor_angle_0x204;
//extern int16_t RealSpeedLF,RealSpeedRF,RealSpeedLB,RealSpeedRB;  //���̵���ķ����ٶ�

//void CanFilter_Init(CAN_HandleTypeDef* hcan);
//void HAL_CAN_Get(CAN_HandleTypeDef* hcan);
//void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id, uint8_t len);

//extern int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB;

#endif
