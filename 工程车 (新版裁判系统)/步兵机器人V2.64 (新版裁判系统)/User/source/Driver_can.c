/**
  *@file Driver_can.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */

#include "Driver_can.h"
#include "Motor_Chassis.h"
#include "Motor_Shoot.h"

uint8_t can1_rx_data[8];
uint8_t can2_rx_data[8];

uint8_t MotorTxData[8] = {0};    //底盘控制
//uint8_t HeadTxData[8] = {0,0,0,0,0,0,0,0};     //云台控制
uint8_t TestTxData[8] = {0};     //底盘电机遥控控制实验


uint16_t motor_angle_0x201,motor_angle_0x202,motor_angle_0x203,motor_angle_0x204;
int16_t RealSpeedLF,RealSpeedRF,RealSpeedLB,RealSpeedRB;  //底盘电机的反馈速度
int16_t RealSpeedPLUCK=0,RealSpeedRotate,RealSpeedPLUCK3;
uint16_t RealAngleYAW,RealAnglePITCH,RealAnglePLUCK,RealAngleRotate;


//can filter must be initialized before use
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
  static CanTxMsgTypeDef  Tx2Message;
  static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
  }
  if(hcan == &hcan2)
  {
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;
  }
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  
}

extern uint8_t contro_flag,Head_flag;
extern uint16_t aim_angle_206,aim_angle_205;

//获取/刷新can接收数据用
//it will be auto callback when can receive msg completely
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{

	  can1_rx_data[0] = hcan->pRxMsg->Data[0];
	  can1_rx_data[1] = hcan->pRxMsg->Data[1];
	  can1_rx_data[2] = hcan->pRxMsg->Data[2];
	  can1_rx_data[3] = hcan->pRxMsg->Data[3];

	
	 switch(hcan->pRxMsg->StdId)
     {
		 
		 case 0x201:
			{
			  	motor_angle_0x201= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
			    RealSpeedLF= (can1_rx_data[2]<<8)|(can1_rx_data[3]);
				
			}break;
			
			case 0x202:
			{
				motor_angle_0x202= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
				RealSpeedRF= (can1_rx_data[2]<<8)|(can1_rx_data[3]);
				
			}break;
			
			case 0x203:
			{
				motor_angle_0x203= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
				RealSpeedRB= (can1_rx_data[2]<<8)|(can1_rx_data[3]);
			
				
			}break;
			case 0x204:            //底盘LB
			{
				motor_angle_0x204= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
				RealSpeedLB= (can1_rx_data[2]<<8)|(can1_rx_data[3]);
				
			}break;
			
      
// #ifdef ROBOT1
//      		case 0x205:     //航偏角YAW
//    {
//			
//			RealAngleYAW=(can1_rx_data[0]<<8)|(can1_rx_data[1]);
//			
//			
//    }break;
//		case 0x206:  //翻滚角
//    {
//			
//			RealAnglePITCH	= (can1_rx_data[0]<<8)|(can1_rx_data[1]); 
//    }break;
// #elif ROBOT2      
//		case 0x205:     //航偏角YAW
//    {
//			RealAngleYAW=(can1_rx_data[0]<<8)|(can1_rx_data[1]);
//				if(RealAngleYAW<=1500) RealAngleYAW = RealAngleYAW+8180;
//			    RealAngleYAW = RealAngleYAW-2800;
//			
//    }break;
//    
//		case 0x206:  //翻滚角
//    {
//			RealAnglePITCH	= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
//       RealAnglePITCH=6400-RealAnglePITCH;
//    }break;
//#elif ROBOT3
//    case 0x205:     //航偏角YAW
//    {
//			
//			RealAngleYAW=(can1_rx_data[0]<<8)|(can1_rx_data[1]);
//			
//			
//    }break;
//		case 0x206:  //翻滚角
//    {
//			
//			RealAnglePITCH	= (can1_rx_data[0]<<8)|(can1_rx_data[1]); 
//    }break;  
//#endif

   case 0x205:   //旋转电机
    {
			RealAngleRotate	= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
      RealSpeedRotate = (can1_rx_data[2]<<8)|(can1_rx_data[3]);			
    }break;

		case 0x207:  //拨弹电机
    {
			RealAnglePLUCK	= (can1_rx_data[0]<<8)|(can1_rx_data[1]);
      RealSpeedPLUCK=   (can1_rx_data[2]<<8)|(can1_rx_data[3]);
    }break;
    
  }
     
	   //根据接收到的ID号返回数据帧 
    switch(hcan->pRxMsg->StdId)
   {
     case 0x201:
     case 0x202:
     case 0x203:
     case 0x204:
      if(contro_flag==1)
      {
          Control_ChassisPID();
				  Control_PluckPID();          
          contro_flag = 0;   
      }break;
     
//     case 0x205:
//     case 0x206:
//     if(Head_flag==1)  
//     {
////       Control_PitchPID();                   
////       Head_flag=0;
//     }break;
//     
//     case 0x207:
////       ShootControl(RealAnglePLUCK);
//     break;
     default:break;
   }

   
		//		num++;
//		printf("%d %d %d\n", num , RealSpeedLF , GimContro201);
      HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

//CAN send message test
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id, uint8_t len)
{
  uint8_t index = 0;
  
  hcan->pTxMsg->StdId = id;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->DLC = len;
  
  for(index = 0; index <len; index++)
    hcan->pTxMsg->Data[index] = msg[index];
  
  HAL_CAN_Transmit_IT(hcan);

}

