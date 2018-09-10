/**
  *@file test_uart.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__UART_H
#define _TEST__UART_H

#include "stm32f4xx_HAL.h"
#include "common.h"

extern uint8_t uart1_rx_buff[50];
extern uint8_t uart2_rx_buff[15];
extern uint8_t uart3_rx_buff[50];
extern uint8_t uart6_rx_buff[50];
 
extern __IO uint16_t  Uart6_Lenth;
extern uint8_t   UART6_BUFF[];

void UART_Rx_X(UART_HandleTypeDef *huart);


/*��λ��*/
void UART_SendByte (unsigned char dat);
void uart_putbuff (uint8_t *buff, unsigned int len);
void vcan_sendware(uint8_t *wareaddr, unsigned int waresize);
void VcanSendSpeedElectric(void);  //�����ٶ�ֵ�����ֵ
void VcanSendHead(void);//��ӡ��̨���ݵ�
 void VcanStatues(void);
/* var[0] =(uint16)AD_value[1];
   var[1] =(uint16)AD_value[3];
   var[2] =(int16)AD_value[4];
   
   vcan_sendware((uint8_t *)var, sizeof(var));
   
 */ 

#endif

