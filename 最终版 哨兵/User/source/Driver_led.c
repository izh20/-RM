/**
  *@file Driver_led.c
  *@date 2018/01/06
  *@author ��������
  *@brief 
  */
	
// #include "common.h"
 #include "Driver_led.h"
 #include "sys.h"
 
 //��ͷ�ļ��������������ֱ��ʹ��
//#define LED_Red_On()        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
//#define LED_Red_Off()       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
//#define LED_Red_Toggle()    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7)
//#define LED_Green_On()      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
//#define LED_Green_Off()     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
//#define LED_Green_Toggle()  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14)
//#define KEY_PRESS          ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10))?0x00:0x01)



//����ɨ�����
//����ֵ�� 0x01:���� 0x00:δ���»��ɿ�
//
uint8_t KeyScan()
{
	 static uint8_t key_up=0x01;    //�����ɿ���־λ
	 
	if((key_up&&KEY_PRESS) ==0x01)
	 {
		 		delay_ms(10);
		    key_up=0;
		     if(KEY_PRESS==0X01)
					  return 0x01;		 
	 }
	 else if(KEY_PRESS==0x00) 
		 key_up=0x01;
	 
	 return 0x00;	 
}

 






