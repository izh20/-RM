/**
  *@file Driver_led.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _Driver__LED_H
#define _Driver__LED_H

#include "stm32f4xx_hal.h"
#include "common.h"

#define LED_Red_On()        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
#define LED_Red_Off()       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
#define LED_Red_Toggle()    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7)
#define LED_Green_On()      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_Green_Off()     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_Green_Toggle()  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14)
#define KEY_PRESS          ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10))?0x00:0x01)
//按键扫描程序
//返回值： 0x01:按下 0x00:未按下或松开
uint8_t KeyScan(void);
#endif

