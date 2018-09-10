#ifndef __COMMAN_H
#define __COMMAN_H

//Library header

//#include "stm32f4xx_hal.h"

#include "main.h"


//hardware header
#include "INIT_CAN.h"
#include "INIT_DMA.h"
#include "INIT_GPIO.h"
#include "INIT_SPI.h"
#include "INIT_TIM.h"
#include "INIT_USART.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "system_clock.h"

//Configeration header
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
//#include "Task_control.h"
#include "Driver_DBUS.h"	
#include "Algorithm_pid.h" 
#include "Task_tim.h"
#include "Driver_beep.h"
#include "Driver_can.h"
//#include "Driver_shoot.h"
#include "Driver_usart.h"
#include "Driver_can.h"
//#include "Task_control.h"
#include "Driver_imu.h"
#include "Driver_led.h"
#include "Motor_Pitch.h"
#include "Motor_Chassis.h"

#include "referee_sys.h"




#endif
