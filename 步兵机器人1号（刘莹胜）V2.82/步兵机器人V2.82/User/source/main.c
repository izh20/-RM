
#include "common.h"
#include "Motor_Chassis.h"
#include "Motor_Shoot.h"
 #include "Driver_oled.h"
#include "Control_Vision.h"
 #include "INIT_IWDOG.h"       //看门狗
 


//extern uint8_t contro_flag;
extern uint16_t RealAnglePLUCK;
extern int16_t RealSpeedPLUCK,RealSpeedPLUCK2,RealSpeedPLUCK3;
extern uint16_t RealAnglePITCH,RealAngleYAW;

//四个底盘电机的机械角度值和速度值 ID： 201 202 203 204 

extern PidTypeDef YawOPID;
extern PidTypeDef PitchOPID;
extern uint8_t DBUSCheak;
extern RC_Ctl_t RC_Ctl;
//extern StopNum OneStep;
extern int32_t ElectricShoot;
extern int16_t ANGLE;
extern float Circles;
extern Shoot PluckMotor;
extern uint8_t VisionFlag;


int main(void)
{
	/*********************系统运行状态监视***************************/
  HAL_Init();  //初始化硬件库,每1毫秒产生一次中断,时钟来源为内部HSI时钟,频率为16MHZ,将优先级设置为4组4等级  
  SystemClock_Config();   //超时安全处理程序，主要用来监视程序是否死机，并做出处理
	
  /* Initialize all configured peripherals */
  MX_GPIO_Init();  //初始化接口
  MX_DMA_Init();   //DMA传输
  MX_TIM6_Init();      //基本时钟，1ms,作为任务调度的频率
  MX_TIM7_Init();      //基本定时器,1ms,令绿色LED灯闪动
  HAL_TIM_Base_Start_IT(&htim6);  
  HAL_TIM_Base_Start_IT(&htim7); 
  
  MX_CAN1_Init();		//控制电机的CAN总线 (RX CAN_L:PD0   TX:PD1)
  CanFilter_Init(&hcan1);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);	//
	
	
 // MX_TIM2_Init();   //用户拓展口 TIM2   CH1-CH4(PA0  PA1  PA2  PA3)
  MX_TIM4_Init();   //用户拓展口 TIM4   CH1-CH4(PD12 PD13 PD14 PH10 ) 
  MX_TIM5_Init();   //用户拓展口 TIM5   CH1-CH4(PD15 PH11 PH12 PI 0) 
  MX_TIM8_Init();   //用户拓展口 TIM8   CH1-CH4(PI5  PI6  PI7  PI2) 
  MX_TIM12_Init();   //摩擦轮

	//HAL_TIM_Base_Start_IT(&htim12);
  
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	
  //MX_USB_DEVICE_Init();   //拓展USB接口 (PA10 PA11 PA12)
  MX_SPI5_Init();   //MPU6550陀螺仪通信口 (PF6 PF7 PF8 PF9)
 if(MPU_id != 0)
  sTestResult.imuTest = 0x01;
  MPU6500_Init();      //SPI5通信

	 /*****************蜂鸣器接口*******************/	
   MX_TIM3_Init();          //蜂鸣器定时发声 TIM3_CH1(PB4)
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
// /***********************USART通信接口********************/
   MX_USART1_UART_Init();  //遥控器串口接收配置  DBUS RX: PB7
   //MX_USART2_UART_Init();  //裁判系统(用户自己拓展使用) TX:PD5  RX:PD6
   MX_USART3_UART_Init();  //用户拓展串口 TX:PD8 RX:PD9
   MX_USART6_UART_Init();  //prinf函数接口  TX:PG14 RX:PG9
   HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18);
   //HAL_UART_Receive_IT(&huart2, uart2_rx_buff, 8);  
   //HAL_UART_Receive_IT(&huart3, uart3_rx_buff, 8);
   HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
 
	 
	 uint8_t key=0;
	 uint8_t key_flag=0;

	PIDInit();
	PitchPIDInit();
	YawPIDInit();
	DBUS_Init();  
	PluckPIDInit();
	RM2312_Init();   //初始化摩擦轮无刷电机
	ref_sys_init();  //裁判系统初始化
	//LASER_GPIO_Port
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
  ScanDataInit();
   
  OLED_Init();  
  OLED_Clear();
  OLED_ShowString(0,0,"Vision",16);
  IWDG_Init(IWDG_PRESCALER_64,50); 
  
//   
    
	while (1)    
	{

		key=KeyScan();
		
	//	display();
  //  VcanStatues();
		if(key==0x01)
		{	

		//	 StartShoot();
			
      if(key_flag==0)			
      {
    
      }
      else
      {  
      
      }
			 
   }
    
   // VcanSendSpeedElectric();
    switch(RC_Ctl.rc.s2)
    {
      case 1: 
       TIM_SetTIM12Compare(1000,1000);
          break;
      case 3: TIM_SetTIM12Compare(1350,1350);break;
      default: break;  
    }	
    
     
    
    if(VisionFlag==0x01)
    {
       ScanDataVision(UART6_BUFF); 
       VisionFlag=0x00; 
       //Uart6_Lenth=0;
    }
     //  TIM_SetTIM12Compare(0,0);
   // printf("Hello");
     //while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
   //	USART3->DR = (uint8_t) 'S'; 
  
    //delay_ms(1);
   //display();
   //printf("Hello\n"); 
   
      VcanSendHead();
   // VcanStatues(); 
    
   // printf("Hello\n");  
  //  printf("PITCH=%d  YAW=%d\n",RealAnglePITCH,RealAngleYAW);
     
    // RC_Ctl.mouse.x
		//打印遥控值  S1:  1 1 0  S2  上->下  1 3 2
	  //	printf("CH1=%d CH2=%d CH3=%d CH4=%d key1=%d key2=%d\n",RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.ch2,RC_Ctl.rc.ch3,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
		// printf("x=%d y=%d z=%d l=%d r=%d rc=%d\n",RC_Ctl.mouse.x,RC_Ctl.mouse.y,RC_Ctl.mouse.z,RC_Ctl.mouse.press_l,RC_Ctl.mouse.press_r,RC_Ctl.key.v);
   //  printf("x=%d      y=%d      target=%d    RealAngleYAW=%d\n",RC_Ctl.mouse.x,RC_Ctl.mouse.y,PitchOPID.target,RealAngleYAW);
	}
  
}

