
#include "common.h"
#include "Motor_Chassis.h"
#include "Motor_Shoot.h"
 #include "Driver_oled.h"
#include "Control_Vision.h"
 #include "INIT_IWDOG.h"       //���Ź�
 


//extern uint8_t contro_flag;
extern uint16_t RealAnglePLUCK;
extern int16_t RealSpeedPLUCK,RealSpeedPLUCK2,RealSpeedPLUCK3;
extern uint16_t RealAnglePITCH,RealAngleYAW;

//�ĸ����̵���Ļ�е�Ƕ�ֵ���ٶ�ֵ ID�� 201 202 203 204 

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
	/*********************ϵͳ����״̬����***************************/
  HAL_Init();  //��ʼ��Ӳ����,ÿ1�������һ���ж�,ʱ����ԴΪ�ڲ�HSIʱ��,Ƶ��Ϊ16MHZ,�����ȼ�����Ϊ4��4�ȼ�  
  SystemClock_Config();   //��ʱ��ȫ���������Ҫ�������ӳ����Ƿ�����������������
	
  /* Initialize all configured peripherals */
  MX_GPIO_Init();  //��ʼ���ӿ�
  MX_DMA_Init();   //DMA����
  MX_TIM6_Init();      //����ʱ�ӣ�1ms,��Ϊ������ȵ�Ƶ��
  MX_TIM7_Init();      //������ʱ��,1ms,����ɫLED������
  HAL_TIM_Base_Start_IT(&htim6);  
  HAL_TIM_Base_Start_IT(&htim7); 
  
  MX_CAN1_Init();		//���Ƶ����CAN���� (RX CAN_L:PD0   TX:PD1)
  CanFilter_Init(&hcan1);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);	//
	
	
 // MX_TIM2_Init();   //�û���չ�� TIM2   CH1-CH4(PA0  PA1  PA2  PA3)
  MX_TIM4_Init();   //�û���չ�� TIM4   CH1-CH4(PD12 PD13 PD14 PH10 ) 
  MX_TIM5_Init();   //�û���չ�� TIM5   CH1-CH4(PD15 PH11 PH12 PI 0) 
  MX_TIM8_Init();   //�û���չ�� TIM8   CH1-CH4(PI5  PI6  PI7  PI2) 
  MX_TIM12_Init();   //Ħ����

	//HAL_TIM_Base_Start_IT(&htim12);
  
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	
  //MX_USB_DEVICE_Init();   //��չUSB�ӿ� (PA10 PA11 PA12)
  MX_SPI5_Init();   //MPU6550������ͨ�ſ� (PF6 PF7 PF8 PF9)
 if(MPU_id != 0)
  sTestResult.imuTest = 0x01;
  MPU6500_Init();      //SPI5ͨ��

	 /*****************�������ӿ�*******************/	
   MX_TIM3_Init();          //��������ʱ���� TIM3_CH1(PB4)
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
// /***********************USARTͨ�Žӿ�********************/
   MX_USART1_UART_Init();  //ң�������ڽ�������  DBUS RX: PB7
   //MX_USART2_UART_Init();  //����ϵͳ(�û��Լ���չʹ��) TX:PD5  RX:PD6
   MX_USART3_UART_Init();  //�û���չ���� TX:PD8 RX:PD9
   MX_USART6_UART_Init();  //prinf�����ӿ�  TX:PG14 RX:PG9
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
	RM2312_Init();   //��ʼ��Ħ������ˢ���
	ref_sys_init();  //����ϵͳ��ʼ��
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
     //while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
   //	USART3->DR = (uint8_t) 'S'; 
  
    //delay_ms(1);
   //display();
   //printf("Hello\n"); 
   
      VcanSendHead();
   // VcanStatues(); 
    
   // printf("Hello\n");  
  //  printf("PITCH=%d  YAW=%d\n",RealAnglePITCH,RealAngleYAW);
     
    // RC_Ctl.mouse.x
		//��ӡң��ֵ  S1:  1 1 0  S2  ��->��  1 3 2
	  //	printf("CH1=%d CH2=%d CH3=%d CH4=%d key1=%d key2=%d\n",RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.ch2,RC_Ctl.rc.ch3,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
		// printf("x=%d y=%d z=%d l=%d r=%d rc=%d\n",RC_Ctl.mouse.x,RC_Ctl.mouse.y,RC_Ctl.mouse.z,RC_Ctl.mouse.press_l,RC_Ctl.mouse.press_r,RC_Ctl.key.v);
   //  printf("x=%d      y=%d      target=%d    RealAngleYAW=%d\n",RC_Ctl.mouse.x,RC_Ctl.mouse.y,PitchOPID.target,RealAngleYAW);
	}
  
}

