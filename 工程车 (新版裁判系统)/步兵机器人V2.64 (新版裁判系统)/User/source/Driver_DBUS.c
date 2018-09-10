#include "Driver_DBUS.h"
#include "Motor_Shoot.h"
#include "Driver_usart.h"
#include "Control_Vision.h"
   
//ң����Ҫ�˲��ֳ���
#define Speed_Coefficient 8   //ң���ٶ�ϵ��
#define Left_Coefficient  6   //��תϵ��
#define Right_Coefficient 8   //��תϵ��

//���̿���ϵ��
#define KEY_Coefficient 2


#define IS ==

RC_Ctl_t RC_Ctl;
int16_t buff_zero[4],buff_one[4],buff_two[4],buff_thr[4],moto_ctr[6];
uint16_t aim_angle_206 = 4250;
uint16_t aim_angle_205 = 3172;
double mouse_206,mouse_205;
uint8_t mouse_flag,reversal_flag=0;
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
extern int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 
extern PidTypeDef PitchOPID;  //������λ���⻷
extern PidTypeDef YawOPID;
extern PidTypeDef PluckPID;   //���������
extern PidTypeDef RotatePID;
extern int16_t RealSpeedPLUCK;
//ң�ؽ��뺯��
//�ڽ�����Ϻ��Զ�����
//�����ļ�: Driver_uart.c
void Yaokong_translate(void)
{
    RC_Ctl.rc.ch0 = (uart1_rx_buff[0]| (uart1_rx_buff[1] << 8)) & 0x07ff;	//!< Channel 0
    RC_Ctl.rc.ch1 = ((uart1_rx_buff[1] >> 3) | (uart1_rx_buff[2] << 5)) & 0x07ff;	//!< Channel 1
    RC_Ctl.rc.ch2 = (((int16_t)uart1_rx_buff[2] >> 6) | ((int16_t)uart1_rx_buff[3] << 2) | ((int16_t)uart1_rx_buff[4] << 10)) & 0x07ff;	//!< Channel 2
    RC_Ctl.rc.ch3 = ((uart1_rx_buff[4] >> 1) | (uart1_rx_buff[5] << 7)) & 0x07ff;	//!< Channel 3
    RC_Ctl.rc.s1	= ((uart1_rx_buff[5] >> 4)& 0x000C) >> 2;	//!< Switch left
    RC_Ctl.rc.s2	= ((uart1_rx_buff[5] >> 4)& 0x0003);	//!< Switch right
    RC_Ctl.mouse.x = uart1_rx_buff[6] | (uart1_rx_buff[7] << 8); //!< Mouse X axis
    RC_Ctl.mouse.y = uart1_rx_buff[8] | (uart1_rx_buff[9] << 8); //!< Mouse Y axis
    RC_Ctl.mouse.z = uart1_rx_buff[10] | (uart1_rx_buff[11] << 8);  //!< Mouse Z axis
    RC_Ctl.mouse.press_l = uart1_rx_buff[12]; //!< Mouse Left Is Press ? 
    RC_Ctl.mouse.press_r = uart1_rx_buff[13]; //!< Mouse Right Is Press ?
    RC_Ctl.key.v=uart1_rx_buff[14]|uart1_rx_buff[15]<<8;
}

//���ң��������
void DBUS_Init()
{
	      RC_Ctl.rc.ch0 = 1024;
				RC_Ctl.rc.ch1 = 1024;
				RC_Ctl.rc.ch2 = 1024;
      	RC_Ctl.rc.ch3 = 1024;
				RC_Ctl.rc.s1	= 0;
				RC_Ctl.rc.s2	= 0;
				RC_Ctl.mouse.x =0;
				RC_Ctl.mouse.y = 0;
				RC_Ctl.mouse.z = 0;
				RC_Ctl.mouse.press_l =0; 
				RC_Ctl.mouse.press_r =0;
	      RC_Ctl.key.v=0;
}

/******************************************************
                      ���̵������2017(��һ��)

1, �����������̬�����
2��������̬�ĵ���
3��׼��CAN���������
4����ʼ�������
__________________________________________________________________
| ���λ�� |�����| ���\���� | ǰ | �� | �� | �� | ˳ | �� | ͣ | 
``````````````````````````````````````````````````````````````````
|   ��ǰ   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      
``````````````````````````````````````````````````````````````````
|   ��ǰ   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|   ���   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|   �Һ�   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|  ��ע:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
//ң�ؿ���
const int THRESHOLD=3000;  //ת����ֵ
const int TURNSPEED=3000;   //ת�伫���ٶ�

//���̿���
#define SPEED_S       0
#define SPEED_NORMAL   6000    //����ǰ���ٶ�

//��ϰ����������б����  3000:1500 22.5����ǰ   3000:0    45��        3000��-3000��ת  
#define SPEED_Y        3500     //ǰ�����ٶ�
#define SPEED_X        3000      //�����ٶ�


//ǰ  >0    <0  	<0  	>0
//��ƽ��    <0    <0  	>0 	  >0
//��ǰ -           - 

//��ƽ��  >0    >0  	<0 	  <0
//��  <0    >0		>0		<0
//��  <0    <0  	>0 	  >0
//            -          -

//ǰ-ת�� �ٶȺϳ�
#define SPEED_T1        3500    //�����ٶ�1
#define SPEED_T2        1500    //�����ٶ�2

//����ԭ��ת��
#define SPEED_T3        1500        //ԭ��ת���ٶ�1
#define SPEED_T4        2000        //ԭ��ת���ٶ�2

#define ADD_SPEED  1000

 extern ScanMove HeadMove;
int16_t ADDTarget(int16_t last_target,int16_t real_target);


//static void OneKeyDeal_WSADQR(uint16_t key);
//static void TowKeyDeal_WSAD(uint16_t key);
//static void  FastMode_Shift(int16_t *ctr);

//ң�صĸ���ͨ�����ݴ���
void DBUS_Deal(void)
{
    //ң������ͨ�����ݴ���
	buff_zero[0]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;   // �����ƶ�
	buff_zero[1]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[2]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[3]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	
	buff_one[0]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;    // ǰ���ƶ�  �ٷ��汾���ٷ��������Լ��ĳ�ǰ����Ʒ���
	buff_one[1]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	buff_one[2]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	buff_one[3]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	
	buff_two[0]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;    //˳ʱ����ת
	buff_two[1]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	buff_two[2]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	buff_two[3]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	
	
	buff_thr[0]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;   //��ʱ����ת
	buff_thr[1]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
	buff_thr[2]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
	buff_thr[3]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
 	
	
		//�ٶȸ�ֵ
		ChassisPidLF.target=buff_zero[0]+buff_one[0]+buff_two[0]+buff_thr[0];
		ChassisPidRF.target=buff_zero[1]+buff_one[1]+buff_two[1]+buff_thr[1];
		ChassisPidRB.target=buff_zero[2]+buff_one[2]+buff_two[2]+buff_thr[2];
		ChassisPidLB.target=buff_zero[3]+buff_one[3]+buff_two[3]+buff_thr[3];
		
		
		if((ChassisPidLF.target>THRESHOLD)&&(ChassisPidRF.target>THRESHOLD)&&(ChassisPidRB.target>THRESHOLD)&&(ChassisPidLB.target>THRESHOLD))			//˳ת�ٶȿ���
		{
			ChassisPidLF.target=TURNSPEED;
			ChassisPidRF.target=TURNSPEED;
			ChassisPidRB.target=TURNSPEED;
			ChassisPidLB.target=TURNSPEED;
		}
		
		if((ChassisPidLF.target<-THRESHOLD)&&(ChassisPidRF.target<-THRESHOLD)&&(ChassisPidRB.target<-THRESHOLD)&&(ChassisPidLB.target<-THRESHOLD))			//˳ת�ٶȿ���
		{
	    ChassisPidLF.target=-TURNSPEED;
			ChassisPidRF.target=-TURNSPEED;
			ChassisPidRB.target=-TURNSPEED;
			ChassisPidLB.target=-TURNSPEED;
		}	  
}


//����������
//static void OneKeyDeal_WSADQR(uint16_t KeyPress)
//{
//	
//	if(KeyPress&KEY_PRESSED_W)    //when "W" is pushed ,then go along. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
//		{
//			moto_ctr[0]=SPEED_NORMAL;
//			moto_ctr[1]=-SPEED_NORMAL;
//			moto_ctr[2]=-SPEED_NORMAL;
//			moto_ctr[3]=SPEED_NORMAL;	
//		}
//		
//	  if(KeyPress&KEY_PRESSED_S)    //when "S"  is pushed ,then go back. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
//		{
//			moto_ctr[0]=-SPEED_NORMAL;
//			moto_ctr[1]=SPEED_NORMAL;
//			moto_ctr[2]=SPEED_NORMAL;
//			moto_ctr[3]=-SPEED_NORMAL;			
//		}
//		
//	   if(KeyPress&KEY_PRESSED_A)    //when "A" is pushed ,then rotate anticlockwise
//		{
//	    moto_ctr[0]=-SPEED_X;
//			moto_ctr[1]=-SPEED_X;
//			moto_ctr[2]=SPEED_X;
//			moto_ctr[3]=SPEED_X;		
//		}
//		
//		if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then rotate clockwise
//		{
//			
//			moto_ctr[0]=SPEED_X;
//			moto_ctr[1]=SPEED_X;
//			moto_ctr[2]=-SPEED_X;
//			moto_ctr[3]=-SPEED_X;
//	
//		}
//		
//		if(KeyPress&KEY_PRESSED_Q)    //when "Q"  is pushed ,then go left
//		{
//			

//			moto_ctr[0]=-SPEED_T3;
//			moto_ctr[1]=-SPEED_T3;
//			moto_ctr[2]=-SPEED_T3;
//			moto_ctr[3]=-SPEED_T3;
//		}
//		
//		if(KeyPress&KEY_PRESSED_R)    //when "R" is pushed ,then go right
//		{
//			moto_ctr[0]=SPEED_T3;
//			moto_ctr[1]=SPEED_T3;
//			moto_ctr[2]=SPEED_T3;
//			moto_ctr[3]=SPEED_T3;
//		}
//		
//		if((RC_Ctl.key.v==0)&&(RC_Ctl.mouse.x==0))    //when mouse is not moved and no key is pushed ,then stop
//		{
//			moto_ctr[0]=0;
//			moto_ctr[1]=0;
//			moto_ctr[2]=0;
//			moto_ctr[3]=0;
//		}
//		
//}


//�ٶȺϳ�
//ǰ��    :  +  -  -  + 
//����    :  -  -  -  -
//��ƽ    :  -  -  +  +
//ǰ����  :  0        0


//ǰ  >0    <0  	<0  	>0
//��ƽ��    <0    <0  	>0 	  >0
//��ǰ -           - 

//��ƽ��  >0    >0  	<0 	  <0
//��  <0    >0		>0		<0
//��  <0    <0  	>0 	  >0

//������������ͬʱ����  б���˶�
//static void TowKeyDeal_WSAD_T(uint16_t KeyPress)
//{
//	  //WA  ��ǰ��
//		if((KeyPress&0X05)==0X05)    //when "W" and "A" are pushed ,then turn left
//		{
//			moto_ctr[0]=SPEED_T2;
//			moto_ctr[1]=-SPEED_T1;
//			moto_ctr[2]=-SPEED_T1;
//			moto_ctr[3]=SPEED_T2;
//		}
//		
//		//WD   ��ǰ��
//		if((KeyPress&0X09)==0X09)    //when "W" and "D" are pushed ,then turn right
//		{
//			moto_ctr[0]=SPEED_T1;   
//			moto_ctr[1]=-SPEED_T2;
//			moto_ctr[2]=-SPEED_T2;
//			moto_ctr[3]=SPEED_T1;
//		}
//		
//		//WSAD   ���  2,4��С
//		if((KeyPress&0X06)==0X06)    //when "S" and "A" are pushed ,then turn left
//		{
//			moto_ctr[0]=-SPEED_T2;
//			moto_ctr[1]=SPEED_T1;
//			moto_ctr[2]=SPEED_T1;
//			moto_ctr[3]=-SPEED_T2;
//		}
//		
//		//SD    �Һ�  2,4��С
//		if((KeyPress&0X0A)==0X0A)    //when S" and "D" are pushed ,then turn left
//		{
//			moto_ctr[0]=-SPEED_T1;
//			moto_ctr[1]=SPEED_T2;
//			moto_ctr[2]=SPEED_T2;
//			moto_ctr[3]=-SPEED_T1;
//		}

//    if(KeyPress&0x0010)  //���shift����
//		 {
//			 if(RC_Ctl.key.v&0X0003)        //WS�а��µİ���
//		      FastMode_Shift(moto_ctr);
//       else if(RC_Ctl.key.v&0X000C)    //AD����
//			    {
//							if(KeyPress&KEY_PRESSED_A)    //when "A"  is pushed ,then go left
//								{
//									moto_ctr[0]=-SPEED_T3;
//									moto_ctr[1]=-SPEED_T3;
//									moto_ctr[2]=-SPEED_T3;
//									moto_ctr[3]=-SPEED_T3;
//								}
//								
//						 		if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then go right
//								{
//									moto_ctr[0]=SPEED_T3;
//									moto_ctr[1]=SPEED_T3;
//									moto_ctr[2]=SPEED_T3;
//									moto_ctr[3]=SPEED_T3;
//								}							
//					}
//       else
// 			 {
//				  	moto_ctr[0]=0;
//						moto_ctr[1]=0;
//						moto_ctr[2]=0;
//				    moto_ctr[3]=0;
//						
//			 }				 
//		 }
//    		
//}




//ң�����ݴ���
void MotorTargetChange(void)	
{
  
//   static int8_t Left_press=0;
//	 static char send=0;
  										
     DBUS_Deal();  //ң�ظ���ͨ������
	  if(RC_Ctl.mouse.press_l==1)  //��Ѫ����
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
    else if(RC_Ctl.mouse.press_r==1)  
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
		
		
	  if(RC_Ctl.rc.s2==2)  //���������207������ s2Ϊ2ʱ�������
    {
			if(RealSpeedPLUCK==0)
				reversal_flag++;
			if(reversal_flag>10)
			{
				PluckPID.target=0;
				delay_ms(10);
				PluckPID.target=-500;
        reversal_flag=0;				
				delay_ms(40);
				PluckPID.target=0;
				delay_ms(10);
			}
       PluckPID.target=500;       
    }
    else
		{
				PluckPID.target=0;
		}

      if((RC_Ctl.key.v&KEY_PRESSED_Z)&&(RC_Ctl.key.v&KEY_PRESSED_CTRL))	   //̧������
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			else if((RC_Ctl.key.v&KEY_PRESSED_Z)==KEY_PRESSED_Z)
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

      if((RC_Ctl.key.v&KEY_PRESSED_X)&&(RC_Ctl.key.v&KEY_PRESSED_CTRL))	   //�н�����
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			else if((RC_Ctl.key.v&KEY_PRESSED_X)==KEY_PRESSED_X)
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			

      if((RC_Ctl.key.v&KEY_PRESSED_C)&&(RC_Ctl.key.v&KEY_PRESSED_CTRL))	   //��ת���205
				  RotatePID.target=1000;
			else if((RC_Ctl.key.v&KEY_PRESSED_C)==KEY_PRESSED_C)
				  RotatePID.target=-1000;
			else 
					RotatePID.target=0;

      if((RC_Ctl.key.v&KEY_PRESSED_V)&&(RC_Ctl.key.v&KEY_PRESSED_CTRL))	   //�������
				  TIM2->CCR1=450;		
			else if((RC_Ctl.key.v&KEY_PRESSED_V)==KEY_PRESSED_V)				
				  TIM2->CCR1=1000;
			
			
			
//		OneKeyDeal_WSADQR(RC_Ctl.key.v);  //���������� WSAD QR
//		TowKeyDeal_WSAD_T(RC_Ctl.key.v);
		 
    
   
    
      if((RC_Ctl.key.v&KEY_PRESSED_V)==KEY_PRESSED_V)
      {
        
          //if(HeadMove.YawErr>1000)
           //  HeadMove.YawErr=1000;
                   
          //else if(HeadMove.YawErr<-1000)
           //  HeadMove.YawErr=-1000;
          
          
          
      //  #define YAWANGLEMAX  5300          //��ƫ���ұ�
      //  #define YAWANGLEMIN   3000          //��ƫ�����
      //  #define YAWANGLEMID   4250 

      //  #define PITCHANGLEMAX   4100       //���������Ƕ�
      //  #define PITCHANGLEMIN   3500          //��������С�Ƕ�
      //  #define PITCHANGLEMID   3800
           
           // if(send==0)
//            {
//              YawOPID.target=YAWANGLEMID-(int16_t)((float)HeadMove.YawErr);
//              //send=1;
//            }
//           YawOPID.target=(YawOPID.target>YAWANGLEMAX)?YAWANGLEMAX:YawOPID.target;
//           YawOPID.target=(YawOPID.target<YAWANGLEMIN)?YAWANGLEMIN:YawOPID.target; 
                                                                                       
          
//           PitchOPID.target=PITCHANGLEMID+HeadMove.PitchErr/5;
//           PitchOPID.target=(PitchOPID.target>PITCHANGLEMAX)?PITCHANGLEMAX:PitchOPID.target;
//           PitchOPID.target=(PitchOPID.target<PITCHANGLEMIN)?PITCHANGLEMIN:PitchOPID.target; 
//          
                          
				ChassisPidLF.target=ADDTarget(ChassisPidLF.target,moto_ctr[0]);
				ChassisPidRF.target=ADDTarget(ChassisPidRF.target,moto_ctr[1]);
				ChassisPidRB.target=ADDTarget(ChassisPidRB.target,moto_ctr[2]);
				ChassisPidLB.target=ADDTarget(ChassisPidLB.target,moto_ctr[3]);
	
	   }
}

int16_t ADDTargetHead(int16_t last_target,int16_t real_target)
{
	  if(abs(last_target-real_target)>150)   //�����һ�ε�Ŀ��ֵ��˴�Ŀ��ֵ�෴
		{
			  if(last_target<real_target)   //�����һ�ε�Ŀ��ֵΪ��
				{
					  return last_target+150;    //Ŀ��ת�ټ�100
				}
				else
				{
					  return last_target-150;
				}
		}
		else 
			return real_target;
}


//��ֹ��ΪĿ��ֵͻ�������ʧ��
//Ŀ���ٶ�ֵ�𽥱仯
//����ֵ����һ�ε��ٶ�ֵ Ŀ���ٶ�ֵ
//����  ��������ת��ֵ
int16_t ADDTarget(int16_t last_target,int16_t real_target)
{
	  if(abs(last_target-real_target)>2000)   //�����һ�ε�Ŀ��ֵ��˴�Ŀ��ֵ�෴
		{
			  if(last_target<real_target)   //�����һ�ε�Ŀ��ֵΪ��
				{
					  return last_target+150;    //Ŀ��ת�ټ�100
				}
				else
				{
					  return last_target-150;
				}
		}
		else 
			return real_target;
}

////����״̬
//static void FastMode_Shift(int16_t ctr[])
//{
//	  char i=0;
//	for(i=0;i<4;i++)
//	{
//		 if(ctr[i]<-1500)
//			 ctr[i]-=ADD_SPEED;
//		 else if(ctr[i]>1500)
//			 ctr[i]+=ADD_SPEED;
//	}
//}
	
	
/****************************************************************
 * ��ʱ����
 *
 *
*****************************************************************/

void delay_ms(unsigned int t)
{
	int i;
  int a=0;
	for( i=0;i<t;i++)
	{
		for(a=0;a<42000;a++)
		{};
		//int a=42000; //at 168MHz 42000 is ok
		//while(a--);
	}
}



