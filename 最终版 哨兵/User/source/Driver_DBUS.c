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
double mouse_206,mouse_205;
uint8_t mouse_flag;
uint8_t null_flag=0;
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
extern int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 
extern PidTypeDef PitchOPID;  //������λ���⻷
extern PidTypeDef YawOPID;
extern CatTypedef CatStruct;  

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
const int THRESHOLD=2000;  //ת����ֵ
const int TURNSPEED=2000;   //ת�伫���ٶ�

//���̿���
#define SPEED_S       0
#define SPEED_NORMAL  5000     //����ǰ���ٶ�

//��ϰ����������б����  3000:1500 22.5����ǰ   3000:0    45��        3000��-3000��ת  
//#define SPEED_Y        3500     //ǰ�����ٶ�

#define SPEED_X        3000      //�����ٶ�


//ǰ  >0    <0  	<0  	>0
//��ƽ��    <0    <0  	>0 	  >0
//��ǰ -           - 

//��ƽ��  >0    >0  	<0 	  <0
//��  <0    >0		>0		<0
//��  <0    <0  	>0 	  >0
//            -          -

//ǰ-ת�� �ٶȺϳ�
#define SPEED_T1        4500    //�����ٶ�1
#define SPEED_T2        2000    //�����ٶ�2


//����ԭ��ת��
#define SPEED_T3        2000        //ԭ��ת���ٶ�1
#define SPEED_T4        3000        //ԭ��ת���ٶ�2

#define ADD_SPEED  1000

 extern ScanMove HeaddadMove;
 //extern AngleTypedef  ChassisAngle;
// extern int16_t ChassisAngleNow;
int16_t ADDTarget(int16_t last_target,int16_t real_target);
 extern ScanMove HeadMove;

static void OneKeyDeal_WSADQR(uint16_t key);
static void TowKeyDeal_WSAD(uint16_t key);
static void  FastMode_Shift(int16_t *ctr);



//ң�صĸ���ͨ�����ݴ���
void DBUS_Deal(void)
{
//    //ң������ͨ�����ݴ���
	buff_zero[0]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;   // �����ƶ�
	buff_zero[1]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[2]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[3]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
////	
//	buff_one[0]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;    // ǰ���ƶ�  �ٷ��汾���ٷ��������Լ��ĳ�ǰ����Ʒ���
//	buff_one[1]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
//	buff_one[2]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
//	buff_one[3]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
//	
//	buff_two[0]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;    //˳ʱ����ת
//	buff_two[1]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
//	buff_two[2]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
//	buff_two[3]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
//	
//	
//	buff_thr[0]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;   //��ʱ����ת
//	buff_thr[1]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
//	buff_thr[2]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
//	buff_thr[3]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
 	
  
	   
//  moto_ctr[0]=buff_zero[0]+buff_one[0]+buff_two[0]+buff_thr[0];
//  moto_ctr[1]=buff_zero[1]+buff_one[1]+buff_two[1]+buff_thr[1];
//  moto_ctr[2]=buff_zero[2]+buff_one[2]+buff_two[2]+buff_thr[2];
//  moto_ctr[3]=buff_zero[3]+buff_one[3]+buff_two[3]+buff_thr[3];

  moto_ctr[0]=buff_zero[0];
  moto_ctr[1]=buff_zero[1];
  moto_ctr[2]=buff_zero[2];
  moto_ctr[3]=buff_zero[3];
  
  //if((buff_two[0]==0)&&(buff_thr[0]==0))
	// CorrectDirect(moto_ctr);   //��̬У��
 
   ChassisPidLF.target=moto_ctr[0];
   ChassisPidRF.target=moto_ctr[1];
   ChassisPidRB.target=moto_ctr[2];
   ChassisPidLB.target=moto_ctr[3];
	
		
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
static void OneKeyDeal_WSADQR(uint16_t KeyPress)
{
	  // static char set=0;
  
	  if(KeyPress&KEY_PRESSED_W)    //when "W" is pushed ,then go along. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
		{
			moto_ctr[0]=SPEED_NORMAL;
			moto_ctr[1]=-SPEED_NORMAL;
			moto_ctr[2]=-SPEED_NORMAL;
			moto_ctr[3]=SPEED_NORMAL;	
		}
		
	  if(KeyPress&KEY_PRESSED_S)    //when "S"  is pushed ,then go back. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
		{
			moto_ctr[0]=-SPEED_NORMAL;
			moto_ctr[1]=SPEED_NORMAL;
			moto_ctr[2]=SPEED_NORMAL;
			moto_ctr[3]=-SPEED_NORMAL;			
		}
		
	   if(KeyPress&KEY_PRESSED_A)    //when "A" is pushed ,then rotate anticlockwise
		{
	    moto_ctr[0]=-SPEED_X;
			moto_ctr[1]=-SPEED_X;
			moto_ctr[2]=SPEED_X;
			moto_ctr[3]=SPEED_X;		
		}
		
		if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then rotate clockwise
		{
			moto_ctr[0]=SPEED_X;
			moto_ctr[1]=SPEED_X;
			moto_ctr[2]=-SPEED_X;
			moto_ctr[3]=-SPEED_X;
		}
		
		if(KeyPress&KEY_PRESSED_Q)    //when "Q"  is pushed ,then go left
		{
			moto_ctr[0]=-SPEED_T3;
			moto_ctr[1]=-SPEED_T3;
			moto_ctr[2]=-SPEED_T3;
			moto_ctr[3]=-SPEED_T3;
		}
		
		if(KeyPress&KEY_PRESSED_R)    //when "R" is pushed ,then go right
		{
			moto_ctr[0]=SPEED_T3;
			moto_ctr[1]=SPEED_T3;
			moto_ctr[2]=SPEED_T3;
			moto_ctr[3]=SPEED_T3;
		}
    
    if(KeyPress&KEY_PRESSED_F)     //F:ת��Ϊ��ɫʶ��
		{
		  while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	    USART6->DR =0XFF;
     
      while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	    USART6->DR =0X00;
      
      while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	    USART6->DR =0X00;
      
      while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	    USART6->DR =0XFE;
      
		}
		
//    if(RC_Ctl.key.v&KEY_PRESSED_C)      //���C�������� //�����Զ����ģʽ
//    { 
//        if(set==0)
//        {
//          set=1;
//          //CatStruct.GyHeadPID.target= (int16_t)refSysData.GameRobotPos_t.yaw;  
//          CatStruct.GyTime=450;
//					
//          CatStruct.dir=1;
//          CatStruct.mode=1;          //����ģʽ
//      
//          ChassisPidLF.target=2000;
//     //     ChassisPidRF.target=2000;
//     //     ChassisPidLB.target=2000;
//          ChassisPidRB.target=-2000;
//          
////        ChassisAngle.AngleMid=ChassisAngleNow;  //��ȡ��ǰ�ĽǶ�ֵ       
////        ChassisAngle.AngleMove=0;               //��ǰ��Ҫƫ�Ƶ���ԽǶ�ֵ   
//        }
//    }
//    else 
//    {
//       set=0;
//    //  CatStruct.mode=0;           //�˳��ģʽ
//    
//    }
     
		if((RC_Ctl.key.v==0)&&(RC_Ctl.mouse.x==0))    //when mouse is not moved and no key is pushed ,then stop
		{
			moto_ctr[0]=0;
			moto_ctr[1]=0;
			moto_ctr[2]=0;
			moto_ctr[3]=0;
		}
		
}


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
static void TowKeyDeal_WSAD_T(uint16_t KeyPress)
{
	  //WA  ��ǰ��
		if((KeyPress&0X05)==0X05)    //when "W" and "A" are pushed ,then turn left
		{
			moto_ctr[0]=SPEED_T2;
			moto_ctr[1]=-SPEED_T1;
			moto_ctr[2]=-SPEED_T1;
			moto_ctr[3]=SPEED_T2;
		}
		
		//WD   ��ǰ��
		if((KeyPress&0X09)==0X09)    //when "W" and "D" are pushed ,then turn right
		{
			moto_ctr[0]=SPEED_T1;   
			moto_ctr[1]=-SPEED_T2;
			moto_ctr[2]=-SPEED_T2;
			moto_ctr[3]=SPEED_T1;
		}
		
		//WSAD   ���  2,4��С
		if((KeyPress&0X06)==0X06)    //when "S" and "A" are pushed ,then turn left
		{
			moto_ctr[0]=-SPEED_T2;
			moto_ctr[1]=SPEED_T1;
			moto_ctr[2]=SPEED_T1;
			moto_ctr[3]=-SPEED_T2;
		}
		
		//SD    �Һ�  2,4��С
		if((KeyPress&0X0A)==0X0A)    //when S" and "D" are pushed ,then turn left
		{
			moto_ctr[0]=-SPEED_T1;
			moto_ctr[1]=SPEED_T2;
			moto_ctr[2]=SPEED_T2;
			moto_ctr[3]=-SPEED_T1;
		}

    if(KeyPress&0x0010)  //���shift����
		 {
			 if(RC_Ctl.key.v&0X0003)        //WS�а��µİ���
		      FastMode_Shift(moto_ctr);
       else if(RC_Ctl.key.v&0X000C)    //AD����
			    {
							if(KeyPress&KEY_PRESSED_A)    //when "A"  is pushed ,then go left
								{
									moto_ctr[0]=-SPEED_T3;
									moto_ctr[1]=-SPEED_T3;
									moto_ctr[2]=-SPEED_T3;
									moto_ctr[3]=-SPEED_T3;
                  
                  //CatStruct.GyHeadPID.target=1300;
								 //  CatStruct.GyHeadPID.target=800; //(int16_t)refSysData.GameRobotPos_t.yaw;  
                 // CatStruct.mode=2;
                }
								
						 	 else if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then go right
								{
									moto_ctr[0]=SPEED_T3;
									moto_ctr[1]=SPEED_T3;
									moto_ctr[2]=SPEED_T3;
									moto_ctr[3]=SPEED_T3;
               //    CatStruct.GyHeadPID.target=800;// (int16_t)refSysData.GameRobotPos_t.yaw;  
               //   CatStruct.mode=2;
                  //CatStruct.GyHeadPID.target=1300;
                  
								}							
					}
       else
 			 {
				  	moto_ctr[0]=0;
						moto_ctr[1]=0;
						moto_ctr[2]=0;
				    moto_ctr[3]=0;
						
			 }				 
		 }
    		
}



 extern uint16_t RealAngleYAW,RealAnglePITCH;
 extern PidTypeDef HeadTargets;
 extern uint8_t color_flag;

//ң�����ݴ���
void MotorTargetChange(void)	
{
   static int MoveSet=0;
	 static int move=0;
static	 uint8_t color_flag=1;
static	 uint8_t set_red=0;
static	 uint8_t set_blue=0;
   static int8_t Left_press=0;

	
	 //static char send=0;
  
  if(RC_Ctl.rc.s1==0)  //ң����ģʽ
	{											
      DBUS_Deal();  //ң�ظ���ͨ������
	    if(RC_Ctl.rc.s2==2)  // ��s2 23���ع����� ����
    {
         if(Left_press==0)
         {
            Left_press=1;
           if(ShootNumber.Number<3)  //����������
               ShootNumber.Number++;
         }          
    }
    else
       Left_press=0;
		     
      YawOPID.target=YAWANGLEMID;
      PitchOPID.target=PITCHANGLEMID;
 
  }

  else if(RC_Ctl.rc.s1==1)   //�����Զ�ʶ��ģʽ
	{
		
		if(RC_Ctl.rc.ch1>1400)
		{		
			 MoveSet=1;
		}
	 else if(RC_Ctl.rc.ch1<500)
	 {
		 MoveSet=0;
	 }
	 
	 
	 
	 		if(RC_Ctl.rc.ch3>1400)
		{		
			  color_flag=1;
		}
	 else if(RC_Ctl.rc.ch3<500)
	 { 
		   color_flag=0;
	 }
	 
	 
	 
	 
	  if(color_flag==1)     //ת��Ϊ��ɫʶ��
		{
      if(set_red==0x00)
      {
				//printf("red \n");
        set_red=1;
        while((USART6->SR&0X40)==0);      //��ʼ
        USART6->DR =0XFF;
       
        while((USART6->SR&0X40)==0); 
        USART6->DR =0X00;
        
        while((USART6->SR&0X40)==0);   
        USART6->DR =0X00;
        
        while((USART6->SR&0X40)==0);   
        USART6->DR =0XFE;
				 while((USART6->SR&0X40)==0);  
       }
		}
    else
      set_red=0;
		
    
	if(color_flag==0)     
		{
      if(set_blue==0x00)  //color flag Ϊ0ʱ��set blueΪ1 set redΪ0 
      {
				//	printf("blue \n");
        set_blue=0X01;
        while((USART6->SR&0X40)==0);      //��ʼ
        USART6->DR =0XFF;
       
        while((USART6->SR&0X40)==0); 
        USART6->DR =0X00;
        
        while((USART6->SR&0X40)==0);   
        USART6->DR =0X01;
        
        while((USART6->SR&0X40)==0);   
        USART6->DR =0XFE;
			  while((USART6->SR&0X40)==0);   
       }
	 }
	else
      set_blue=0;
	
	

		if(MoveSet==1)
		{
			if(move==0)
			{
			move=1;
		  CatStruct.GyTime=1000;	
      CatStruct.dir=1;
			CatStruct.mode=0x01;   //�����ģʽ
		//���Ƶ��
	//	  ChassisPidLF.target=2000;
      ChassisPidRF.target=3000;
      ChassisPidLB.target=-3000;
  //   ChassisPidRB.target=-2000;
		 }
			
	 }
		else
		{
				move=0;
				CatStruct.mode=0;	 //�˳��ƶ�ģ  �����Զ�ģʽ
				ChassisPidLB.target=0;
				ChassisPidRF.target=0;
		}
	
	
if(MoveSet  == 0)
	{	
		
       if(RC_Ctl.rc.s2==2)  // ��s2 23���ع����� ����
    {
         if(Left_press==0)
         {
            Left_press=1;
           if(ShootNumber.Number<3)  //����������
               ShootNumber.Number++;
         }          
    }
		else
		{
        Left_press=0;
		}
	
         HeadMove.robots=0x01;  //Ŀ���ǲ���
        if(HeadMove.mode!=0)     //���������Ч        
        {
          if(abs(HeadMove.YawErr)>70||abs(HeadMove.PitchErr)>40)
					 {
                 if(HeadMove.flags>=5)
                {
                   YawOPID.target=RealAngleYAW+(int16_t)((double)HeadMove.YawErr*0.21); 
                   PitchOPID.target=RealAnglePITCH-(int16_t)((double)HeadMove.PitchErr*0.2); 
                   HeadMove.flags=0;
                }
            }
            else
            {   
               HeadMove.flags++;
              if((abs(HeadMove.YawErr)<70)&&(abs(HeadMove.PitchErr)<40))
              {
                HeadMove.flags=5;
                    if(ShootNumber.AllowShoot==0)
                    {
                      ShootNumber.Number=1;  
                    }             
              }  
            }	
					
           YawOPID.target=(YawOPID.target>YAWANGLEMAX)?YAWANGLEMAX:YawOPID.target;
           YawOPID.target=(YawOPID.target<YAWANGLEMIN)?YAWANGLEMIN:YawOPID.target; 
         }  
       else
				{
							if(null_flag>2)
							{
								      YawOPID.target=YAWANGLEMID;
											PitchOPID.target=PITCHANGLEMID;
											null_flag=0;
							}
							
				}
   
			} 
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
	    if(real_target-last_target>100)   //�����һ�ε�Ŀ��ֵΪ��
				{
					  return last_target+100;    //Ŀ��ת�ټ�100
				}
				else if(real_target-last_target<-100)
				{
					  return last_target-100;
				}
        
			return real_target;
}

//����״̬
static void FastMode_Shift(int16_t ctr[])
{  
	  char i=0;
	for(i=0;i<4;i++)
	{
		 if(ctr[i]<-1500)
			 ctr[i]-=ADD_SPEED;
		 else if(ctr[i]>1500)
			 ctr[i]+=ADD_SPEED;
	}
}
	
	
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



