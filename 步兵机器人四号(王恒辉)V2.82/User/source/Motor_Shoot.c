/**
  *@file 
  *@date 2018-3-17
  *@author ��������
  *@brief Ħ����ת���������������
  */
	
#include "Motor_Shoot.h"

extern RC_Ctl_t RC_Ctl;




//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//	if(htim == &htim6)
//	{
////		CAN_Send_Msg(&hcan1, trans_data, Driver_CAN1_ID, 8);
//	}

//}
	void TIM_SetTIM5Compare(uint16_t compare1,uint16_t compare2)
{
	TIM5->CCR1=compare1;
	TIM5->CCR2=compare2;
}

	void TIM_SetTIM2Compare(uint16_t bo_compare1,uint16_t bo_compare2)
{
	TIM2->CCR1=bo_compare1;
	TIM2->CCR1=bo_compare2;

}




char mouseflag_l=1;
char mouseflag_r=1;
char flag_bostart=0;


//Ħ���ֳ�ʼ��
void RM2312_Init(void)
{
	uint16_t pulse=1000;					
	TIM_SetTIM5Compare(pulse,pulse);
	delay_ms(400);
//	char  i;
//  for(i=0;i<10;i++)
//	{
//		pulse+=30;
//		TIM_SetTIM12Compare(pulse,pulse);
//		delay_ms(200);		
//	} 
}




PidTypeDef PluckPID;
ShootNumTypedef ShootNumber={0,0,0};    //��Ҫ����Ĵ����ۼ�
 
void PluckPIDInit(void)
{
	 
     //ShootNumbe.
   ShootNumber.AllowShoot=0x00;     //Ĭ���������   0:�������  1����ֹ���
   ShootNumber.LastFinish=0x00;     //0����һ�������� 1����һ��������ڽ���
   ShootNumber.Number=0x00;         //����Ĵ���
  
	  PluckPID.Kp=2;   //2000  95ת *15 
	  PluckPID.Ki=0;
	  PluckPID.Kd=0;
	  //PluckPID.AddMax=30;
	  PluckPID.Istart=100;    //���ַ��룬���ֿ�ʼ��ֵ
	  PluckPID.SumMax=3000;   //�����޷�
	  PluckPID.OutMax=2500;   //����������
	  PluckPID.target=1200;   //
}



int16_t ElectricShoot=0;

// 0x01:   0:����        1:ֹͣ 
// 0x02:   0:��ʼֵ��λ  1������
// 0x04:   0:Ȧ���ı�    1��Ȧ������
// 0x08:   0:Ȧ����1     1��Ȧ����1
// 0x10:   0:�ǵ�һ��    1�����ǵ�һ�ο�ʼ

#ifdef __MAX__
typedef struct 
{
	
	int8_t   StartFlag;     //��־λ
	int8_t   dir;
	int8_t   Stop;            //�ж��Ƿ񿨵�
	int16_t  StartThreshold;  //�ʼ�ĽǶ�ֵ
	int16_t  Threshold;   //�Ƚϵ���ֵ��ÿ�ο�ʼ�ĽǶ�ֵ	
	int16_t  DeadElectric;  //������ѹ  
	float    Circle;      //1�β��ӵ���Ȧ��
	float    Circles;     //7�ε�Ȧ��
}Shoot;
#endif


Shoot PluckMotor;//={0X5D,0,0,0,0,300,0.0,0.0};

void PluckMotorInit(void)
{
	PluckMotor.dir=0;
	PluckMotor.StartFlag=0X51;  //0101 1101
	PluckMotor.StartThreshold=0;
	PluckMotor.Threshold=0;
	PluckMotor.Num=0;
	PluckMotor.Circle=0.0;
	PluckMotor.Circles=0.0;
	PluckMotor.DeadElectric=400;
	  
}
float CycleNumber(int16_t NowAngle);
void SevenCircleNumber(int16_t NowAngle);
void CorrectCircle(void);   //У��
void CircleStep(void);      //��1�����п���

void StartShoot()
{
//   PluckMotor.StartFlag=PluckMotor.StartFlag&0XF0;
   PluckMotor.StartFlag=PluckMotor.StartFlag|0X01;
  
   PluckMotor.Circle=0;
   PluckMotor.dir=0;
}


extern int16_t RealSpeedPLUCK;

//������� 
void ShootControl(uint16_t NowAngle)
{
     
    CycleNumber(NowAngle);  //��ȡ��ǰ�ĽǶ�ֵ  
	   //SevenCircleNumber(NowAngle);    //��ȡ7���ۼ�Ȧ��
	  if(PluckMotor.Stop==1)      //������ֶ�ת
		{
		//	 HeadTxData[4] = (uint8_t)(-1000>>8)&0xFF);  //   
	     ElectricShoot=-600;     //��ת����
			 PluckMotorInit();  
		}
		else
		{
		 	 CircleStep();
		}
    
		HeadTxData[4] = (uint8_t)((ElectricShoot>>8)&0xFF);  // 
	  HeadTxData[5] = (uint8_t)(ElectricShoot&0xFF); 
   
}

//     if((PluckMotor.StartFlag&0x80)==0x80)
//	 {
//		 //   CorrectCircle();
//	 }		 
//	 else
//	 {
		//   CircleStep();
	// }
 
//

//��ÿȦ���м�������֤һ��
void CircleStep(void)
{
	int32_t Target;
//	static char i=0;
  
	if((PluckMotor.StartFlag&0x01)==0x01)    //����Ƿ�����
	 {   
     
		     Target=(int32_t)(15-PluckMotor.Circle)*100; 
		     PluckPID.target=(Target>2500)?2500:Target;
		     ElectricShoot=PIDCalc(&PluckPID,RealSpeedPLUCK);
		     ElectricShoot=((ElectricShoot<PluckMotor.DeadElectric))?PluckMotor.DeadElectric:ElectricShoot;
		 
			 if(5-PluckMotor.Circle<1) 
		 { 
				 
       
			//	if(PluckMotor.Circles-192<10)
//					if(i>=13) 
//			 {
//				     PluckMotor.StartFlag|=0x80;  
//				     
//					  if(PluckMotor.Circles>192.00)   //��Ҫ��ת
//					    PluckMotor.dir=0X01;
//					  else
//						  PluckMotor.dir=0X00;
//					 
//					 LED_Red_Toggle();
//				}
//				else     
//				{
//			      i++;
//	          PluckMotor.StartFlag=PluckMotor.StartFlag|0x01;   //ֹͣ���						
//				}		
       
        PluckMotor.StartFlag=PluckMotor.StartFlag&0xFE;   //ֹͣ���
				ElectricShoot=0;
        ShootNumber.LastFinish=0X00;
        
			 }
				 
	 }
	 else 
		  ElectricShoot=0;
	 
}



// ���ص�ǰȦ��ֵ
//��������� NowAngle����ǰ����ĽǶ�  
//����ֵ:    ��ǰ��Ȧ��ֵ
// 0xff    
// 0x01:   ���״̬      0:����        1:ֹͣ 
// 0x02:                0:��ʼֵ��λ   1������
// 0x04:                0:Ȧ���ı�     1��Ȧ������
// 0x08:                

//�ۻ�Ȧ��
//  0x20:   0:��ʼֵ��λ  1������
//  0x40:   0:Ȧ���ı�    1��Ȧ������
//  0x80:   0:�Ѿ�У��    1����ҪУ��

//0110 1011

float CycleNumber(int16_t NowAngle)
{
	//������µĿ���
  int MoreAngle=0;
	

    if((PluckMotor.StartFlag&0X02)==0x00)
    {
      PluckMotor.StartFlag|=0X02;
      PluckMotor.Threshold=NowAngle;	
      PluckMotor.Circle-=1;
    }
    else 
    {
       NowAngle=(NowAngle<PluckMotor.Threshold)?(NowAngle+8192):NowAngle;
         MoreAngle=NowAngle-PluckMotor.Threshold;   //�����Ķ���
        if((MoreAngle>0)&&(MoreAngle<1000))
        {
           if((PluckMotor.StartFlag&0X04)==0x00)
          {   
             PluckMotor.StartFlag|=0X04;
              PluckMotor.Circle+=1;
                  
          }  		
       }
          else if((MoreAngle>3000)&&(MoreAngle<8190))
       {
        PluckMotor.StartFlag&=0xFB;  //��־λ����	 			
       }		
    }
    PluckMotor.Circle=(int)PluckMotor.Circle+(float)MoreAngle/8192;

	return  PluckMotor.Circle;	
}






/**********************************************|�߸�У��,������������,��������ʱδ��|*******************************************/

// 7��У������ �ڴ�δ�� 
//ÿ7�����һ��У��
void CorrectCircle(void)
{
	int32_t Target;
	if((PluckMotor.StartFlag&0x01)==0x00)    //����Ƿ�����
	{
	  Target=(192-PluckMotor.Circles)*100;
	  PluckPID.target=(Target>3000)?3000:Target;
	  ElectricShoot=PIDCalc(&PluckPID,RealSpeedPLUCK);
		
	  if(PluckMotor.dir==0)  //��Ҫ��ת
			 ElectricShoot=((ElectricShoot<250))?250:ElectricShoot;
	  else 
		  ElectricShoot=(ElectricShoot>-250)?-250:ElectricShoot;
	
      if((192-PluckMotor.Circles<0.2)&&(192-PluckMotor.Circles>-0.2)) 
	  { 
			PluckMotorInit();
			ElectricShoot=0;
//		     PluckMotor.StartFlag=0X51;     //��λһ��
//		     PluckMotor.dir=0x00;
//         PluckMotor.Circle=0;
//         PluckMotor.Circles=0;
		  
	  }
   }
   else
      ElectricShoot=0;
     	  
}



//�߸񵥶���Ȧ  δʹ��
//��������� NowAngle����ǰ����ĽǶ�  
//����ֵ:    ��ǰ��Ȧ��ֵ
// 0xff    
// 0x01:   ���״̬      0:����        1:ֹͣ 
// 0x02:                0:��ʼֵ��λ  1������
// 0x04:                0:Ȧ���ı�    1��Ȧ������
// 0x08:   ���з���      0������   1������

//�ۻ�Ȧ��
//  0x20:   0:��ʼֵ��λ    1������
//  0x40:   0:Ȧ���ı�    1��Ȧ������

void SevenCircleNumber(int16_t NowAngle)
{
	//������µĿ���
  int MoreAngle=0;
  if((PluckMotor.StartFlag&0X20)==0x00)
	{
		PluckMotor.StartFlag|=0X20;
		PluckMotor.StartThreshold=NowAngle;	
		PluckMotor.Circles-=1;
	}
  else 
	{
		   NowAngle=(NowAngle<PluckMotor.StartThreshold)?(NowAngle+8192):NowAngle;
	     MoreAngle=NowAngle-PluckMotor.StartThreshold;   //�����Ķ���
		  if((MoreAngle>0)&&(MoreAngle<1000))
		  {
				 if((PluckMotor.StartFlag&0X40)==0x00)
				{   
					PluckMotor.StartFlag|=0X40;
					if(PluckMotor.dir==0X00)                
					{
						PluckMotor.Circles+=1;
					}
					else
					{
						PluckMotor.Circles-=1;
					}						
				}  
				
		 }
	      else if((MoreAngle>4000)&&(MoreAngle<8190))
		 {
			PluckMotor.StartFlag&=0xBF;  //��־λ����	 			
		 }		
	}
	PluckMotor.Circles=(int)PluckMotor.Circles+(float)MoreAngle/8192;
	
}

