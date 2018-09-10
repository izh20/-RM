/**
  *@file 
  *@date 2018-3-17
  *@author 天涯浪子
  *@brief 摩擦轮转动、拨弹电机计数
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


//摩擦轮初始化
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
ShootNumTypedef ShootNumber={0,0,0};    //需要射击的次数累计
 
void PluckPIDInit(void)
{
	 
     //ShootNumbe.
   ShootNumber.AllowShoot=0x00;     //默认允许射击   0:允许射击  1：禁止射击
   ShootNumber.LastFinish=0x00;     //0：上一次射击完毕 1：上一次射击正在进行
   ShootNumber.Number=0x00;         //射击的次数
  
	  PluckPID.Kp=2;   //2000  95转 *15 
	  PluckPID.Ki=0;
	  PluckPID.Kd=0;
	  //PluckPID.AddMax=30;
	  PluckPID.Istart=100;    //积分分离，积分开始的值
	  PluckPID.SumMax=3000;   //积分限幅
	  PluckPID.OutMax=2500;   //最大输出电流
	  PluckPID.target=1200;   //
}



int16_t ElectricShoot=0;

// 0x01:   0:启动        1:停止 
// 0x02:   0:初始值复位  1：不变
// 0x04:   0:圈数改变    1：圈数不变
// 0x08:   0:圈数加1     1：圈数减1
// 0x10:   0:是第一次    1：不是第一次开始

#ifdef __MAX__
typedef struct 
{
	
	int8_t   StartFlag;     //标志位
	int8_t   dir;
	int8_t   Stop;            //判断是否卡弹
	int16_t  StartThreshold;  //最开始的角度值
	int16_t  Threshold;   //比较的阈值，每次开始的角度值	
	int16_t  DeadElectric;  //死区电压  
	float    Circle;      //1次拨子弹的圈数
	float    Circles;     //7次的圈数
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
void CorrectCircle(void);   //校正
void CircleStep(void);      //对1步进行控制

void StartShoot()
{
//   PluckMotor.StartFlag=PluckMotor.StartFlag&0XF0;
   PluckMotor.StartFlag=PluckMotor.StartFlag|0X01;
  
   PluckMotor.Circle=0;
   PluckMotor.dir=0;
}


extern int16_t RealSpeedPLUCK;

//射击控制 
void ShootControl(uint16_t NowAngle)
{
     
    CycleNumber(NowAngle);  //获取当前的角度值  
	   //SevenCircleNumber(NowAngle);    //获取7格累计圈速
	  if(PluckMotor.Stop==1)      //如果发现堵转
		{
		//	 HeadTxData[4] = (uint8_t)(-1000>>8)&0xFF);  //   
	     ElectricShoot=-600;     //反转电流
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

//对每圈进行计数，保证一格
void CircleStep(void)
{
	int32_t Target;
//	static char i=0;
  
	if((PluckMotor.StartFlag&0x01)==0x01)    //电机是否启动
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
//					  if(PluckMotor.Circles>192.00)   //需要反转
//					    PluckMotor.dir=0X01;
//					  else
//						  PluckMotor.dir=0X00;
//					 
//					 LED_Red_Toggle();
//				}
//				else     
//				{
//			      i++;
//	          PluckMotor.StartFlag=PluckMotor.StartFlag|0x01;   //停止电机						
//				}		
       
        PluckMotor.StartFlag=PluckMotor.StartFlag&0xFE;   //停止电机
				ElectricShoot=0;
        ShootNumber.LastFinish=0X00;
        
			 }
				 
	 }
	 else 
		  ElectricShoot=0;
	 
}



// 返回当前圈数值
//传入参数： NowAngle：当前电机的角度  
//返回值:    当前的圈数值
// 0xff    
// 0x01:   电机状态      0:启动        1:停止 
// 0x02:                0:初始值复位   1：不变
// 0x04:                0:圈数改变     1：圈数不变
// 0x08:                

//累积圈数
//  0x20:   0:初始值复位  1：不变
//  0x40:   0:圈数改变    1：圈数不变
//  0x80:   0:已经校正    1：需要校正

//0110 1011

float CycleNumber(int16_t NowAngle)
{
	//如果是新的开启
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
         MoreAngle=NowAngle-PluckMotor.Threshold;   //超过的度数
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
        PluckMotor.StartFlag&=0xFB;  //标志位清零	 			
       }		
    }
    PluckMotor.Circle=(int)PluckMotor.Circle+(float)MoreAngle/8192;

	return  PluckMotor.Circle;	
}






/**********************************************|七格校正,单独计数程序,程序中暂时未加|*******************************************/

// 7格校正程序 在此未用 
//每7格进行一次校正
void CorrectCircle(void)
{
	int32_t Target;
	if((PluckMotor.StartFlag&0x01)==0x00)    //电机是否启动
	{
	  Target=(192-PluckMotor.Circles)*100;
	  PluckPID.target=(Target>3000)?3000:Target;
	  ElectricShoot=PIDCalc(&PluckPID,RealSpeedPLUCK);
		
	  if(PluckMotor.dir==0)  //需要正转
			 ElectricShoot=((ElectricShoot<250))?250:ElectricShoot;
	  else 
		  ElectricShoot=(ElectricShoot>-250)?-250:ElectricShoot;
	
      if((192-PluckMotor.Circles<0.2)&&(192-PluckMotor.Circles>-0.2)) 
	  { 
			PluckMotorInit();
			ElectricShoot=0;
//		     PluckMotor.StartFlag=0X51;     //复位一切
//		     PluckMotor.dir=0x00;
//         PluckMotor.Circle=0;
//         PluckMotor.Circles=0;
		  
	  }
   }
   else
      ElectricShoot=0;
     	  
}



//七格单独计圈  未使用
//传入参数： NowAngle：当前电机的角度  
//返回值:    当前的圈数值
// 0xff    
// 0x01:   电机状态      0:启动        1:停止 
// 0x02:                0:初始值复位  1：不变
// 0x04:                0:圈数改变    1：圈数不变
// 0x08:   运行方向      0：正向   1：反向

//累积圈数
//  0x20:   0:初始值复位    1：不变
//  0x40:   0:圈数改变    1：圈数不变

void SevenCircleNumber(int16_t NowAngle)
{
	//如果是新的开启
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
	     MoreAngle=NowAngle-PluckMotor.StartThreshold;   //超过的度数
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
			PluckMotor.StartFlag&=0xBF;  //标志位清零	 			
		 }		
	}
	PluckMotor.Circles=(int)PluckMotor.Circles+(float)MoreAngle/8192;
	
}

