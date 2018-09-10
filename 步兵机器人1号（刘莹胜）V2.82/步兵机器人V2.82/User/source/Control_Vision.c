#include "Control_Vision.h"
#include "common.h"
#include "Driver_oled.h"

#define SCANSTART 0XFF   //数据帧开始
#define SCANENDS  0XFE    //数据帧结束
ScanMove HeadMove={0}; 

uint8_t temp[30];


void ScanDataInit(void)
{
   HeadMove.YawErr=0; 
   HeadMove.PitchErr=0; 
  
   HeadMove.YawMissHero=-250;      
   HeadMove.PitchMissHero=250;
  
  
    HeadMove.YawMissInfance=-180;       //向左
    HeadMove.PitchMissInfance=230;     //向下
  
   HeadMove.robots=0x00;  //默认为英雄模式
   HeadMove.flags=0X02; 
}

//提取视觉模块发来的数据
void  ScanDataVision(unsigned char *str)
{
    char i=0;
    char success=0;
    int16_t YawTemp=0,PitchTemp=0;
  
    for(i=0;i<8;i++)
    {
        if((str[i]==SCANSTART)&&(str[i+7]==SCANENDS))
        {
          success=1;
          break;
        }
    }
    
    if(success==0x01)
    {
 
      YawTemp=(str[i+2]<<8)+str[i+1];
      PitchTemp=(str[i+4]<<8)+str[i+3];
			if((YawTemp<=3000)&&(YawTemp>=-3000)&&(PitchTemp<3000)&&(PitchTemp>=-3000))
			{
					HeadMove.mode=str[i+5];
					HeadMove.num=str[i+6];	 
         
          if(HeadMove.robots==0x00)
          {
             HeadMove.YawErr=YawTemp+HeadMove.YawMissHero;
					   HeadMove.PitchErr=PitchTemp+HeadMove.PitchMissHero;
          }
          else
          {
             HeadMove.YawErr=YawTemp+HeadMove.YawMissInfance;
					   HeadMove.PitchErr=PitchTemp+HeadMove.PitchMissInfance;
          }
          
            HeadMove.flags++;              //目标值更新
         if(HeadMove.flags>=100)
            HeadMove.flags=21;
      
      }
      
//      sprintf(temp,"Y:%5d P:%5d",HeadMove.YawErr,HeadMove.PitchErr);
//      OLED_ShowString(0,2,temp,16);
//      
//      sprintf(temp,"M:%5d N:%5d",HeadMove.mode,HeadMove.num);
//      OLED_ShowString(0,4,temp,16);
    }    
}
