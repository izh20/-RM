#include "Control_Vision.h"
#include "common.h"
#include "Driver_oled.h"

#define SCANSTART 0XFF   //数据帧开始
#define SCANENDS  0XFE    //数据帧结束
ScanMove HeadMove={0,0,0,0}; 

uint8_t temp[30];

//提取视觉模块发来的数据
void  ScanDataVision(char *str)
{
    char i=0;
    char success=0;
  
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
      HeadMove.YawErr=str[i+2]*256+str[i+1];
      HeadMove.PitchErr=str[i+4]*256+str[i+3];
      HeadMove.mode=str[i+5];
      HeadMove.num=str[i+6];
      
      
      sprintf(temp,"Y:%5d P:%5d",HeadMove.YawErr,HeadMove.PitchErr);
      OLED_ShowString(0,2,temp,16);
      
      sprintf(temp,"M:%5d N:%5d",HeadMove.mode,HeadMove.num);
      OLED_ShowString(0,4,temp,16);
    }    
}
