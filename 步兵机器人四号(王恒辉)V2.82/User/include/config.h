// 配置头文件
//
//

       //目标速度
//左 6500 右边 2000  中间：4300 //步兵： 3500-4500  下->上
  
#define MAXELECTRICITY 2000      //底盘最大允许电流


#ifdef ROBOT1

  //左 5300 右边 3000  中间：4350
  #define YAWANGLEMAX   5900          //航偏角左边
  #define YAWANGLEMIN   2100          //航偏角右边
  
  #define YAWANGLEMID   3800      //4250

  #define PITCHANGLEMAX   6700          //俯仰角最大角度
  #define PITCHANGLEMIN   7450          //俯仰角最小角度
  #define PITCHANGLEMID   7200
  
  #define CHASSISMAX     2600           //单个底盘电机最大允许电流
  

#elif  ROBOT2
//左 5300 右边 3000  中间：4350
  #define YAWANGLEMAX   3000          //航偏角右边
  #define YAWANGLEMIN   5300          //航偏角左边
  #define YAWANGLEMID   4400 

  #define PITCHANGLEMAX   4800       //俯仰角最大角度
  #define PITCHANGLEMIN   3900       //俯仰角最小角度
  #define PITCHANGLEMID   4450

  #define CHASSISMAX     3500
    
#elif  ROBOT3

   //左 5300 右边 3000  中间：4350
  #define YAWANGLEMAX   4700          //航偏角左边
  #define YAWANGLEMIN   200          //航偏角右边
  #define YAWANGLEMID   2400 


  #define PITCHANGLEMAX   500       //俯仰角最大角度
  #define PITCHANGLEMIN   1140       //俯仰角最小角度
  #define PITCHANGLEMID   650
  #define CHASSISMAX     2500

#endif



#define INTEGARALSNUM    5        //PID积分次数

#define MOTORID   0x200  //底盘电机ID
#define HEADID    0x1FF   //云台电机ID  
 
 //uint8_t sdm;

#define CHEAKSTART     0X00  //校正开始
#define CHEAKING       0X40  //校正运行中
#define CHEAKFINISH    0X80  //校正结束


