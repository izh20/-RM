// 配置头文件
//
//

       //目标速度
//左 6500 右边 2000  中间：4300 //步兵： 3500-4500  下->上


  //左 5300 右边 3000  中间：4350
  #define YAWANGLEMAX   4800          //航偏角左边
  #define YAWANGLEMIN   2600          //航偏角右边
  #define YAWANGLEMID   3700               //4250

  #define PITCHANGLEMAX   4800       //俯仰角最大角度
  #define PITCHANGLEMIN   3600          //俯仰角最小角度
  #define PITCHANGLEMID   4200
  
  #define CHASSISMAX     4000           //单个底盘电机最大允许电流
  




#define INTEGARALSNUM    5        //PID积分次数

#define MOTORID   0x200  //底盘电机ID
#define HEADID    0x1FF   //云台电机ID  
 
 

#define CHEAKSTART     0X00  //校正开始
#define CHEAKING       0X40  //校正运行中
#define CHEAKFINISH    0X80  //校正结束


