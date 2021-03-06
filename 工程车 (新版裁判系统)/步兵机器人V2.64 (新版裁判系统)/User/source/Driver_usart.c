/**
  *@file Driver_uart.c
  *@date 2018-1-14
  *@author 天涯浪子
  *@brief 
  */

#include "Driver_usart.h"
#include "Control_Vision.h"

uint8_t uart1_rx_buff[50];
uint8_t uart2_rx_buff[15]; 
uint8_t uart3_rx_buff[50];
uint8_t uart6_rx_buff[50];

uint8_t   UART6_BUFF[250];

__IO uint16_t  Uart6_Lenth=0;

extern uint16_t aim_angle_206;
extern uint16_t aim_angle_205;
extern uint8_t start_flag;
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
extern int16_t ANGLE;
//it will be auto callback when usart receive msg completely
//遥控接收18个数据结束后中断函数
//

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  if(huart == &huart6)
  {  
    __HAL_UART_CLEAR_PEFLAG(&huart6);
    
	//	HAL_UART_Transmit(huart, uart6_rx_buff, 1, 200);
     // if(uart6_rx_buff[0]==0XFF)    //寻找帧头
     //      Uart6_Lenth=0X4000;
    
    UART6_BUFF[Uart6_Lenth]=uart6_rx_buff[0];    //将数据缓存到设备
    Uart6_Lenth++;
    
    if(Uart6_Lenth>=250)
       Uart6_Lenth=0;
    
    //ScanDataVision(uart6_rx_buff);
		HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);    
	}
	
	
  if(huart == &huart1)
  {
		//遥控器数据帧处理函数
			Yaokong_translate();
			start_flag = 1;
		//直接返回值
//		HAL_UART_Transmit(huart, uart1_rx_buff, 18, 100);
		//用usart6返回值
//		HAL_UART_Transmit(&huart6, uart1_rx_buff, 15, 100); 
	
		__HAL_UART_CLEAR_PEFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18);
	}
	
	//电子罗盘口
	//if(huart == &huart2)
  //{
  //   ANGLE=(uart2_rx_buff[2]-'0')*1000+(uart2_rx_buff[3]-'0')*100+(uart2_rx_buff[4]-'0')*10+(uart2_rx_buff[6]-'0');   
	//   __HAL_UART_CLEAR_PEFLAG(&huart2);
	//}
	
}




//void UART_SendByte (unsigned char dat)
//{ 
//	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART6->DR = (uint8_t) dat; 
//}

//void UART_SendByte (unsigned char dat)
//{ 
//	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART3->DR = (uint8_t) dat; 
//}
  void UART_SendByte (unsigned char dat)
{ 
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (uint8_t) dat; 
}

void uart_putbuff(uint8_t *buff, unsigned int len)
{
   while(len--)
   {
       UART_SendByte (*buff);
       buff++;
   }
}

extern int16_t ElectricPitch206,ElectricYaw205;
extern uint16_t RealAnglePITCH,RealAngleYAW;
extern int16_t RealAnglePLUCK,RealSpeedPLUCK;
extern IMUDataTypedef imu_data;

void VcanSendHead(void)
{
	 uint8_t sendnum[16]={0};

	 sendnum[0]=(uint8_t)((RealAnglePITCH)&0XFF);
	 sendnum[1]=(uint8_t)((RealAnglePITCH>>8)&0XFF);
	 
	 sendnum[2]=(uint8_t)((ElectricPitch206)&0XFF);
	 sendnum[3]=(uint8_t)((ElectricPitch206>>8)&0XFF);
	 
//	sendnum[4]=(uint8_t)((RealAngleYAW)&0XFF);
//	sendnum[5]=(uint8_t)((RealAngleYAW>>8)&0XFF);
	 
//	sendnum[6]=(uint8_t)((ElectricYaw205)&0XFF);
//	sendnum[7]=(uint8_t)((ElectricYaw205>>8)&0XFF);	 
	 
	 sendnum[4]=(uint8_t)((RealAngleYAW)&0XFF);
	 sendnum[5]=(uint8_t)((RealAngleYAW>>8)&0XFF);
	 
	 sendnum[6]=(uint8_t)((ElectricYaw205)&0XFF);
	 sendnum[7]=(uint8_t)((ElectricYaw205>>8)&0XFF);
	 
	 //发送目标转速值
	 sendnum[8]=(uint8_t)((imu_data.az)&0XFF);
	 sendnum[9]=(uint8_t)((imu_data.az>>8)&0XFF);
	 
	 sendnum[10]=(uint8_t)((imu_data.gx)&0XFF);
	 sendnum[11]=(uint8_t)((imu_data.gx>>8)&0XFF);
	 
	 sendnum[12]=(uint8_t)((imu_data.gy)&0XFF);
	 sendnum[13]=(uint8_t)((imu_data.gy>>8)&0XFF);
  
	 sendnum[14]=(uint8_t)((imu_data.gz)&0XFF);
	 sendnum[15]=(uint8_t)((imu_data.gz>>8)&0XFF);

   // imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
   
	  vcan_sendware(sendnum,16);
 }

//发送速度值与电流值,传输到山外上位机显示
void VcanSendSpeedElectric(void)
{
	 uint8_t sendnum[16]={0};
//	 static char i=0;

	 //发送转速值	 
	 sendnum[0]=(uint8_t)((RealAnglePITCH)&0XFF);
	 sendnum[1]=(uint8_t)((RealAnglePITCH>>8)&0XFF);
	 
	 sendnum[2]=(uint8_t)((ElectricPitch206)&0XFF);
	 sendnum[3]=(uint8_t)((ElectricPitch206>>8)&0XFF);
	 
	 sendnum[4]=(uint8_t)((imu_data.gx)&0XFF);
	 sendnum[5]=(uint8_t)((imu_data.gx>>8)&0XFF);
	 
	 sendnum[6]=(uint8_t)((RealSpeedLB)&0XFF);
	 sendnum[7]=(uint8_t)((RealSpeedLB>>8)&0XFF);
	 
	 vcan_sendware(sendnum,16);
	
}

extern refDataStruct  refSysData;
//发送速度值与电流值,传输到山外上位机显示
void VcanStatues(void)
{
	 uint8_t sendnum[16]={0};
   int16_t Temp[4]={0};
  
//   Temp[0]=(int16_t)(refSysData.refData1Struct.remianLifeValue);
//   Temp[1]=(int16_t)(refSysData.refData1Struct.realChassisOutA*refSysData.refData1Struct.realChassisOutV);
//   Temp[2]= (int16_t)(refSysData.refData3Struct.realBulletShootSpeed*10);
//   Temp[3]= (int16_t)(refSysData.refData3Struct.realBulletShootFreq);
   
    //血量
	 sendnum[0]=(uint8_t)(Temp[0]&0XFF);
	 sendnum[1]=(uint8_t)((Temp[0]>>8)&0XFF);
	
	 //功率	 
	 sendnum[2]=(uint8_t)(Temp[1]&0XFF);
	 sendnum[3]=(uint8_t)((Temp[1]>>8)&0XFF);
	 
   //子弹射速
	 sendnum[4]=(uint8_t)(Temp[2]&0XFF);
	 sendnum[5]=(uint8_t)((Temp[2]>>8)&0XFF);
	 
   //子弹射频
	 sendnum[6]=(uint8_t)((Temp[3])&0XFF);
	 sendnum[7]=(uint8_t)((Temp[3]>>8)&0XFF);
   
   
   sendnum[8]=(uint8_t)((RealSpeedLF)&0XFF);
	 sendnum[9]=(uint8_t)((RealSpeedLF>>8)&0XFF);
	 
	 sendnum[10]=(uint8_t)((RealSpeedRF)&0XFF);
	 sendnum[11]=(uint8_t)((RealSpeedRF>>8)&0XFF);
	 
	 sendnum[12]=(uint8_t)((RealSpeedRB)&0XFF);
	 sendnum[13]=(uint8_t)((RealSpeedRB>>8)&0XFF);
	 
	 sendnum[14]=(uint8_t)((RealSpeedLB)&0XFF);
	 sendnum[15]=(uint8_t)((RealSpeedLB>>8)&0XFF);
   
	 vcan_sendware(sendnum,16);
	
}


//void display(void)
//{
//	if(dis_flag == 1)
//	{
//		printf("剩余血量:%d\n",refSysData.refData1Struct.remianLifeValue);
//		printf("底盘电压:%0.2f\n",refSysData.refData1Struct.realChassisOutV);
//		printf("底盘电流:%0.2f\n",refSysData.refData1Struct.realChassisOutA);
//		printf("底盘功率:%0.2f\n",refSysData.refData1Struct.realChassisOutA*refSysData.refData1Struct.realChassisOutV);
//		printf("\n");
//		printf("受伤装甲:%d\n",refSysData.refData2Struct.weakld);
//		printf("受伤方式:%d\n",refSysData.refData2Struct.way);
//		printf("血量变化值:%d\n",refSysData.refData2Struct.value);
//		printf("\n");
//		printf("步兵子弹速度:%0.2f\n",refSysData.refData3Struct.realBulletShootSpeed);
//		printf("步兵子弹频率:%0.2f\n",refSysData.refData3Struct.realBulletShootFreq);
//		printf("英雄子弹速度:%0.2f\n",refSysData.refData3Struct.realGolfShootSpeed);
//		printf("英雄子弹频率:%0.2f\n",refSysData.refData3Struct.realGolfShootFreq);
//		printf("\n");
//		dis_flag = 0;
//	}
//}


/**************************************************************
*发送数据到上位机虚拟示波器 山外上位机调试助手
* wareaddr: 需要发送的数组地址,默认uint8,每一个数据代表一个通道,最大为8
* waresize: 发送数据的大小
*
***************************************************************/
void vcan_sendware(uint8_t *wareaddr, unsigned int waresize)
{
    #define  CMD_WARE     3   //使用虚拟示波器模式
    unsigned char cmdf[2] = {CMD_WARE, ~CMD_WARE};  
    unsigned char cmdr[2] = {~CMD_WARE, CMD_WARE};    

    uart_putbuff(cmdf, sizeof(cmdf));    //发送帧头
	  uart_putbuff(wareaddr, waresize);   //发送数据
     
    uart_putbuff(cmdr, sizeof(cmdr));    
}
