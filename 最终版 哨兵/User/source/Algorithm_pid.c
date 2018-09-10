
//天涯浪子
//2018/1/30
//
#include "Algorithm_pid.h"

/**
  * @brief  通用位置式PID算法
  * @param  pid:输入的PID结构体参数    real:真实值
  * @retval 计算输出值
  */

//PID函数		
int16_t PIDCalc(PidTypeDef *pid,int16_t real)
{
	float derror, error;
	int16_t ElectricOutput;
	double Electric;
	
	error =pid->target - real;

	pid->error=	pid->target - real;
	pid->derror=pid->error-pid->lasterror; 
  pid->lasterror=pid->error;	
   	
	 //积分分离
	if(abs(error)<pid->Istart)
	{ 
	  pid->sum += error;
	}
	
	  //积分限幅
	pid->sum = (pid->sum  >  pid->SumMax)?  pid->SumMax : pid->sum;
	pid->sum = (pid->sum  <  -pid->SumMax)?  -pid->SumMax : pid->sum;

	pid->Pout= pid->Kp * error;
	pid->Iout= pid->Ki * pid->sum;
	pid->Dout= pid->Kd * derror;

	Electric=pid->Pout+pid->Iout+pid->Dout;
  
  Electric=(Electric>20000)?20000:Electric;      //防止开机时强制转换数据溢出 ！！！
  Electric=(Electric<-20000)?-20000:Electric;
	ElectricOutput =(int16_t)(Electric);
	
		//判断是否超过限流
	ElectricOutput=(ElectricOutput>pid->OutMax)? pid->OutMax:ElectricOutput;  
	ElectricOutput=(ElectricOutput<(-pid->OutMax))? (-pid->OutMax):(ElectricOutput);			

	return ElectricOutput;
}











