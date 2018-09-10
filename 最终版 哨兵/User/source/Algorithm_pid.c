
//��������
//2018/1/30
//
#include "Algorithm_pid.h"

/**
  * @brief  ͨ��λ��ʽPID�㷨
  * @param  pid:�����PID�ṹ�����    real:��ʵֵ
  * @retval �������ֵ
  */

//PID����		
int16_t PIDCalc(PidTypeDef *pid,int16_t real)
{
	float derror, error;
	int16_t ElectricOutput;
	double Electric;
	
	error =pid->target - real;

	pid->error=	pid->target - real;
	pid->derror=pid->error-pid->lasterror; 
  pid->lasterror=pid->error;	
   	
	 //���ַ���
	if(abs(error)<pid->Istart)
	{ 
	  pid->sum += error;
	}
	
	  //�����޷�
	pid->sum = (pid->sum  >  pid->SumMax)?  pid->SumMax : pid->sum;
	pid->sum = (pid->sum  <  -pid->SumMax)?  -pid->SumMax : pid->sum;

	pid->Pout= pid->Kp * error;
	pid->Iout= pid->Ki * pid->sum;
	pid->Dout= pid->Kd * derror;

	Electric=pid->Pout+pid->Iout+pid->Dout;
  
  Electric=(Electric>20000)?20000:Electric;      //��ֹ����ʱǿ��ת��������� ������
  Electric=(Electric<-20000)?-20000:Electric;
	ElectricOutput =(int16_t)(Electric);
	
		//�ж��Ƿ񳬹�����
	ElectricOutput=(ElectricOutput>pid->OutMax)? pid->OutMax:ElectricOutput;  
	ElectricOutput=(ElectricOutput<(-pid->OutMax))? (-pid->OutMax):(ElectricOutput);			

	return ElectricOutput;
}











