
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
    float index=1;
	
	error =pid->target - real;
	
//�����ϵ��
	if(abs(error)<300)
	{
		index=0.7+(abs(error)/300)*0.3;
	}
	
	pid->lasterror=pid->error;	
	pid->error=	pid->target - real;
	pid->derror=pid->error-pid->lasterror; 
   	
	 //���ַ���
	if(abs(error)<pid->Istart)
	{ 
	  pid->sum += error;
	}
	
	  //�����޷�
	pid->sum = (pid->sum>pid->SumMax)? pid->SumMax:pid->sum;
	pid->sum = (pid->sum<-pid->SumMax)? -pid->SumMax:pid->sum;

	pid->Pout= pid->Kp *index* error;
	pid->Iout= pid->Ki * pid->sum;
	pid->Dout= pid->Kd * derror;

	Electric=pid->Pout+pid->Iout+pid->Dout;       
	ElectricOutput =(int16_t)(Electric);
	
//	  ElectricOutput=(ElectricOutput-pid->PIDout>pid->AddMax)? (pid->PIDout+pid->AddMax):ElectricOutput;
//	  ElectricOutput=(ElectricOutput-pid->PIDout<-pid->AddMax)? (pid->PIDout-pid->AddMax):ElectricOutput;
//pid->PIDout=ElectricOutput;
	
		//�ж��Ƿ񳬹�����
	ElectricOutput=(ElectricOutput>pid->OutMax)? pid->OutMax:ElectricOutput;  
	ElectricOutput=(ElectricOutput<(-pid->OutMax))? (-pid->OutMax):(ElectricOutput);			

	return ElectricOutput;
}


////����PID����
//int16_t PIDCalc( PidTypeDef *pid, int16_t real) 
//{
//	float derror, error;
//	int16_t ElectricOutput;
//	double Electric;
//  float index=1;
//	
//	
//	error =pid->target - real;
////�����ϵ��
//	if(abs(error)<300)
//	{
//		index=0.7+(abs(error)/300)*0.3;
//	}
//	
//	pid->lasterror=pid->error;	
//  pid->error=	pid->target - real;
//  pid->derror=pid->error-pid->lasterror; 
//   	
//	 //���ַ���
//	if(abs(error)<ERRORMAX)
//	 { 
//	  pid->sum += error;
//	 }
//	  pid->sum = (pid->sum>20000)? 20000:pid->sum;
//	  
//	  pid->Pout= pid->Kp *index* error;
//	  pid->Iout= pid->Ki * pid->sum;
//	  pid->Dout= pid->Kd * derror;
//     
//    Electric=pid->Pout+pid->Iout+pid->Dout;     
//    
//	//������ĵ����޷�
//    if((pid->target>0))
//		{
//			   if(Electric<0)
//					   Electric=0;
//		}
//		
//		if((pid->target<0))
//		{
//			   if(Electric>0)
//					  Electric=0;
//		}
//	      
//		ElectricOutput =(int16_t)(Electric);

//		//�ж��Ƿ񳬹�����
//		ElectricOutput=(ElectricOutput>MAXELECTRICITY)? MAXELECTRICITY:ElectricOutput;  
//		ElectricOutput=(ElectricOutput<(-MAXELECTRICITY))? (-MAXELECTRICITY):(ElectricOutput);			
//		
//		return ElectricOutput;
//		

////		error = pid->target - real; 
////	 
////		derror = pid->ierror[0] - 2*pid->ierror[1] + pid->ierror[2]; 
////		
////		ElectricOutput = pid->Kp * (pid->ierror[0] - pid->ierror[1]) + pid->Ki * pid->ierror[0] + pid->Kd * derror;

////		pid->ierror[2] = pid->ierror[1];
////		pid->ierror[1] = pid->ierror[0];
////		pid->ierror[0] = error;	
//		
////		ElectricOutput=(ElectricOutput>MAXELECTRICITY)? MAXELECTRICITY:ElectricOutput;  
////		ElectricOutput=(ElectricOutput<(-MAXELECTRICITY))? (-MAXELECTRICITY):(ElectricOutput);	
////		
////		return ElectricOutput;

//}









