#include "Driver_shoot.h"


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//	if(htim == &htim6)
//	{
////		CAN_Send_Msg(&hcan1, trans_data, Driver_CAN1_ID, 8);
//	}

//}
	void TIM_SetTIM12Compare(uint16_t compare1,uint16_t compare2)
{
	TIM12->CCR1=compare1;
	TIM12->CCR2=compare2;
}
	void TIM_SetTIM2Compare(uint16_t bo_compare1,uint16_t bo_compare2)
{
	TIM2->CCR1=bo_compare1;
	TIM2->CCR1=bo_compare2;

}


char flag=0;
char mouseflag_l=1;
char mouseflag_r=1;
char flag_bostart=0;

//摩擦轮初始化
void mo_init()
{
	uint16_t pulse=1000;					//(pulse)/10000;
	TIM_SetTIM12Compare(pulse,pulse);
	delay_ms(500);
	char  i;
  for(i=0;i<10;i++)
	{
		pulse+=30;
		TIM_SetTIM12Compare(pulse,pulse);
		delay_ms(200);		
	}
}

//拨弹电机转动
void mocha_shoot()
{
		if(RC_Ctl.rc.s1==0)	
			{
					if(RC_Ctl.rc.s2==3&&flag==0)
						{
							TIM_SetTIM2Compare(5000,10000);
							delay_ms(500);
							TIM_SetTIM2Compare(5000,5000);
							TIM_SetTIM12Compare(800,800);
							flag=1;
						}
					if(RC_Ctl.rc.s2==3&&flag==1)
						{
							TIM_SetTIM2Compare(5000,5000);
							TIM_SetTIM12Compare(800,800);
							flag=1;
						}
						if(RC_Ctl.rc.s2==2&&flag==1)
						{
							mo_init();
							TIM_SetTIM2Compare(5000,0);
							flag=0;
						}
						if(RC_Ctl.rc.s2==1&&flag==1)
						{
							mo_init();
							TIM_SetTIM2Compare(5000,5000);
							flag=0;
						}
			}
		if(RC_Ctl.rc.s1==1)	
		{
			if(RC_Ctl.mouse.press_l==1&&mouseflag_l==1)
				{
					mo_init();
					flag_bostart=1;
					mouseflag_l=0;
					delay_ms(1000);
				}
				if(RC_Ctl.mouse.press_l==1&&mouseflag_l==0)
				{
					
					TIM_SetTIM12Compare(800,800);
					TIM_SetTIM2Compare(5000,5000);
					mouseflag_l=1;
					flag_bostart=0;
					delay_ms(1000);
				}
				if(RC_Ctl.mouse.press_r==1&&mouseflag_r==1&&flag_bostart==1)
				{
					TIM_SetTIM2Compare(5000,0);
					mouseflag_r=0;
					delay_ms(1000);
				}
					if(RC_Ctl.mouse.press_r==1&&mouseflag_r==0)
				{
					TIM_SetTIM2Compare(5000,5000);
					mouseflag_r=1;
					delay_ms(1000);
				}
				
		}
	
}
