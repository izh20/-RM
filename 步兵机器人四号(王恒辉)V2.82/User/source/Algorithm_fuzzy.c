/******************************************************************************
*******************************************************************************
*                           Freescale-������ѧ--ӣ                            *
*                               ģ���㷨�ӳ���                                *
*                                                                             *
*                                                                             *
*                                                                             *
*                                                                             *
*                                               Miz.Wong                      *
*                                                  @                          *
*                                             Innovation.Lab                  *
*******************************************************************************/
#include "Algorithm_fuzzy.h"

/****************************
Fuction: Caculate Fuzzy Value 
Par.   : FuzzyStruct* Fuzzy_S
Return : Fuzzy Value
****************************/
double FuzzyCtrl(FuzzyStruct* Fuzzy_S)
{	
	double  eFuzzy[2]  ={0.0, 0.0};
	double  ecFuzzy[2] ={0.0, 0.0};
	double  U1Fuzzy[7] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	int  num=0,pe=0,pec=0;
	double kp=0.0;
	int rank;
	int i=0;
	rank=Fuzzy_S->Rank;
	switch(rank)
	{
	case 7:
		{
                        /*-----���������������-----*/
			if(Fuzzy_S->fe < Fuzzy_S->eRule[0])		        // |x_x_x_x_x_x_x_
			{
				eFuzzy[0] =1.0; 
				pe= 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[1])	        // _x|x_x_x_x_x_x_
			{       
				eFuzzy[0] = (Fuzzy_S->eRule[1]-Fuzzy_S->fe)/(Fuzzy_S->eRule[1]-Fuzzy_S->eRule[0]);
				pe = 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[2])	        // _x_x|x_x_x_x_x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[2] -Fuzzy_S->fe)/(Fuzzy_S->eRule[2]-Fuzzy_S->eRule[1]);
				pe =1;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[3])	        // _x_x_x|x_x_x_x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[3] -Fuzzy_S->fe)/(Fuzzy_S->eRule[3]-Fuzzy_S->eRule[2]);
				pe =2;
			}
			else if(Fuzzy_S->fe<Fuzzy_S->eRule[4])		        // _x_x_x_x|x_x_x_
			{   
				eFuzzy[0] = (Fuzzy_S->eRule[4]-Fuzzy_S->fe)/(Fuzzy_S->eRule[4]-Fuzzy_S->eRule[3]);
				pe=3;
			}
			else if(Fuzzy_S->fe<Fuzzy_S->eRule[5])		        // _x_x_x_x_x|x_x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[5]-Fuzzy_S->fe)/(Fuzzy_S->eRule[5]-Fuzzy_S->eRule[4]);
				pe=4;
			}
			else if(Fuzzy_S->fe<Fuzzy_S->eRule[6])		        // _x_x_x_x_x_x|x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[6]-Fuzzy_S->fe)/(Fuzzy_S->eRule[6]-Fuzzy_S->eRule[5]);
				pe=5;
			}		
			else						        // _x_x_x_x_x_x_x|
			{
				eFuzzy[0] =1.0;
				pe=6;
			}
			eFuzzy[1] = 1.0 - eFuzzy[0];

			/*-----���仯������������-----*/
			if(Fuzzy_S->fec < Fuzzy_S->ecRule[0])
			{
				ecFuzzy[0] =1.0;
				pec = 0;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[1])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[1] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[1]-Fuzzy_S->ecRule[0]);
				pec = 0 ;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[2])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[2] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[2]-Fuzzy_S->ecRule[1]);
				pec = 1;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[3])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[3] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[3]-Fuzzy_S->ecRule[2]);
				pec = 2 ;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[4])
			{   ecFuzzy[0] = (Fuzzy_S->ecRule[4] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[4]-Fuzzy_S->ecRule[3]);
				pec=3;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[5])		
			{
				eFuzzy[0] = (Fuzzy_S->ecRule[5]-Fuzzy_S->fec)/(Fuzzy_S->ecRule[5]-Fuzzy_S->ecRule[4]);
				pec=4;
			}
			else if(Fuzzy_S->fec<Fuzzy_S->ecRule[6])		
			{
				eFuzzy[0] = (Fuzzy_S->ecRule[6]-Fuzzy_S->fec)/(Fuzzy_S->ecRule[6]-Fuzzy_S->ecRule[5]);
				pec=5;
			}		
			else										
			{
				ecFuzzy[0] =1.0;
				pec=6;
			}
			
			ecFuzzy[1] = 1.0 - ecFuzzy[0];
			break;
		}//...end case 7 
	case 5:
		{
                        /*-----���������������-----*/
			if(Fuzzy_S->fe < Fuzzy_S->eRule[0])			// |x_x_x_x_x_ 
			{
				eFuzzy[0] =1.0; 
				pe= 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[1])               // _x|x_x_x_x_ 
			{
				eFuzzy[0] = (Fuzzy_S->eRule[1]-Fuzzy_S->fe)/(Fuzzy_S->eRule[1]-Fuzzy_S->eRule[0]);
				pe = 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[2])               // _x_x|x_x_x_ 
			{
				eFuzzy[0] = (Fuzzy_S->eRule[2] -Fuzzy_S->fe)/(Fuzzy_S->eRule[2]-Fuzzy_S->eRule[1]);
				pe =1;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[3])               // _x_x_x|x_x_ 
			{
				eFuzzy[0] = (Fuzzy_S->eRule[3] -Fuzzy_S->fe)/(Fuzzy_S->eRule[3]-Fuzzy_S->eRule[2]);
				pe =2;
			}
			else if(Fuzzy_S->fe<Fuzzy_S->eRule[4])		        // _x_x_x_x|x_
			{   eFuzzy[0] = (Fuzzy_S->eRule[4]-Fuzzy_S->fe)/(Fuzzy_S->eRule[4]-Fuzzy_S->eRule[3]);
				pe=3;
			}
			else							// _x_x_x_x_x| 
			{
				eFuzzy[0] =1.0;
				pe =4;
			}
			eFuzzy[1] = 1.0 - eFuzzy[0];

			/*-----���仯������������-----*/
			if(Fuzzy_S->fec < Fuzzy_S->ecRule[0])
			{
				ecFuzzy[0] =1.0;
				pec = 0;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[1])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[1] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[1]-Fuzzy_S->ecRule[0]);
				pec = 0 ;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[2])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[2] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[2]-Fuzzy_S->ecRule[1]);
				pec = 1;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[3])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[3] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[3]-Fuzzy_S->ecRule[2]);
				pec = 2 ;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[4])
			{   ecFuzzy[0] = (Fuzzy_S->ecRule[4] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[4]-Fuzzy_S->ecRule[3]);
				pec = 3;
			}
			else
			{
				ecFuzzy[0] =1.0;
				pec = 4;
			}
			
			ecFuzzy[1] = 1.0 - ecFuzzy[0];
			break;
		}//...end case 5
	case 3:
		{
                        /*-----���������������-----*/
			if(Fuzzy_S->fe < Fuzzy_S->eRule[0])			// |x_x_x_
			{
				eFuzzy[0] =1.0; 
				pe= 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[1])		// _x|x_x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[1]-Fuzzy_S->fe)/(Fuzzy_S->eRule[1]-Fuzzy_S->eRule[0]);
				pe = 0;
			}
			else if(Fuzzy_S->fe < Fuzzy_S->eRule[2])		// _x_x|x_
			{
				eFuzzy[0] = (Fuzzy_S->eRule[2] -Fuzzy_S->fe)/(Fuzzy_S->eRule[2]-Fuzzy_S->eRule[1]);
				pe =1;
			}
			else							// _x_x_x|
			{
				eFuzzy[0] =1.0;
				pe =2;
			}
			eFuzzy[1] = 1.0 - eFuzzy[0];

			/*-----���仯������������-----*/
			if(Fuzzy_S->fec < Fuzzy_S->ecRule[0])
			{
				ecFuzzy[0] =1.0;
				pec = 0;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[1])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[1] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[1]-Fuzzy_S->ecRule[0]);
				pec = 0 ;
			}
			else if(Fuzzy_S->fec < Fuzzy_S->ecRule[2])
			{
				ecFuzzy[0] = (Fuzzy_S->ecRule[2] - Fuzzy_S->fec)/(Fuzzy_S->ecRule[2]-Fuzzy_S->ecRule[1]);
				pec = 1;
			}
			else
			{
				ecFuzzy[0] =1.0;
				pec = 2;
			}
			
			ecFuzzy[1] = 1.0 - ecFuzzy[0];
			break;
		}//...end case 3 
	default: break;
	}//...end switch
	/*��ѯģ�������*/
	if(pe<(rank-1) && pec<(rank-1))        // e��e'��û�дﵽ��Ե
	{
		num =Fuzzy_S->rule[pec][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[0];
                
		num =Fuzzy_S->rule[pec][pe+1];
		U1Fuzzy[num] += eFuzzy[1]*ecFuzzy[0];	
                
		num =Fuzzy_S->rule[pec+1][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[1];

		num =Fuzzy_S->rule[pec+1][pe+1];
		U1Fuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	}
	else if(pe==(rank-1) && pec<(rank-1))  // e�ﵽ��Ե
	{
		num =Fuzzy_S->rule[pec][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[0];
                
		num =Fuzzy_S->rule[pec+1][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[1];
	}
	else if(pe<(rank-1) && pec==(rank-1))  // e'�ﵽ��Ե
	{
		num =Fuzzy_S->rule[pec][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[0];	
                
		num =Fuzzy_S->rule[pec][pe+1];
		U1Fuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	}
	else				       // e��e'ͬʱ�ﵽ��Ե
	{
		num =Fuzzy_S->rule[pec][pe];
		U1Fuzzy[num] += eFuzzy[0]*ecFuzzy[0];		
	}
	/*������ķ���ģ��*/
	for(i=0;i<rank;i++)
		kp+=U1Fuzzy[i]*Fuzzy_S->U1Rule[i];
    return(kp);
}

/****************************
Fuction: Caculate Fuzzy Value
Par.   : FuzzyStruct* F_S
         float ek
         float ekc
Return : Fuzzy Value
****************************/
double Fuzzy_Update(FuzzyStruct* F_S,double ek,double ekc)
{
  double value=0;
  F_S->fe=ek;
  F_S->fec=ekc;
  value=FuzzyCtrl(F_S);
  return value;
}//...end Fuzzy_Update();







