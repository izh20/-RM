#ifndef _FUZZY_H
#define _FUZZY_H


typedef enum
{
Fuzzy_Rank7=7,
Fuzzy_Rank5=5,
Fuzzy_Rank3=3,
}Fuzzy_Rank_e; // Fuzzy Rank Enum

typedef struct
{
Fuzzy_Rank_e Rank;  // 分级数量
double fe;  // e(k)
double fec; // e(k)'
double eRule[7];   //误差隶属度函数中心值
double ecRule[7];  //误差变化隶属度函数中心值
double U1Rule[7];  //输出隶属函数中心值
int rule[7][7];
}FuzzyStruct;  // 模糊结构体


extern double Fuzzy_Update(FuzzyStruct* F_S,double ek,double ekc);
extern double FuzzyCtrl(FuzzyStruct* Fuzzy_S);

#endif

//FuzzyStruct duoji_P1={
// Fuzzy_Rank7,
// 0,
// 0,
// {-130,-80,-40,0,40,80,130},
// {-12,-8,-4,0,4,8,12},
// {0.35,0.37,0.39,0.41,0.43,0.45,0.47},                          // p-


// {

//    {6,5,4,3,3,2,1},
//    {5,4,3,2,2,1,2},
//    {4,3,2,1,1,2,3},
//    {3,2,1,0,1,2,3},
//    {3,2,1,1,2,3,4},
//    {2,1,2,2,3,4,5},
//    {1,2,3,3,4,5,6},

//  
// }
//} ;
