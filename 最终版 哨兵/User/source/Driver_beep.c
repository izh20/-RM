/**
  *@file Driver_beep.c
  *@date 2016-12-13
  *@author Albert.D
  *@brief 
  */
  
#include "Driver_beep.h"

const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};
//启动成功
const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
{
  So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
};
//送别
const Sound_tone_e X_music[] = 
{
    So5M,So5M,So5M,So5M, Mi3M,Mi3M,So5M,So5M, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Do1H,Do1H,
	La6M,La6M,La6M,La6M, Do1H,Do1H,Do1H,Do1H, So5M,So5M,So5M,So5M, So5M,So5M,So5M,So5M,
	So5M,So5M,So5M,So5M, Do1M,Do1M,Re2M,Re2M, Mi3M,Mi3M,Mi3M,Mi3M, Re2M,Re2M,Do1M,Do1M,
	Re2M,Re2M,Re2M,Re2M, Re2M,Re2M,Re2M,Re2M, Re2M,Re2M,Re2M,Re2M, Silent,Silent,Silent,Silent,
	So5M,So5M,So5M,So5M, Mi3M,Mi3M,So5M,So5M, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Si7M,Si7M,
	La6M,La6M,La6M,La6M, Do1H,Do1H,Do1H,Do1H, So5M,So5M,So5M,So5M, So5M,So5M,So5M,So5M,
	So5M,So5M,So5M,So5M, Re2M,Re2M,Mi3M,Mi3M, Fa4M,Fa4M,Fa4M,Fa4M, Fa4M,Fa4M,Si7L,Si7L,
	Do1M,Do1M,Do1M,Do1M, Do1M,Do1M,Do1M,Do1M, Do1M,Do1M,Do1M,Do1M, Do1M,Do1M,Do1M,Do1M,
	La6M,La6M,La6M,La6M, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Do1H,Do1H,
	Si7M,Si7M,Si7M,Si7M, La6M,La6M,Si7M,Si7M, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Do1H,Do1H,
	La6M,La6M,Si7M,Si7M, Do1H,Do1H,La6M,La6M, La6M,La6M,So5M,So5M, Mi3M,Mi3M,Do1M,Do1M,
	Re2M,Re2M,Re2M,Re2M, Re2M,Re2M,Re2M,Re2M, Re2M,Re2M,Re2M,Re2M, Silent,Silent,Silent,Silent,
	So5M,So5M,So5M,So5M, Mi3M,Mi3M,So5M,So5M, Do1H,Do1H,Do1H,Do1H, Do1H,Do1H,Si7M,Si7M,
	La6M,La6M,La6M,La6M, Do1H,Do1H,Do1H,Do1H, So5M,So5M,So5M,So5M, So5M,So5M,So5M,So5M,
	So5M,So5M,So5M,So5M, Re2M,Re2M,Mi3M,Mi3M, Fa4M,Fa4M,Fa4M,Fa4M, Fa4M,Fa4M,Si7L,Si7L,
	Do1M,Do1M,Do1M,Do1M, Do1M,Do1M,Do1M,Do1M, Do1M,Do1M,Do1M,Do1M, Silent,Silent,Silent,Silent,
};

//
void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}

//play the start up music
void Sing_Startup_music(uint32_t index)
{
  if(index < Startup_Success_music_len)
    Sing(Mavic_Startup_music[index]);
}
//play the X music
void Sing_X_music(uint32_t index)
{
  if(index < X_music_len)
    Sing(X_music[index]);
}

