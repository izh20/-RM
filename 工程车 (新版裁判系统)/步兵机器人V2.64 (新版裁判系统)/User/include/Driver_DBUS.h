#ifndef YAOKONG_X_H
#define YAOKONG_X_H


#include "common.h"

typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t	s1;
		uint8_t	s2;
	}rc;

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;

	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;
extern uint16_t aim_angle_206;
extern uint16_t aim_angle_205;

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_W ((uint16_t)0x01<<0)   //1
#define KEY_PRESSED_S ((uint16_t)0x01<<1)   //2
#define KEY_PRESSED_A ((uint16_t)0x01<<2)   //4
#define KEY_PRESSED_D ((uint16_t)0x01<<3)   //8
#define KEY_PRESSED_SHIFT ((uint16_t)0x01<<4) //16
#define KEY_PRESSED_CTRL ((uint16_t)0x01<<5)   //32
#define KEY_PRESSED_Q ((uint16_t)0x01<<6)     //64
//#define KEY_PRESSED_E ((uint16_t)0x01<<7)  Êµ¼ÊÎ´²âµ½
#define KEY_PRESSED_R ((uint16_t)0x01<<8)   //256
#define KEY_PRESSED_F ((uint16_t)0x01<<9)    //512

#define KEY_PRESSED_Z ((uint16_t)0x01<<11)    //2048
#define KEY_PRESSED_X ((uint16_t)0x01<<12)    //4096
#define KEY_PRESSED_C ((uint16_t)0x01<<13)    //8192
#define KEY_PRESSED_V ((uint16_t)0x01<<14)    //16384
void DBUS_Init(void);
void Yaokong_translate(void);
void MotorTargetChange(void);

void delay_ms(unsigned int t);

void printf_what_I_want(void);

#endif
