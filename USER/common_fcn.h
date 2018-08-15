#ifndef COMMON_FCN__H__
#define COMMON_FCN__H__


// this file is used to store some structures and common functions

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

float my_abs(float num);
u8 strcmp_real(char str1[3],char str2[3]);
float my_atof(char str[4]);
double gps_atof(char *s);
int is_space(char ch);
int is_digit(char ch);

#define radius 85
#define pi 3.14159f
#define remote_control_mode 0
#define com_control_mode    1

typedef struct
{
	short   frame_X;						//车体X位置	 单位：毫米
	short 	frame_Y;						//车体Y位置	 单位：毫米	
	float 	frame_Vx;
	float 	frame_Vy;
	float 	omega;
	float 	angle;							//车体角度	 单位：度								
	float   frame_ax;
	float   frame_ay;
	float   omega_wheel[3];
	float   angle_offset;
}RB_State;
#endif



