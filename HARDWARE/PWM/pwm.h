#ifndef __PWM__H__
#define __PWM__H__
#include "sys.h"
  
#define forward  1
#define backward 0
#define motor1 1
#define motor2 2 
#define motor3 3
#define motor4 4

void TIM5_PWM_Init(u32 arr,u32 psc);
void motor1_pwm(u16 newPwm,u8 dir);
void motor2_pwm(u16 newPwm,u8 dir);
void motor3_pwm(u16 newPwm,u8 dir);
void motor4_pwm(u16 newPwm,u8 dir);
void Run_as_vol(float vol,u8 motornum);
void stop(u8 motornum);
float motor_pid(float exp_vol,float vol,u8 motor_n);
void stop_all(void);

#endif
