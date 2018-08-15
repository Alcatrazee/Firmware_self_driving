#ifndef CTRL__H__
#define CTRL__H__

#include "sys.h"

void offset_cut(void);
void Get_IMU_Data(void);
void Controller(u16 signal[4]);
void Process_Signal(u16 temp[4],float linear_v_oy[2]);
void Speed_Moto_Control(float linear_xyz[2],float linear_v[2]);
void Move(float linear_v[2]);
float omega_PID(float exp_omega,float current_omega);
#endif


