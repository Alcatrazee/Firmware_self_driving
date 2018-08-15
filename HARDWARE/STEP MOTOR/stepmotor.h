#ifndef STEPMOTOR__H__
#define STEPMOTOR__H__

#include "sys.h"
#include "common_fcn.h"

void Step_motor_init(void);
void Step1_Pulse(void);
void Step2_Pulse(void);
void Step3_Pulse(void);
//user functions
void process_input(u8 carbinet_axis);
short delta_pos(u8 carbinet_axis);
void Saturation(u8 carbinet_axis);
u8 Judge_need_avg_stage(u8 carbinet_axis);
void Get_avg_steps(u8 carbinet_axis);
void Get_none_avg_steps(u8 carbinet_axis);

typedef struct{
	u16 acc2avg;
	u16 avg2dac;
	u16 end_steps;
	u16 total_steps;
}Change_steps;

typedef struct{
	u16 x;
	u16 z;
	u16	theta;
}Carbinet_step_data;

typedef struct{
	Change_steps x;
	Change_steps z;
	Change_steps theta;
}Change_steps_data;

typedef struct{
	u16 pos;
	u16 steps;
	Change_steps change_state_steps;
	u8 Working_or_not;
	u8 step_motor_stage;
	u8 movement_class;
	u8 data_process_flag;
	short TIM_FREQ;
}Carbinet_stm_Axis_Data;

typedef struct{
	u8 z;
	u8 x;
	u8 theta_zero;
	u8 theta_full;
	u8 arm_zero;
	u8 arm_full;
	u8 wrist_zero;
	u8 wrist_full;
}Limit_Switch_Data;

typedef struct{
	Carbinet_stm_Axis_Data x;
	Carbinet_stm_Axis_Data z;
	Carbinet_stm_Axis_Data theta;
	Limit_Switch_Data LSD;
}Carbinet_State;

typedef struct{
	u16 x_pos;
	u16 z_pos;
	u16 theta_pos;
}Carbinet_Exp_State;

extern Carbinet_State carbinet_state;
extern Carbinet_Exp_State carbinet_exp_state;

#define stop 0
#define normal 1
#define no_avg_stage 2
#define acclerate 3
#define avg 4
#define disacc 5

#define avg_freq 3000
#define Jerk 4
#define deacc_Jerk 4

#define x_avg_freq 4800
#define x_Jerk 9
#define x_deacc_Jerk 9

#define theta_avg_freq 1200
#define theta_Jerk 1
#define theta_deacc_Jerk 1

#define Not_working 0
#define Working     1

#define carbinet_x 0
#define carbinet_z 1
#define carbinet_theta 2

#endif



