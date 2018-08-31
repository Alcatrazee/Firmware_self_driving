#include "stepmotor.h"
#include "sys.h"
#include "common_fcn.h"
#include "math.h"
#include "includes.h"

// STM MOTOR 1 is z axis motor
//					 2    x
//					 3    theta

//#define ENA1 PEout(7)
#define DIR1 PAout(7) 
#define PUL1 PAout(6)
#define DIR2 PBout(1) 
#define PUL2 PBout(0)
#define DIR3 PCout(5) 
#define PUL3 PCout(4)

void Step_motor_init(void){

	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);	 //使能PB,PE端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;			//stm 2 
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;			//stm 2 
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
	PUL1 = 1;						 
	DIR1 = 1;
	PUL2 = 1;						 
	DIR2 = 1;
	PUL3 = 1;						 
	DIR3 = 1;
// ENA1 = 1;
 
}

void Step1_Pulse(void){
	PUL1 = ~PUL1;
}

void Step2_Pulse(void){
	PUL2 = ~PUL2;
}

void Step3_Pulse(void){
	PUL3 = ~PUL3;
}

void process_input(u8 carbinet_axis){
	short delta=delta_pos(carbinet_axis);
	if(my_abs(delta)){
		//step 1 : saturation 
		Saturation(carbinet_axis);
		//step 2 : judge if we need the avg stage
		if(Judge_need_avg_stage(carbinet_axis)){
			//step 3 : we need the avg stage and process the flag
			Get_avg_steps(carbinet_axis);
			switch(carbinet_axis){
				case carbinet_z:carbinet_state.z.movement_class = normal;break;
				case carbinet_x:carbinet_state.x.movement_class = normal;break;
				case carbinet_theta:carbinet_state.theta.movement_class = normal;break;
			}
		}else{
			// step 3 : we don't need the avg stage 
			Get_none_avg_steps(carbinet_axis);
			switch(carbinet_axis){
				case carbinet_z:carbinet_state.z.movement_class = no_avg_stage;break;
				case carbinet_x:carbinet_state.x.movement_class = no_avg_stage;break;
				case carbinet_theta:carbinet_state.theta.movement_class = no_avg_stage;break;
			}
		}
		//step 4 : start the moving and change the pole DIR which controls the direction
			switch(carbinet_axis){
				case carbinet_z:
					carbinet_state.z.step_motor_stage = acclerate;
					if(delta>0)
						DIR1 = 0;
					else 
						DIR1 = 1;
					carbinet_state.z.Working_or_not=1;
					TIM_Cmd(TIM3,ENABLE);
					break;
				case carbinet_x:
					carbinet_state.x.step_motor_stage = acclerate;
					if(delta>0)
						DIR2 = 0;
					else 
						DIR2 = 1;
					carbinet_state.x.Working_or_not=1;
					TIM_Cmd(TIM1,ENABLE);
					break;
				case carbinet_theta:
					carbinet_state.theta.step_motor_stage = acclerate;
					if(delta>0)
						DIR3 = 1;
					else 
						DIR3 = 0;
					carbinet_state.theta.Working_or_not=1;
					TIM_Cmd(TIM10,ENABLE);
					break;
			}
		
	}else{
		switch(carbinet_axis){
			case carbinet_z: carbinet_state.z.Working_or_not=0;break;
			case carbinet_x: carbinet_state.x.Working_or_not=0;break;
			case carbinet_theta: carbinet_state.theta.Working_or_not=0;break;
		}
	}
}
// get the delta distance of two points
short delta_pos(u8 carbinet_axis){
	short delta=0;
	switch(carbinet_axis){
		case carbinet_z: delta = (short)carbinet_exp_state.z_pos - (short)carbinet_state.z.pos;break;
		case carbinet_x: delta = (short)carbinet_exp_state.x_pos - (short)carbinet_state.x.pos;break;
		case carbinet_theta: delta = (short)carbinet_exp_state.theta_pos - (short)carbinet_state.theta.pos;break;
	}
	return delta;
}

#define edge_pos_z 5020				//unit 1 unit = 0.1mm
#define edge_pos_x 5850				//unit
#define edge_pos_theta 180    //degree
// to avoid crosssing the border
void Saturation(u8 carbinet_axis){
	switch(carbinet_axis){
		case carbinet_z:
			if(carbinet_exp_state.z_pos>=edge_pos_z)
				carbinet_exp_state.z_pos = edge_pos_z;
			else if(carbinet_exp_state.z_pos<=0)
				carbinet_exp_state.z_pos = 0;
			break;
		case carbinet_x:
			if(carbinet_exp_state.x_pos>=edge_pos_x)
				carbinet_exp_state.x_pos = edge_pos_x;
			else if(carbinet_exp_state.x_pos<=0)
				carbinet_exp_state.x_pos = 0;
			break;
		case carbinet_theta:
			if(carbinet_exp_state.theta_pos>=edge_pos_theta)
				carbinet_exp_state.theta_pos = edge_pos_theta;
			else if(carbinet_exp_state.theta_pos<=0)
				carbinet_exp_state.theta_pos = 0;
			break;
	}
	
}
#define acc_distance_z 334
#define acc_distance_x 140
// judge if it needs to go in avg stage
// return : 1: need avg stage
//					0: do not need ... 
u8 Judge_need_avg_stage(u8 carbinet_axis){
	short temp = delta_pos(carbinet_axis);
	switch(carbinet_axis){
		case carbinet_z:  if(my_abs(temp)>acc_distance_z)
												return 1;
											else 
												return 0;
		case carbinet_x:  if(my_abs(temp)>acc_distance_x)
												return 1;
											else 
												return 0;
		default:	return 1;
	}
}
// get the avg stage step position
// length of avg steps = delta_distance * pulse_per_unit - 2*(accleration_steps)
// position of avg speed period = current position + accleration steps
// position of deaccleration = position of avg speed + length of avg steps
// cofficient = input*18000/5020
#define acc_steps 600
#define theta_acc_steps 600
#define cofficient_z 3.585657f
#define cofficient_x 8.560683f
#define cofficient_theta 106.9444444444444f
//with the Jerk = 1 hz per gain,acc steps is 385
void Get_avg_steps(u8 carbinet_axis){
	float avg_stage_pulse=0;
	short delta = delta_pos(carbinet_axis);
	u16 steps_theta = 21250;
	switch(carbinet_axis){
		case carbinet_z:	carbinet_state.z.change_state_steps.total_steps = (u16)(my_abs(carbinet_exp_state.z_pos-carbinet_state.z.pos)*cofficient_z+0.5f);
											if(carbinet_exp_state.z_pos-carbinet_state.z.pos<0)
												carbinet_state.z.change_state_steps.end_steps = carbinet_state.z.steps-carbinet_state.z.change_state_steps.total_steps;
											else 
												carbinet_state.z.change_state_steps.end_steps = carbinet_state.z.steps+carbinet_state.z.change_state_steps.total_steps;
											avg_stage_pulse = carbinet_state.z.change_state_steps.total_steps-acc_steps*2;     // get length of pulse of avg stage
											if(delta>0){
												carbinet_state.z.change_state_steps.acc2avg = carbinet_state.z.pos*cofficient_z + acc_steps;													 // set the position of change point1
												carbinet_state.z.change_state_steps.avg2dac = carbinet_state.z.pos*cofficient_z + acc_steps + avg_stage_pulse; // set the position of change point2
											}else{
												carbinet_state.z.change_state_steps.acc2avg = carbinet_state.z.pos*cofficient_z - acc_steps;													 // set the position of change point1
												carbinet_state.z.change_state_steps.avg2dac = carbinet_state.z.pos*cofficient_z - acc_steps - avg_stage_pulse; // set the position of change point2
											}
											
											break;
											
		case carbinet_x:	carbinet_state.x.change_state_steps.total_steps = (u16)(my_abs(carbinet_exp_state.x_pos-carbinet_state.x.pos)*cofficient_x+0.5f);
											if(carbinet_exp_state.x_pos-carbinet_state.x.pos<0)
												carbinet_state.x.change_state_steps.end_steps = carbinet_state.x.steps-carbinet_state.x.change_state_steps.total_steps;
											else 
												carbinet_state.x.change_state_steps.end_steps = carbinet_state.x.steps+carbinet_state.x.change_state_steps.total_steps;
											avg_stage_pulse = carbinet_state.x.change_state_steps.total_steps-acc_steps*2;     // get length of pulse of avg stage
											if(delta>0){
												carbinet_state.x.change_state_steps.acc2avg = carbinet_state.x.pos*cofficient_x + acc_steps;													 // set the position of change point1
												carbinet_state.x.change_state_steps.avg2dac = carbinet_state.x.pos*cofficient_x + acc_steps + avg_stage_pulse; // set the position of change point2
											}else{
												carbinet_state.x.change_state_steps.acc2avg = carbinet_state.x.pos*cofficient_x - acc_steps;													 // set the position of change point1
												carbinet_state.x.change_state_steps.avg2dac = carbinet_state.x.pos*cofficient_x - acc_steps - avg_stage_pulse; // set the position of change point2
											}
											break;						
		case carbinet_theta:
											if(delta>0){
												carbinet_state.theta.change_state_steps.total_steps = steps_theta;
											  carbinet_state.theta.change_state_steps.end_steps = steps_theta;
												carbinet_state.theta.change_state_steps.acc2avg = carbinet_state.theta.pos*cofficient_theta + theta_acc_steps;
												carbinet_state.theta.change_state_steps.avg2dac = carbinet_state.theta.change_state_steps.end_steps - theta_acc_steps;
											}else{
												carbinet_state.theta.change_state_steps.total_steps = steps_theta;
											  carbinet_state.theta.change_state_steps.end_steps = 0;
												carbinet_state.theta.change_state_steps.acc2avg = carbinet_state.theta.pos*cofficient_theta - theta_acc_steps;		
												carbinet_state.theta.change_state_steps.avg2dac = carbinet_state.theta.change_state_steps.end_steps + theta_acc_steps; 
											}
											break;
	
	}
//	printf("%f\t%d\t%d\t\r\n",avg_stage_pulse,change_state_steps.acc2avg,change_state_steps.avg2dac);
}

void Get_none_avg_steps(u8 carbinet_axis){
	switch(carbinet_axis){
		case carbinet_z:carbinet_state.z.change_state_steps.acc2avg = ((float)my_abs(carbinet_exp_state.z_pos-carbinet_state.z.pos)/2)*cofficient_z;break;
		case carbinet_x:carbinet_state.x.change_state_steps.acc2avg = ((float)my_abs(carbinet_exp_state.x_pos-carbinet_state.x.pos)/2)*cofficient_x;break;
	}
}

void Change_dir(u8 carbinet_axis,short delta){
	switch(carbinet_axis){
		case carbinet_z: 
			if(delta<0)
				DIR1 = 1;
			else
				DIR1 = 0;
			break;
		case carbinet_x: 
			if(delta<0)
				DIR2 = 1;
			else
				DIR2 = 0;
			break;
		case carbinet_theta:
			if(delta<0)
				DIR3 = 1;
			else
				DIR3 = 0;
			break;
	}
}



