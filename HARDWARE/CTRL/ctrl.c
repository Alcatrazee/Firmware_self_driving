#include "ctrl.h"
#include "common_fcn.h"
#include "mpu9250.h"
#include "includes.h"
#include "motor.h"

#define Axis_x 0
#define Axis_y 1

//#define STOPMOVING


u8 Process_finish_flag=0;
float Gyro_Offset[3]={0,0,0};
float Acc_Offset[3]={0,0,0};
float ax,ay,gz;
float usable_gz=0;
float deg_angle = 0;
int vol_wL=0,vol_wR=0;

extern double arr_L,arr_R;

extern RB_State State,Exp_State;

// get the offset of ax ay gz
void offset_cut(void){
	unsigned int count = 5000,i;
	float gyro_sum[3]={0,0,0};
	float accel_sum[3]={0,0,0};
	for(i=count;i>0;i--){
		READ_MPU9250_ACCEL(&Acc.X,&Acc.Y,&Acc.Z);
		READ_MPU9250_GYRO(&Gyro.X,&Gyro.Y,&Gyro.Z);
		
		gyro_sum[0] += (float)Gyro.X*2000/32767;
		gyro_sum[1] += (float)Gyro.Y*2000/32767;
		gyro_sum[2] += (float)Gyro.Z*2000/32767;
		
		accel_sum[0]+=((float)Acc.X/32767*19.6f);
		accel_sum[1]+=((float)Acc.Y/32767*19.6f);
		accel_sum[2]+=((float)Acc.Z/32767*19.6f);
	}
	Gyro_Offset[0] = (float)gyro_sum[0]/count;
	Gyro_Offset[1] = (float)gyro_sum[1]/count;
	Gyro_Offset[2] = (float)gyro_sum[2]/count;
	Acc_Offset[0] = (float)accel_sum[0]/count;
	Acc_Offset[1] = (float)accel_sum[1]/count;
	Acc_Offset[2] = (float)accel_sum[2]/count;
}

// get the true data we need
void Get_IMU_Data(void){
	static float former_time=0;
	float current_time=0;
	OS_ERR err;
	READ_MPU9250_ACCEL(&Acc.X,&Acc.Y,&Acc.Z);
	READ_MPU9250_GYRO(&Gyro.X,&Gyro.Y,&Gyro.Z);
	
	IMU_DAT.ax = State.frame_ax=((float)Acc.X/32767*19.6f)-Acc_Offset[0];
	IMU_DAT.ay = State.frame_ay=((float)Acc.Y/32767*19.6f)-Acc_Offset[1];
	IMU_DAT.az =((float)Acc.Z/32767*19.6f);
	
	IMU_DAT.gx = ((float)Gyro.X/32767*2000)-Gyro_Offset[0];
	IMU_DAT.gy = ((float)Gyro.Y/32767*2000)-Gyro_Offset[1];
	IMU_DAT.gz = usable_gz = ((float)Gyro.Z/32767*2000)-Gyro_Offset[2];
	
	current_time = ((float)OSTimeGet(&err)*5)/1000;
	deg_angle+=usable_gz*(current_time-former_time);
	former_time = current_time;

	State.angle = deg_angle;
	State.omega = usable_gz;
	State.angle = deg_angle;
 // for debug 
	//printf("%.2f\t%.2f\t%.2f\t%d\r\n",usable_gz,ay,State.angle,OSTimeGet(&err)*5);
}

void Process_Signal(u16 temp[4],float linear_v_oy[2]){
	float k1=6.1224,k2=1.96078;
	float b1=-9244.898,b2=-4000;
	float k=0,b=0;
	float k_vertical=0.09375;
	float temp_signal=0;
	if(temp[0]>1520){
		k=k1;
		b=b1;
	}else if(temp[0]<1500) {
		k=k2;
		b=b2;
	}
	
	if((temp[1]>1000&&temp[1]<2050)&&(temp[1]>1530||temp[1]<1500)){
		temp_signal=my_abs(temp[1]-1517);
		linear_v_oy[0]=k_vertical*temp_signal;
		if(temp[1]>1530)
			linear_v_oy[0]=-linear_v_oy[0];
	}else{
		linear_v_oy[0]=0;
	}
	linear_v_oy[1]=-((float)temp[0]*k+b);												//y axis
}


void Speed_Moto_Control(float linear_xyz[2],float linear_v[2])
{ 
  linear_v[1]   = (linear_xyz[0]+linear_xyz[1]);
  linear_v[0]   = (linear_xyz[1]-linear_xyz[0]);
	//printf("%.2f\t%.2f\t%.2f\t%.2f\r\n",linear_xyz[0],linear_xyz[1],linear_v[0],linear_v[1]);
}

#define L			400.0f
#define pulse	1024
void Move(float linear_v[2]){
		float tempL=0,tempR=0;
		float omega1=0,omega2=0;
		omega1 = (linear_v[0]/L)*pulse;
		omega2 = (linear_v[1]/L)*pulse;
		
		tempL=prescaler_calculator(my_abs(omega1));
		tempR=prescaler_calculator(my_abs(omega2));
	
		if(omega1>0){
			DIRL=1;
		}else if(omega1<0){
			DIRL=0;
		}
		arr_L=tempL;
		
		if(omega2<0){
			DIRR=0;
		}else if(omega2>0){
			DIRR=1;
		}
	  arr_R=tempR;
		//printf("%f\t%f\t\r\n",omega1,omega2);
}

void Controller(u16 signal[4]){
	CPU_SR_ALLOC();
	float linear_v_oy[2]={0,0};						//声明局部变量，目标速度，y轴与z轴速度
	float linear_v[2]={0};								//最终落到两个轮子的线速度（由于没有闭环，因此不一定准）
	float linear_vxy[2]={0};							//过渡变量，作用：计算过程产物，可以优化掉
	Get_IMU_Data();												//获取IMU数据，获取六轴数据
	
	OS_CRITICAL_ENTER();									//进入临界区
	Process_Signal(signal,linear_v_oy);		//处理输入信号signal[4]
	OS_CRITICAL_EXIT();										//退出临界区
	
	linear_vxy[1]=linear_v_oy[1];					
	Exp_State.omega=linear_v_oy[0];				//更新期望值
	
	linear_vxy[0] = omega_PID(Exp_State.omega,usable_gz);//角速度PID
	Exp_State.angle=State.angle;					//由于使用角速度闭环，因此期望值等于现在的值
	
	if(linear_vxy[0]>2000)								//限幅
		linear_vxy[0]=2000;
	else if(linear_vxy[0]<-2000)
		linear_vxy[0]=-2000;

	Speed_Moto_Control(linear_vxy,linear_v);//差速轮移动机器人逆运动学求解
	
#ifndef STOPMOVING
	Move(linear_v);												//行动，即移动机构
#endif
}

#define max_out 2000
#define min_out -max_out
float omega_PID(float exp_omega,float current_omega){
	OS_ERR err;
	static float former_time=0;
	float current_time=0;
	float Kp_a = 040.000,
				Ki_a = 100.0000000,																			//ki=120
				Kd_a = 20.00000;
	float error_sum=0,error,d_error=0;
	static float error_f;
	float out;
	
	error = exp_omega - current_omega*27/30;
	error_sum+=error;
	
	current_time = ((float)OSTimeGet(&err)*5)/1000;

	
	d_error = (error - error_f)/(current_time-former_time);
	
	former_time = current_time;
	
	out = Kp_a*error + Ki_a*error_sum + Kd_a*d_error;
	
	if(out>=max_out)
		out = max_out;
	else if(out<-max_out)
		out = -max_out;
	
	error_f = error;

	return -out;
}


