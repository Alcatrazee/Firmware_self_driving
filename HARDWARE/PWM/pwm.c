#include "pwm.h"
#include "led.h"
#include "usart.h"

//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM5_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //GPIOF9����Ϊ��ʱ��14
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;   //    DIR2 DIR2 DIR3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //DIR1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5 ,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM5 , &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 4OC1
	TIM_OC2Init(TIM5 , &TIM_OCInitStructure);
	
	TIM_CtrlPWMOutputs(TIM5,ENABLE);

	TIM_OC1PreloadConfig(TIM5 , TIM_OCPreload_Enable);  //ʹ��TIM54��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM5 , TIM_OCPreload_Enable); 
 
  TIM_ARRPreloadConfig(TIM5 ,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM5 , ENABLE);  //ʹ��TIM5			  
	
}  


void motor2_pwm(u16 newPwm,u8 dir){							//right wheel
	if(dir==backward){
		PCout(2)=0;			//DIR1
		PAout(2)=1;			//DIR2
		TIM_SetCompare1(TIM5 ,newPwm);
	}else if (dir==forward){
		PCout(2)=1;			//DIR1
		PAout(2)=0;			//DIR2
		TIM_SetCompare1(TIM5 ,newPwm);
	}
}

void motor1_pwm(u16 newPwm,u8 dir){							//left wheel
	if(dir==forward){
		PAout(3)=0;			//DIR3
		PAout(4)=1;			//DIR4
		TIM_SetCompare2(TIM5 ,newPwm);
	}else if (dir==backward){
		PAout(3)=1;			//DIR3
		PAout(4)=0;			//DIR4
		TIM_SetCompare2(TIM5 ,newPwm);
	}
}

void Run_as_vol(float vol,u8 motornum){
	float v_temp;
	v_temp = vol;
	if(vol<0){
		v_temp = -v_temp;
	}
	switch(motornum){
		case motor1: 
					if(vol>=0)
						motor1_pwm(v_temp,forward);
					else if(vol<0)
						motor1_pwm(v_temp,backward);
		break;
		case motor2:
					if(vol>=0)
						motor2_pwm(v_temp,forward);
					else if(vol<0)
						motor2_pwm(v_temp,backward);
		break;
	}
}

void stop(u8 motornum){
	switch(motornum){
		case motor1: 
			motor1_pwm(0,forward);
		break;
		case motor2:
			motor2_pwm(0,forward);
		break;
	}
}

void stop_all(void){
	stop(motor1);
	stop(motor2);
}

float motor_pid(float exp_vol,float vol,u8 motor_n){
	float out;
	float error_vol;
	double error_sum;
	static float error_sum1,error_sum2;
	static float former_vol1,former_vol2;
	float A;
	float Kp,Ki,Kd;
	//float Kp_a=0,Ki_a=0,Kd_a=0;

	float Kp1 = 00.00800,Ki1 = 0.0000000000,Kd1 = 00.00001;//Kp1 = 0.008,Ki1 = 0.0000009000,Kd1 = 0.00000060;
	float Kp2 = 00.00800,Ki2 = 00.00000,Kd2 = 00.00001;//float Kp2 = 0.00075,Ki2 = 0.00001,Kd2 = 0.000015;
	if(exp_vol<100){
		error_sum1 = 0;
		error_sum2 = 0;
	}
	
	switch(motor_n){
		case 1: A = vol-former_vol1;	Kp = Kp1;	Ki = Ki1; Kd = Kd1;	error_sum = error_sum1;	break;
		case 2: A = vol-former_vol2;	Kp = Kp2;	Ki = Ki2; Kd = Kd2;	error_sum = error_sum2;	break;
	}
	error_vol = exp_vol - vol;
	error_sum+=error_vol;
	out = Kp*error_vol+Ki*error_sum+Kd*A;		//Kd��Ҫ����һ��ʱ��dt   Kd*A/dt
	
	switch(motor_n){
		case 1: error_sum1 = error_sum;break;
		case 2: error_sum2 = error_sum;break;
	}
	if(out>1400)
		out = 1400;
	else if(out<-1400)
		out = -1400;
	return out;
}




//Ki=0.0000016015
//d = 0.00005












