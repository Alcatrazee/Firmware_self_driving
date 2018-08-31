#include "timer.h"
#include "led.h"
#include "includes.h"
#include "stepmotor.h"
#include "common_fcn.h"

extern RB_State State;
extern RB_State Exp_State;

void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_Cmd(TIM3, DISABLE);  //ʹ��TIMx����	
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

		 
}
u16 freq=0;
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	OSIntEnter();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{	
		Step1_Pulse();
		if((GPIOA->ODR&0x00C0)==0x00C0){
			carbinet_state.z.steps--;
		}
		else if((GPIOA->ODR&0x00C0)==0x0040){
			carbinet_state.z.steps++;
		}
		
		if(carbinet_state.z.steps==carbinet_state.z.change_state_steps.acc2avg){
			if(carbinet_state.z.movement_class==normal){
				carbinet_state.z.step_motor_stage = avg;
			}else {
				carbinet_state.z.step_motor_stage = disacc;
			}
		}else	if(carbinet_state.z.steps==carbinet_state.z.change_state_steps.avg2dac&&carbinet_state.z.movement_class==normal){
			carbinet_state.z.step_motor_stage = disacc;
		}else if(carbinet_state.z.step_motor_stage==disacc&&(carbinet_state.z.steps-carbinet_state.z.change_state_steps.end_steps==0)){
				carbinet_state.z.step_motor_stage = stop;
				carbinet_state.z.movement_class = stop;
				TIM_Cmd(TIM3,DISABLE);
				carbinet_state.z.pos = carbinet_exp_state.z_pos;
				carbinet_state.z.Working_or_not=0;
				carbinet_state.z.TIM_FREQ = 0;
		}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
	}
	OSIntExit();
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ITConfig( TIM2,TIM_IT_Update ,ENABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����	
}


void TIM2_IRQHandler(void)   //TIM2�ж�
{
	OSIntEnter();
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
//////////////////////////////Z axis/////////////////////////////////////////////////	
		switch(carbinet_state.z.step_motor_stage){
			case acclerate: 
					carbinet_state.z.TIM_FREQ+=Jerk;
				break;
			case avg:		
					carbinet_state.z.TIM_FREQ = avg_freq;
				break;
			case disacc:   
				if(carbinet_state.z.TIM_FREQ>0){
					carbinet_state.z.TIM_FREQ-=deacc_Jerk;
				}
				break;
		}
		if(carbinet_state.z.movement_class!=stop)
			TIM3->ARR = Calculate_freq(carbinet_state.z.TIM_FREQ);
//////////////////////////////Z axis end/////////////////////////////////////////////

//////////////////////////////theta axis/////////////////////////////////////////////////	
		switch(carbinet_state.theta.step_motor_stage){
			case acclerate: 
					carbinet_state.theta.TIM_FREQ+=theta_Jerk;
				break;
			case avg:		
					carbinet_state.theta.TIM_FREQ = theta_avg_freq;
				break;
			case disacc:   
				if(carbinet_state.theta.TIM_FREQ>0){
					carbinet_state.theta.TIM_FREQ-=theta_deacc_Jerk;
				}
				break;
		}
		if(carbinet_state.theta.movement_class!=stop)
			TIM10->ARR = Calculate_freq(carbinet_state.theta.TIM_FREQ);
//////////////////////////////theta axis end/////////////////////////////////////////////
		
//////////////////////////////x axis/////////////////////////////////////////////////	
		switch(carbinet_state.x.step_motor_stage){
			case acclerate: 
					carbinet_state.x.TIM_FREQ+=x_Jerk;
				break;
			case avg:		
					carbinet_state.x.TIM_FREQ = x_avg_freq;
				break;
			case disacc:   
				if(carbinet_state.x.TIM_FREQ>0){
					carbinet_state.x.TIM_FREQ-=x_deacc_Jerk;
				}
				break;
		}
		if(carbinet_state.x.movement_class!=stop)
			TIM1->ARR = Calculate_freq(carbinet_state.x.TIM_FREQ);
//////////////////////////////x axis end/////////////////////////////////////////////
		
	}
	OSIntExit();
}

void TIM1_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_Cmd(TIM1, DISABLE);  //ʹ��TIMx����				 
	TIM_ITConfig( TIM1,TIM_IT_Update ,ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	
}

float end_time;
void TIM1_UP_TIM10_IRQHandler(void)
{
	OSIntEnter();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		Step2_Pulse();
		if((GPIOB->ODR&0x0003)==0x0001){
			carbinet_state.x.steps++;
		}
		else if((GPIOB->ODR&0x0003)==0x0003){
			carbinet_state.x.steps--;
		}
		
		if(carbinet_state.x.steps==carbinet_state.x.change_state_steps.acc2avg){
			if(carbinet_state.x.movement_class==normal){
				carbinet_state.x.step_motor_stage = avg;
			}else {
				carbinet_state.x.step_motor_stage = disacc;
			}
		}else	if(carbinet_state.x.steps==carbinet_state.x.change_state_steps.avg2dac&&carbinet_state.x.movement_class==normal){
			carbinet_state.x.step_motor_stage = disacc;
		}else if(carbinet_state.x.step_motor_stage==disacc&&(carbinet_state.x.steps-carbinet_state.x.change_state_steps.end_steps==0)){
				carbinet_state.x.step_motor_stage = stop;
				carbinet_state.x.movement_class = stop;
				TIM_Cmd(TIM1,DISABLE);
				carbinet_state.x.pos = carbinet_exp_state.x_pos;
				carbinet_state.x.TIM_FREQ = 0;
				carbinet_state.x.Working_or_not=0;
		}
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	}
	else if (TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		Step3_Pulse();
		if((GPIOC->ODR&0x0020)==0x0020){
			carbinet_state.theta.steps++;
		}
		else if((GPIOC->ODR&0x0020)==0x0000){
			carbinet_state.theta.steps--;
		}
		
		if(carbinet_state.theta.steps==carbinet_state.theta.change_state_steps.acc2avg){
			if(carbinet_state.theta.movement_class==normal){
				carbinet_state.theta.step_motor_stage = avg;
			}else {
				carbinet_state.theta.step_motor_stage = disacc;
			}
		}else	if(carbinet_state.theta.steps==carbinet_state.theta.change_state_steps.avg2dac&&carbinet_state.theta.movement_class==normal){
			carbinet_state.theta.step_motor_stage = disacc;
		}else if(carbinet_state.theta.step_motor_stage==disacc&&(carbinet_state.theta.steps-carbinet_state.theta.change_state_steps.end_steps==0)){
				carbinet_state.theta.step_motor_stage = stop;
				carbinet_state.theta.movement_class = stop;
				TIM_Cmd(TIM10,DISABLE);
				carbinet_state.theta.pos = carbinet_exp_state.theta_pos;
				carbinet_state.theta.TIM_FREQ = 0;
				carbinet_state.theta.Working_or_not=0;
		}
		TIM_ClearFlag(TIM10,TIM_FLAG_Update);
	}
	OSIntExit();
}


void TIM10_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_Cmd(TIM10, DISABLE);  //ʹ��TIMx����				
	
	TIM_ITConfig( TIM10,TIM_IT_Update ,ENABLE);
	TIM_ClearITPendingBit(TIM10, TIM_IT_Update  );
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	 
}

// needed to change the parameter,because the crystal is not 72Mhz but 168Mhz 
// inverse of the timer calculation
// equation : 2*freq*21/84000000
u16 Calculate_freq(u16 frequency)
{
	float temp;
	temp = 2000000/frequency;
	return (u16)temp-1;
}

void step_motor_go(u16 where){
	
}
// specific function
void Init_STM_TIM(void){
	TIM3_Int_Init(1000-1,21-1);
	TIM2_Int_Init(1000-1,42-1);
	TIM1_Int_Init(1000-1,42-1);
	TIM10_Int_Init(1000-1,42-1);
}



