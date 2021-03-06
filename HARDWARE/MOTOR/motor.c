#include "motor.h"
#include "led.h"
#include "usart.h"
#include "includes.h"

double counter_L=0,counter_R=0;
double arr_L=1,arr_R=1;

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void Motor_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM5时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM5
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PF9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //DIR1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PF9
	
}

#define T 0.0000025			//period /s

//定时器3中断服务函数  T=10us
void TIM5_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{
		counter_L+=0.000005f;
		counter_R+=0.000005f;
		if(counter_L>=arr_L){
			counter_L=0;
			PAout(1)=!PAout(1);
		}
		if(counter_R>=arr_R){
			counter_R=0;
			PAout(0)=!PAout(0);
		}
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
	OSIntExit();
}

float prescaler_calculator(u32 frequency){
		float out=1/(double)(frequency)/2;
		if(frequency==0)
			out=1;
		return out;
}














