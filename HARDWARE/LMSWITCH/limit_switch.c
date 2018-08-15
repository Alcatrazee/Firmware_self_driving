#include "sys.h"
#include "limit_switch.h"
#include "common_fcn.h"
#include "stepmotor.h"
#include "includes.h"

void limit_switch_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |  GPIO_Pin_1|GPIO_Pin_2 |  GPIO_Pin_3| GPIO_Pin_4 |  GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);//PA0 连接到中断线0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
	
  /* 配置EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断0
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断0
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断0
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断0
  NVIC_Init(&NVIC_InitStructure);//配置
}

extern Carbinet_State carbinet_state; 
extern Carbinet_Exp_State carbinet_exp_state;

static OS_ERR err;

void limit_switch_param_Init(void){
	carbinet_state.LSD.arm_full = 1;
	carbinet_state.LSD.arm_zero = 1;
	if(PDin(2))
		carbinet_state.LSD.theta_full = 1;
	else
		carbinet_state.LSD.theta_full = 0;
	if(PDin(1))
		carbinet_state.LSD.theta_zero = 1;
	else
		carbinet_state.LSD.theta_zero = 0;
	carbinet_state.LSD.wrist_full = 1;
	carbinet_state.LSD.wrist_zero = 1;
	if(PDin(0))
		carbinet_state.LSD.x = 1;
	else
		carbinet_state.LSD.x = 0;
	carbinet_state.LSD.z = 1;
}

u8 delay_time = 20;

void EXTI0_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){
		if(!PDin(0)){
			carbinet_state.LSD.x = 0;
		}else{
			carbinet_state.LSD.x = 1;
		}
		EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE6上的中断标志位 
	}
	OSIntExit();
}

void EXTI1_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET){
		OSTimeDlyHMSM(0,0,0,delay_time,OS_OPT_TIME_HMSM_STRICT,&err);
		if(!PDin(1)){
			carbinet_state.LSD.theta_zero = 0;
		}else{
			carbinet_state.LSD.theta_zero = 1;
		}
		EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE6上的中断标志位 
	}
	OSIntExit();
}

void EXTI2_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET){
		OSTimeDlyHMSM(0,0,0,delay_time,OS_OPT_TIME_HMSM_STRICT,&err);
		if(!PDin(2)){
			carbinet_state.LSD.theta_full = 0;
		}else{
			carbinet_state.LSD.theta_full = 1;
		}
		EXTI_ClearITPendingBit(EXTI_Line2); //清除LINE6上的中断标志位 
	}
	OSIntExit();
}

void EXTI3_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET){
		OSTimeDlyHMSM(0,0,0,delay_time,OS_OPT_TIME_HMSM_STRICT,&err);
		if(!PDin(3)){
			carbinet_state.LSD.arm_zero = 0;
		}else{
			carbinet_state.LSD.arm_zero = 1;
		}
		EXTI_ClearITPendingBit(EXTI_Line3); //清除LINE6上的中断标志位 
	}
	OSIntExit();
}

void EXTI4_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET){
		if(!PDin(4)){
			carbinet_state.LSD.arm_full = 0;
		}else{
			carbinet_state.LSD.arm_full = 1;
		}
		EXTI_ClearITPendingBit(EXTI_Line4); //清除LINE6上的中断标志位 
	}
	OSIntExit();
}
















