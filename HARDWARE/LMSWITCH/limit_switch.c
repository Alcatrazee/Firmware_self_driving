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

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |  GPIO_Pin_1|GPIO_Pin_2 |  GPIO_Pin_3| GPIO_Pin_4 |  GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);//PA0 ���ӵ��ж���0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
	
  /* ����EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�����ش��� 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
  EXTI_Init(&EXTI_InitStructure);//����
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//�ⲿ�ж�0
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//�ⲿ�ж�0
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�0
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//�ⲿ�ж�0
  NVIC_Init(&NVIC_InitStructure);//����
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
		EXTI_ClearITPendingBit(EXTI_Line0); //���LINE6�ϵ��жϱ�־λ 
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
		EXTI_ClearITPendingBit(EXTI_Line1); //���LINE6�ϵ��жϱ�־λ 
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
		EXTI_ClearITPendingBit(EXTI_Line2); //���LINE6�ϵ��жϱ�־λ 
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
		EXTI_ClearITPendingBit(EXTI_Line3); //���LINE6�ϵ��жϱ�־λ 
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
		EXTI_ClearITPendingBit(EXTI_Line4); //���LINE6�ϵ��жϱ�־λ 
	}
	OSIntExit();
}
















