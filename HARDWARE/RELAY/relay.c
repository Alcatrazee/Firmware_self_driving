#include "relay.h"

void relay_init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOC,GPIO_Pin_11);//GPIOF9,F10设置高，灯灭
	GPIO_SetBits(GPIOC,GPIO_Pin_12);
}

void relay0_turn(void){
	relay0 = !relay0;
}

void relay0_on(void){
	relay0 = 0;
}

void relay0_off(void){
	relay0 = 1;
}

void relay1_turn(void){
	relay1 = !relay1;
}

void relay1_on(void){
	relay1 = 0;
}

void relay1_off(void){
	relay1 = 1;
}


