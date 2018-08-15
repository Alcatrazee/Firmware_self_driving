#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED端口定义
#define LED0 PBout(9)	// DS0

void LED_Init(void);//初始化		 		
void led_turn(void);
void led_on(void);
void led_off(void);
#endif
