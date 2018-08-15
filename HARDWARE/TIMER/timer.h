#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void TIM3_Int_Init(u16 arr,u16 psc); 
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM10_Int_Init(u16 arr,u16 psc);
void TIM1_Int_Init(u16 arr,u16 psc);
u16 Calculate_freq(u16 frequency);
void Init_STM_TIM(void);
 
#endif
