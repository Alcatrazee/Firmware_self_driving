#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"

void Motor_Init(u16 arr,u16 psc);
float prescaler_calculator(u32 frequency);

extern u32 frequency_motor_L,frequency_motor_R;

#define ENAL PAout(3)
#define ENAR PAout(2)
#define DIRL PAout(4)
#define DIRR PCout(2)

#endif
