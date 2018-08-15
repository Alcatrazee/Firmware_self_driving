#ifndef __TIME_H__
#define __TIME_H__

#include "sys.h"


void SYSTICK_INIT(void);

uint32_t GET_NOWTIME(uint32_t * lasttime);

void mget_ms(unsigned long  *time);

#endif

