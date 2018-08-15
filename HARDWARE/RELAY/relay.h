#ifndef __RELAY__H__
#define __RELAY__H__

#include "sys.h"

#define relay0 PCout(11)
#define relay1 PCout(12)

void relay_init(void);
void relay0_turn(void);
void relay0_on(void);
void relay0_off(void);
void relay1_turn(void);
void relay1_on(void);
void relay1_off(void);

#endif


