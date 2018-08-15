#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	  	
void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put2_Char(u8 arr[2]);
void uart_init(u32 bound);
void Process(u8 Para);
void Clear(void);
void UART1_Put_String(char *p);
void UART1_Put_String_with_space(char *p);
void UART1_Print_timestamp(void);
void UART1_Put_Char_with_space(char DataToSend);

#endif


