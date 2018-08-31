#include "sys.h"
#include "usart.h"	
#include <stdlib.h>
#include "includes.h"
#include "common_fcn.h"
#include "stepmotor.h"

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	
}
/*
#define bufflength 9
u8 rec[bufflength];
u8 counter_usart1=0;

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		rec[counter_usart1] =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		counter_usart1++;
		if(rec[0]=='O'||rec[0]=='V'||rec[0]=='S'||rec[0]=='P'||rec[0]=='I'||rec[0]=='D'||rec[0]=='A'){
			if((counter_usart1==bufflength)&&rec[8]=='E'){
				Process(rec[0]);
				Clear();
			}
		}else{
			Clear();
		}
  } 
}
*/

#define bufflength 30
u8 rec[bufflength]={0};
u8 counter_usart1=0;

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		rec[counter_usart1] = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		counter_usart1++;
		if(counter_usart1<=bufflength){
			if(rec[0]=='M'){
				if(rec[counter_usart1-1]=='\n'&&rec[counter_usart1-2]=='\r'){
					Process(rec[0]);
					Clear();
				}
			}else{
				Clear();
			}
		}else{
			Clear();
		}
  } 
}



 void UART1_Put_Char(unsigned char DataToSend)
{
	while((USART1->SR&0x40)==0);  
    USART1->DR = (u8) DataToSend;  
}

void  UART1_Put2_Char(u8 arr[2]){
	UART1_Put_Char(arr[0]);
	UART1_Put_Char(arr[1]);
}

void UART1_Put_String(char *p){
	while(*p!='\0'){
		UART1_Put_Char(*p);
		p++;
	}
}

void UART1_Put_String_with_space(char *p){
	while(*p!='\0'){
		UART1_Put_Char(*p);
		p++;
	}
	UART1_Put_Char(' ');
}

void UART1_Put_Char_with_space(char DataToSend){
	UART1_Put_Char(DataToSend);
	UART1_Put_Char(' ');
}

void UART1_Print_timestamp(void){
	OS_ERR err;
	u32 time = OSTimeGet(&err);
	char temp[10]={0};
	sprintf(temp,"%d",time);
	UART1_Put_String(temp);
	UART1_Put_String("\r\n");
}

void Clear(void){
	u8 i;
	for(i=0;i<bufflength;i++){
		rec[i] = 0;
	}
	counter_usart1 = 0;
}

void Process(u8 Para){
	u8 i;
	char str_z[4] = {0,0,0,0};
	char str_theta[3] = {0,0,0};
	for(i=0;i<4;i++)
		str_z[i] = rec[i+1];
	carbinet_exp_state.x_pos = atoi(str_z);
	for(i=0;i<4;i++)
		str_z[i] = rec[i+5];
	carbinet_exp_state.z_pos = atoi(str_z);
	for(i=0;i<3;i++)
		str_theta[i] = rec[i+9];
	carbinet_exp_state.theta_pos = atoi(str_theta);
}

	/*char str_v[5]={0};
	char str_o[4]={0};
	u8 i;
	float v_temp=0,o_temp=0;
	
	for(i=0;i<6;i++){
		str_v[i] = rec[i+1];
	}
	v_temp = atof(str_v);
	Exp_State.frame_Vy = v_temp;
	
	for(i=7;i<11;i++){
		str_o[i-7] = rec[i];
	}
	o_temp = my_atof(str_o);
	Exp_State.omega = o_temp;*/

/*
	format
	M00002000180
	Mxposzposomega
	
*/





