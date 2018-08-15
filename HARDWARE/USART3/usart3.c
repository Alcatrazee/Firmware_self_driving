#include "usart3.h"
#include "sys.h"
#include "usart.h"
#include "stdlib.h"
#include "includes.h"
#include "string.h"

GPGGA_Data GPGGA_Dat={0,0,0,0,0,0,0,0,0,0,0,0};
GPGSA_Data GPGSA_Dat={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
GPRMC_Data GPRMC_Dat={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
GPVTG_Data GPVTG_Dat={0,0,0,0,0,0,0,0,0,0};

void USART3_Init(u32 bouderrate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bouderrate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); 
	
  USART_Cmd(USART3, ENABLE);  

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
}

#define usart3_buff_size 200
u8 Res[usart3_buff_size];
u8 counter=0;
	
void USART3_IRQHandler(void)                	
{
	OSIntEnter();
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		Res[counter] = USART3->DR;
		//printf("%c",Res[counter]);
		counter++;
		if(Res[0]!='$')
			USART3_Clear_Buff();
		else if(Res[counter-1]=='\n'&&Res[counter-2]=='\r'){
			USART3_Process();
			USART3_Clear_Buff();
		}
	}
	OSIntExit();
}

void USART3_Process(void){
	char command[3]={0};
	u8 i;
	for(i=3;i<7;i++){
		command[i-3]=Res[i];
	}
	if(strcmp_real(command,"GGA")==0){
		process_GPGGA();
	}
	if(strcmp_real(command,"GSA")==0){
		process_GPGSA();
	}
	if(strcmp_real(command,"RMC")==0){
		process_GPRMC();
	}
//	if(strcmp_real(command,"GSV")==0)
//		process_GPGSV();
//	if(strcmp_real(command,"VTG")==0)
//		process_GPVTG();
//	if(strcmp_real(command,"GLL")==0)
//		process_GPGLL();
//	if(strcmp_real(command,"ZDA")==0)
//		process_GPZDA();
}

/////////////////////////////////////////////////////////////////////////////////working on///////////////////////////////////////////////////////////////////////////////////////////////////
void process_GPGGA(void){
	u8 pos_of_dot[20]={0};
	u8 i=0,j=0,k=0;
	u8 count=0;
	char temp_arr[length_of_buff]={0};
	float temp=0;
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
	for(j=0;j<count-1;j++){
		for(i=pos_of_dot[j]+1;i<pos_of_dot[j+1];i++){
			temp_arr[k] = Res[i];
			k++;
		}
		//set to the structure
		switch(j){
			case 0:	temp = gps_atof(temp_arr);	GPGGA_Dat.Hour = (u32)temp/10000;	GPGGA_Dat.Minute = ((u32)temp-10000*GPGGA_Dat.Hour)/100; GPGGA_Dat.Second = temp-10000*GPGGA_Dat.Hour-100*GPGGA_Dat.Minute;	GPGGA_Dat.Hour+=8;		break;
			case 1:	strcpy(GPGGA_Dat.latitude,temp_arr);    break;
			case 2: GPGGA_Dat.NorS = temp_arr[0];										break;
			case 3: strcpy(GPGGA_Dat.longittude,temp_arr);		break;
			case 4: GPGGA_Dat.EorW = temp_arr[0];										break;
			case 5:	GPGGA_Dat.locate_quality = temp_arr[0];					break;
			case 6:	GPGGA_Dat.num_of_satellite = (u8)atoi(temp_arr);break;
			case 7: strcpy(GPGGA_Dat.accurate_horizontal,temp_arr); break;
			case 8:	strcpy(GPGGA_Dat.altitute,temp_arr);						break; 
		}
		for(;k>0;k--){
			temp_arr[k] = 0;
		}
	}
 }
 
 
void process_GPGSA(void){
	u8 pos_of_dot[20]={0};
	u8 i=0,j=0,k=0;
	u8 count=0;
	char temp_arr[20]={0};
	float temp=0;
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
		for(j=0;j<count;j++){
			if(j<16){
				for(i=pos_of_dot[j]+1;i<pos_of_dot[j+1];i++){
					temp_arr[k] = Res[i];
					k++;
				}
			}else if(j==16){
				for(i=pos_of_dot[16]+1;i<pos_of_dot[16]+5;i++){
					temp_arr[k] = Res[i];
					k++;		
				}
			}
		//set to the structure
		switch(j){
			case 0:	temp = atoi(temp_arr);	GPGSA_Dat.Mode = temp;			break;
			case 1:	GPGSA_Dat.locate_mode = atoi(temp_arr);							break;
			case 2: GPGSA_Dat.code1 = atoi(temp_arr);										break;
			case 3: GPGSA_Dat.code2 = atoi(temp_arr);          					break;
			case 4: GPGSA_Dat.code3 = atoi(temp_arr);          					break;
			case 5: GPGSA_Dat.code4 = atoi(temp_arr);          					break;
			case 6: GPGSA_Dat.code5 = atoi(temp_arr);          					break;
			case 7: GPGSA_Dat.code6 = atoi(temp_arr);          					break;
			case 8: GPGSA_Dat.code7 = atoi(temp_arr);          					break;
			case 9: GPGSA_Dat.code8 = atoi(temp_arr);          					break;
			case 10: GPGSA_Dat.code9 = atoi(temp_arr);          				break;
			case 11: GPGSA_Dat.code10 = atoi(temp_arr);          				break;
			case 12: GPGSA_Dat.code11 = atoi(temp_arr);          				break;
			case 13: GPGSA_Dat.code12 = atoi(temp_arr);           			break;
			case 14: strcpy(GPGSA_Dat.PDOP_accuracy,temp_arr); 					break;
			case 15: strcpy(GPGSA_Dat.HDOP_accuracy,temp_arr); 					break;
			case 16: strcpy(GPGSA_Dat.VDOP_accuracy,temp_arr); 					break;
//			case 17: GPGSA_Dat.sum = 				break;
		}
		for(;k>0;k--){
			temp_arr[k] = 0;
		}
		temp_arr[0]=0;
	}
}




void process_GPRMC(void){
	u8 pos_of_dot[20]={0};
	u8 i=0,j=0,k=0;
	u8 count=0;
	char temp_arr[20]={0};
	float temp=0;
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
	for(j=0;j<count-1;j++){
			for(i=pos_of_dot[j]+1;i<pos_of_dot[j+1];i++){
				temp_arr[k] = Res[i];
				k++;
			}
		
		//set to the structure
			switch(j){
				case 0:																																			
								temp = atoi(temp_arr);	
								GPRMC_Dat.Hour = temp/10000;	
								GPRMC_Dat.Minute = (temp-10000*GPRMC_Dat.Hour)/100; 
								GPRMC_Dat.Second = temp-10000*GPRMC_Dat.Hour-100*GPRMC_Dat.Minute;	
								GPRMC_Dat.Hour+=8;		
								break;
				case 1:	
								GPRMC_Dat.locate_state = temp_arr[0];						
								break;
				case 2:																																								// have trouble with hardfault handler
								strcpy(GPRMC_Dat.latitude,temp_arr);		
								break;
				case 3: 																														
								GPRMC_Dat.NorS = temp_arr[0];			
								break;
				case 4: 																																							// have trouble with hardfault handler
								strcpy(GPRMC_Dat.longittude,temp_arr);			
								break;
				case 5: 
								GPRMC_Dat.EorW = temp_arr[0];					
								break;
				case 6:	
								strcpy(GPRMC_Dat.ground_speed,temp_arr);					
								break;
				case 7:	
								strcpy(GPRMC_Dat.ground_dir,temp_arr); 		
								break;
				case 8:	
								temp = atoi(temp_arr);	
								GPRMC_Dat.day = (u32)temp/10000;	
								GPRMC_Dat.month = ((u32)temp-10000*GPRMC_Dat.day)/100; 
								GPRMC_Dat.year = temp-10000*GPRMC_Dat.day-100*GPRMC_Dat.month;	
								break;
				case 9:	
								GPRMC_Dat.mag_var = gps_atof(temp_arr);																					// have trouble with hardfault handler	
								break;
				case 10:	
								GPRMC_Dat.Declination = temp_arr[0];					
								break;
				case 11:
								GPRMC_Dat.Mode_ini = atoi(temp_arr);					
								break;
	//			case 12:GPRMC_Dat.sum = 
			}
			for(;k>0;k--){
				temp_arr[k] = 0;
			}temp_arr[0] = 0;
	}
}




void process_GPVTG(void){
	u8 pos_of_dot[20]={0};
	u8 i=0,j=0,k=0;
	u8 count=0;
	char temp_arr[20]={0};
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
	for(j=0;j<count;j++){
		if(j<count-1){
			for(i=pos_of_dot[j]+1;i<pos_of_dot[j+1];i++){
				temp_arr[k] = Res[i];
				k++;
			}
		}else if(j==(count-1)){
				i=pos_of_dot[j]+1;
				temp_arr[k] = Res[i];
				k++;
		}
		//set to the structure
		switch(j){
			case 0:	GPVTG_Dat.dir_TrueN = atof(temp_arr);						break;
			case 1:	GPVTG_Dat.Tr = temp_arr[0];											break;
			case 2:	GPVTG_Dat.dir_MagN = atof(temp_arr);						break;
			case 3: GPVTG_Dat.Magni = temp_arr[0];									break;
			case 4: GPVTG_Dat.ground_speed = atof(temp_arr);				break;
			case 5: GPVTG_Dat.unit_N = temp_arr[0];									break;
			case 6:	GPVTG_Dat.ground_speed_kmph = atof(temp_arr);		break;
			case 7:	GPVTG_Dat.unit_kph = temp_arr[0];								break;
			case 8:	GPVTG_Dat.locate_mode = temp_arr[0];						break;
//			case 12:GPVTG_Dat.sum = 
		}
		for(;k>0;k--){
			temp_arr[k] = 0;
		}
		temp_arr[0] = 0;
	}
}

//void process_GPGSV(void){
//	u8 pos_of_dot[20]={0};
//	u8 i;
//	u8 count=0;
//	for(i=0;i<counter+1;i++){
//		if(Res[i]==','){
//			pos_of_dot[count]=i;
//			count++3
;
//		}
//	}
//}
/*
void process_GPGLL(void){
	u8 pos_of_dot[20]={0};
	u8 i;
	u8 count=0;
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
	//printf("longtitude:%10.6f%c\tlatitude:%10.6f%c\ttime:%2d:%2d:%2f\r\n",GPRMC_Dat.longittude,GPRMC_Dat.EorW,GPRMC_Dat.latitude,GPRMC_Dat.NorS,GPRMC_Dat.Hour,GPRMC_Dat.Minute,GPRMC_Dat.Second);

}

void process_GPZDA(void){
	u8 pos_of_dot[20]={0};
	u8 i;
	u8 count=0;
	for(i=0;i<counter+1;i++){
		if(Res[i]==','){
			pos_of_dot[count]=i;
			count++;
		}
	}
}
*/

//////////////////////////////////////////////////////////
//function name:	USART3_Clear_Buff											//
//function		 :	clear rec buff												//
//input Para	 :	None																	//
//output       :  None																	//
//////////////////////////////////////////////////////////
void USART3_Clear_Buff(void){
	u16 i;
	for(i=0;i<counter;i++){
		Res[i]=0;
	}
	counter=0;
}

//////////////////////////////////////////////////////////
//function name:	UART3_Put_Char												//
//function		 :	Send a letter 												//
//input Para	 :	None																	//
//output       :  None																	//
//////////////////////////////////////////////////////////
 void UART3_Put_Char(unsigned char DataToSend)
{
	while((USART3->SR&0x40)==0);  
    USART3->DR = (u8) DataToSend;  
}





































