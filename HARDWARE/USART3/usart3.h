#ifndef __USART3__H__
#define __USART3__H__
#include "sys.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "includes.h"
#include "common_fcn.h"

#define length_of_buff 12
typedef struct 
{
	u8 Hour;
	u8 Minute;
	u8 Second;
	char latitude[length_of_buff];							//γ��				��λ:��
	char NorS;										//�ϱ���
	char longittude[length_of_buff];						//����				��λ:��
	char EorW;										//������
	u8 locate_quality;					//��λ����		0:��Ч  1:��׼��λ	2:��ֶ�λ  6:����
	u8 num_of_satellite;				//
	char accurate_horizontal[5];	//ˮƽ��ȷ��	0.5-99.9
	char altitute[7];							//�����뺣ƽ��ĸ߶�	-9999.9~9999.9M
	short sum;									//У���
}GPGGA_Data;

typedef struct{
	u8 Mode;										//ģʽ2:		M=�ֶ�   A=�Զ�
	u8 locate_mode;							//ģʽ1:		1=δ��λ	2=��ά��λ  3=��ά��λ
	u8 code1;										//��һ�ŵ�ʹ�õ�����PRN����
	u8 code2;
	u8 code3;
	u8 code4;
	u8 code5;
	u8 code6;
	u8 code7;
	u8 code8;
	u8 code9;
	u8 code10;
	u8 code11;
	u8 code12;									//��ʮ���ŵ�ʹ�õ�����PRN����
	char PDOP_accuracy[5];				//�ۺ�λ�þ�������
	char HDOP_accuracy[5];				//ˮƽ��������
	char VDOP_accuracy[5];				//��ֱ��������
	short sum;
}GPGSA_Data;

typedef struct{
	u8 Hour;
	u8 Minute;
	u8 Second;
	u8 locate_state;
	char latitude[length_of_buff];							//γ��				��λ:��
	u8 NorS;										//�ϱ���
	char longittude[length_of_buff];						//����				��λ:��
	u8 EorW;										//������
	char ground_speed[8];						//����				0~999.9��
	char ground_dir[7];							//���溽��		0~359.9
	u8  day;										//UTC	date
	u8  month;
	u16 year;
	float mag_var;							//��ƫ��
	u8 Declination;							//��ƫ�Ƿ���	
	u8 Mode_ini;								//ģʽָʾ		A=������λ  D=���	E=����   N=invalid data
	short sum;									//У���
}GPRMC_Data;

typedef struct{								//course over ground and ground speed
	float dir_TrueN;						//�汱Ϊ��׼�ĵ��溽��
	u8 Tr;											//��
	float	dir_MagN;							//�ű�Ϊ��׼�ĵ��溽��
	u8 Magni;										//M,��ʾ�ų�
	float ground_speed;					//����    ��λ:��
	u8 unit_N;									
	float ground_speed_kmph;		//����		��λ:km/h
	u8 unit_kph;								
	u8 locate_mode;							//��λģʽ:		A=������λ  D=���	E=����   N=invalid data
	short sum;
}GPVTG_Data;



void USART3_Init(u32 bouderrate);
void USART3_Process(void);
void USART3_Clear_Buff(void);
void process_GPGGA(void);
void process_GPGSA(void);
void process_GPRMC(void);
void process_GPVTG(void);
void process_GPGSV(void);
void process_GPGLL(void);
void process_GPZDA(void);

#endif
