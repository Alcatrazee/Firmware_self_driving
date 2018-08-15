#ifndef __USART3__H__
#define __USART3__H__
#include "sys.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"


typedef struct
{
	short plt_X;							//ƽ̨Xλ��  ��λ������
	short plt_Y;							//ƽ̨Yλ��  ��λ������
	int   frame_X;						//����Xλ��	 ��λ������
	int 	frame_Y;						//����Yλ��	 ��λ������	
	float frame_Vx;
	float frame_Vy;
	float omega;
	float angle;							//����Ƕ�	 ��λ����								
	u8    hit_state;					//����״̬λ 		 1:�ѻ���		0:δ����
}RB_State;

typedef struct 
{
	u8 Hour;
	u8 Minute;
	float Second;
	double latitude;							//γ��				��λ:��
	u8 NorS;										//�ϱ���
	double longittude;						//����				��λ:��
	u8 EorW;										//������
	u8 locate_quality;					//��λ����		0:��Ч  1:��׼��λ	2:��ֶ�λ  6:����
	u8 num_of_satellite;				//
	float accurate_horizontal;	//ˮƽ��ȷ��	0.5-99.9
	float altitute;							//�����뺣ƽ��ĸ߶�	-9999.9~9999.9M
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
	float PDOP_accuracy;				//�ۺ�λ�þ�������
	float HDOP_accuracy;				//ˮƽ��������
	float VDOP_accuracy;				//��ֱ��������
	short sum;
}GPGSA_Data;

typedef struct{
	u8 Hour;
	u8 Minute;
	float Second;
	u8 locate_state;
	double latitude;							//γ��				��λ:��
	u8 NorS;										//�ϱ���
	double longittude;						//����				��λ:��
	u8 EorW;										//������
	float ground_speed;						//����				0~999.9��
	float ground_dir;							//���溽��		0~359.9
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
u8 strcmp_real(char str1[3],char str2[3]);

#endif
