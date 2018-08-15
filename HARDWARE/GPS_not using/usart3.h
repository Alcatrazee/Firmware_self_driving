#ifndef __USART3__H__
#define __USART3__H__
#include "sys.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"


typedef struct
{
	short plt_X;							//平台X位置  单位：毫米
	short plt_Y;							//平台Y位置  单位：毫米
	int   frame_X;						//车体X位置	 单位：毫米
	int 	frame_Y;						//车体Y位置	 单位：毫米	
	float frame_Vx;
	float frame_Vy;
	float omega;
	float angle;							//车体角度	 单位：度								
	u8    hit_state;					//击球状态位 		 1:已击球		0:未击球
}RB_State;

typedef struct 
{
	u8 Hour;
	u8 Minute;
	float Second;
	double latitude;							//纬度				单位:°
	u8 NorS;										//南北球
	double longittude;						//经度				单位:°
	u8 EorW;										//东西球
	u8 locate_quality;					//定位质量		0:无效  1:标准定位	2:差分定位  6:估算
	u8 num_of_satellite;				//
	float accurate_horizontal;	//水平精确度	0.5-99.9
	float altitute;							//天线离海平面的高度	-9999.9~9999.9M
	short sum;									//校验和
}GPGGA_Data;

typedef struct{
	u8 Mode;										//模式2:		M=手动   A=自动
	u8 locate_mode;							//模式1:		1=未定位	2=二维定位  3=三维定位
	u8 code1;										//第一信道使用的卫星PRN码编号
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
	u8 code12;									//第十二信道使用的卫星PRN码编号
	float PDOP_accuracy;				//综合位置精度因子
	float HDOP_accuracy;				//水平精度因子
	float VDOP_accuracy;				//垂直精度因子
	short sum;
}GPGSA_Data;

typedef struct{
	u8 Hour;
	u8 Minute;
	float Second;
	u8 locate_state;
	double latitude;							//纬度				单位:°
	u8 NorS;										//南北球
	double longittude;						//经度				单位:°
	u8 EorW;										//东西球
	float ground_speed;						//地速				0~999.9节
	float ground_dir;							//地面航向		0~359.9
	u8  day;										//UTC	date
	u8  month;
	u16 year;
	float mag_var;							//磁偏角
	u8 Declination;							//磁偏角方向	
	u8 Mode_ini;								//模式指示		A=自主定位  D=差分	E=估算   N=invalid data
	short sum;									//校验和
}GPRMC_Data;

typedef struct{								//course over ground and ground speed
	float dir_TrueN;						//真北为基准的地面航向
	u8 Tr;											//真
	float	dir_MagN;							//磁北为基准的地面航向
	u8 Magni;										//M,表示磁场
	float ground_speed;					//地速    单位:节
	u8 unit_N;									
	float ground_speed_kmph;		//地速		单位:km/h
	u8 unit_kph;								
	u8 locate_mode;							//定位模式:		A=自主定位  D=差分	E=估算   N=invalid data
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
