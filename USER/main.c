#include "common_fcn.h"
#include "includes.h"
#include "ctrl.h"
#include "mpu9250.h"
#include "exti.h"
#include "timer.h"
#include "usart2.h"
#include "icapture.h"
#include "usart3.h"
#include "adc.h"
#include "relay.h"
#include "motor.h"
#include "adc.h"
#include "icapture.h"
#include "pwm.h"
#include "stepmotor.h"
#include "limit_switch.h"


#define START_TASK_PRIO		3
#define START_STK_SIZE 		128
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

// task1 is led task
#define LED_TASK_PRIO		6
#define LED_TASK_STK_SIZE 128
OS_TCB LED_TASK_TaskTCB;
CPU_STK LED_TASK_STK[LED_TASK_STK_SIZE];
void Led_task(void *p_arg);

// task2 is print task
#define PRINT_TASK_PRIO	6
#define PRINT_TASK_STK_SIZE 		256
OS_TCB Print_Task_TaskTCB;
CPU_STK PRINT_TASK_STK[PRINT_TASK_STK_SIZE];
void Print_task(void *p_arg);

#define CTRL_TASK_PRIO		5
#define CTRL_STK_SIZE 		256
OS_TCB Ctrl_TaskTCB;
CPU_STK CTRL_TASK_STK[CTRL_STK_SIZE];
void CTRL_task(void *p_arg);

#define IPCAP_TASK_PRIO		6
#define IPCAP_STK_SIZE 		128
OS_TCB IPCAP_TaskTCB;
CPU_STK IPCAP_TASK_STK[IPCAP_STK_SIZE];
void InputCapture_task(void *p_arg);

#define CABINET_CTRL_TASK_PRIO		5
#define CABINET_CTRL_STK_SIZE 		128
OS_TCB Cabinet_Ctrl_TaskTCB;
CPU_STK CABINET_CTRL_TASK_STK[CABINET_CTRL_STK_SIZE];
void CABINET_CTRL_task(void *p_arg);

#define GPS_UPDATE_TASK_PRIO		6
#define GPS_UPDATE_STK_SIZE 		128
OS_TCB GPS_UPDATE_TaskTCB;
CPU_STK GPS_UPDATE_TASK_STK[GPS_UPDATE_STK_SIZE];
void gps_update_task(void *p_arg);

void BSP_Init(void);
void Print_GPS_Data(void);
void Print_IMU_Data(void);
void Crack_float(float fp,char int_part[7],char dec_part[3]);
void Clear_arr(char int_part[7],char dec_part[3]);
void cabinet_mechanical_init(void);
void cabinet_param_init(void);

// from frame control
extern RB_State State;
extern float Yaw;
extern u8 USART2_RX_DAT[11];
extern int vol_wL,vol_wR;

extern u8 GPGGA_Process_finish_flag;
extern u8 GPRMC_Process_finish_flag;
extern u8 GPGSA_Process_finish_flag;


extern int step_counter_L,step_counter_R;
// from cabinet control

extern GPGGA_Data GPGGA_Dat;
extern GPGSA_Data GPGSA_Dat;
extern GPRMC_Data GPRMC_Dat;
extern GPVTG_Data GPVTG_Dat;
Carbinet_State carbinet_state; 
Carbinet_Exp_State carbinet_exp_state;

u16 signal[4];

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	BSP_Init();
	
	OSInit(&err);		    //初始化UCOSIII
	OS_CRITICAL_ENTER();	//进入临界区			 
	
	//创建开始任务
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//任务控制块
								 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
								 
}


//开始任务任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	

	OS_CRITICAL_ENTER();	//进入临界区
	//创建led任务
	OSTaskCreate(  (OS_TCB 	* )&LED_TASK_TaskTCB,		
								 (CPU_CHAR	* )"led task", 		
                 (OS_TASK_PTR )Led_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED_TASK_PRIO,     
                 (CPU_STK   * )&LED_TASK_STK[0],	
                 (CPU_STK_SIZE)LED_TASK_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED_TASK_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//创建TASK2任务	 
	OSTaskCreate((OS_TCB 	* )&Print_Task_TaskTCB,		
				 (CPU_CHAR	* )"print task", 		
                 (OS_TASK_PTR )Print_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )PRINT_TASK_PRIO,     	
                 (CPU_STK   * )&PRINT_TASK_STK[0],	
                 (CPU_STK_SIZE)PRINT_TASK_STK_SIZE/10,	
                 (CPU_STK_SIZE)PRINT_TASK_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			 
	
	OSTaskCreate((OS_TCB 	* )&Ctrl_TaskTCB,		
				 (CPU_CHAR	* )"control task", 		
                 (OS_TASK_PTR )CTRL_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CTRL_TASK_PRIO,     	
                 (CPU_STK   * )&CTRL_TASK_STK[0],	
                 (CPU_STK_SIZE)CTRL_STK_SIZE/10,	
                 (CPU_STK_SIZE)CTRL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
								 
	OSTaskCreate((OS_TCB 	* )&IPCAP_TaskTCB,		
			 (CPU_CHAR	* )"input capture task", 		
							 (OS_TASK_PTR )InputCapture_task, 			
							 (void		* )0,					
							 (OS_PRIO	  )IPCAP_TASK_PRIO,     	
							 (CPU_STK   * )&IPCAP_TASK_STK[0],	
							 (CPU_STK_SIZE)IPCAP_STK_SIZE/10,	
							 (CPU_STK_SIZE)IPCAP_STK_SIZE,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	  )0,					
							 (void   	* )0,				
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
							 (OS_ERR 	* )&err);		
							 				 
	OSTaskCreate((OS_TCB 	* )&Cabinet_Ctrl_TaskTCB,		
			 (CPU_CHAR	* )"cabinet control task", 		
							 (OS_TASK_PTR )CABINET_CTRL_task, 			
							 (void		* )0,					
							 (OS_PRIO	  )CABINET_CTRL_TASK_PRIO,     	
							 (CPU_STK   * )&CABINET_CTRL_TASK_STK[0],	
							 (CPU_STK_SIZE)CABINET_CTRL_STK_SIZE/10,	
							 (CPU_STK_SIZE)CABINET_CTRL_STK_SIZE,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	  )0,					
							 (void   	* )0,				
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
							 (OS_ERR 	* )&err);		
							 
	OSTaskCreate((OS_TCB 	* )&GPS_UPDATE_TaskTCB,		
			 (CPU_CHAR	* )"gps data update task", 		
							 (OS_TASK_PTR )gps_update_task, 			
							 (void		* )0,					
							 (OS_PRIO	  )GPS_UPDATE_TASK_PRIO,     	
							 (CPU_STK   * )&GPS_UPDATE_TASK_STK[0],	
							 (CPU_STK_SIZE)GPS_UPDATE_STK_SIZE/10,	
							 (CPU_STK_SIZE)GPS_UPDATE_STK_SIZE,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	  )0,					
							 (void   	* )0,				
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
							 (OS_ERR 	* )&err);						 
							 
							 
							 
	OSTaskSuspend(&Cabinet_Ctrl_TaskTCB,&err);		 
	//OSTaskSuspend(&Ctrl_TaskTCB,&err);		 
	OSTaskSuspend(&GPS_UPDATE_TaskTCB,&err);		 
	//OSTaskSuspend(&Print_Task_TaskTCB,&err);							 
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}



void Led_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		led_turn();
		//Get_Adc_Val();
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void Print_task(void *p_arg)
{
	CPU_SR_ALLOC();
	OS_ERR err;
	while(1)
	{
		OS_CRITICAL_ENTER();
		//printf("steps_x:%d\tsteps_z:%d\tsteps_theta:%d\r\n",carbinet_state.x.steps,carbinet_state.z.steps,carbinet_state.theta.steps);
		//Print_IMU_Data();
		printf("%d\t%d\r\n",step_counter_L,step_counter_R);
		OS_CRITICAL_EXIT();
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void CTRL_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		Controller(signal);
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void InputCapture_task(void *p_arg){
	OS_ERR err;
	while(1){
		input_capture(signal);
		//printf("%d\t%d\t%d\t%d\r\n",signal[0],signal[1],signal[2],signal[3]);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void CABINET_CTRL_task(void *p_arg){
	extern u8 movement_class;
	extern Change_steps change_state_steps;
	OS_ERR err;
	while(1)
	{
		if((!carbinet_state.z.Working_or_not)&&(carbinet_state.z.movement_class==stop)){
			carbinet_state.z.Working_or_not = 1;
			process_input(carbinet_z);
		}
		if((!carbinet_state.x.Working_or_not)&&(carbinet_state.x.movement_class==stop)){
			carbinet_state.x.Working_or_not = 1;
			process_input(carbinet_x);
		}
		if((!carbinet_state.theta.Working_or_not)&&(carbinet_state.theta.movement_class==stop)){
			carbinet_state.theta.Working_or_not = 1;
			process_input(carbinet_theta);
		}
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void gps_update_task(void *p_arg){
	CPU_SR_ALLOC();
	OS_ERR err;
	while(1)
	{
		OS_CRITICAL_ENTER();
		Print_GPS_Data();
		OS_CRITICAL_EXIT();
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void BSP_Init(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2
	delay_init(168);  															//初始化延时函数	
	LED_Init();																			//初始化LED 
	uart_init(115200);															//初始化串口波特率为460800
//	USART3_Init(9600);															//gps init
	Adc_Init();         														//初始化ADC
	relay_init();
	printf("initializing mpu9250...\r\n");
	while(Init_MPU9250()==MPU9250_FAIL){						//初始化MPU9250
		led_turn();
		delay_ms(100);
	}
	printf("mpu9250 initialization completed...\r\n");
	Motor_Init(10-1,42-1);
  TIM4_Cap_Init(0XFFFF,84-1);		
	EXTIX_Init();

	printf("starting to calculate offset,don't touch the viechle...\r\n");
	offset_cut();
	printf("offset get...\r\n");
	ENAL=1;
	ENAR=1;

	
//	Init_STM_TIM();
//	Step_motor_init();
//	limit_switch_Init();
//	limit_switch_param_Init();
//	cabinet_mechanical_init();
//	cabinet_param_init();
}

void cabinet_mechanical_init(void){
	// initialize the stm of x axis
	TIM10->ARR = Calculate_freq(600);
	PCout(5) = 0;
	TIM_Cmd(TIM10,ENABLE);
	while(carbinet_state.LSD.theta_zero);
//	PCout(5) = 1;
//	while(!carbinet_state.LSD.theta_zero);
	TIM_Cmd(TIM10,DISABLE);
	
	TIM1->ARR = Calculate_freq(1500);
	PBout(1) = 1;
	TIM_Cmd(TIM1,ENABLE);
	while(PDin(0));
	PBout(1) = 0;
	while(!PDin(0));
	TIM_Cmd(TIM1,DISABLE);
}

void cabinet_param_init(void){
	TIM_Cmd(TIM1,DISABLE);
	carbinet_state.x.pos = 0;
	carbinet_state.x.steps = 0;
	carbinet_state.x.movement_class = stop;
	carbinet_state.x.Working_or_not = 0;
	carbinet_state.x.step_motor_stage = stop;
	
	TIM_Cmd(TIM3,DISABLE);
	carbinet_state.z.pos = 0;
	carbinet_state.z.steps = 0;
	carbinet_state.z.movement_class = stop;
	carbinet_state.z.Working_or_not = 0;
	carbinet_state.z.step_motor_stage = stop;
	
	TIM_Cmd(TIM10,DISABLE);
	carbinet_state.theta.pos = 0;
	carbinet_state.theta.steps = 0;
	carbinet_state.theta.movement_class = stop;
	carbinet_state.theta.Working_or_not = 0;
	carbinet_state.theta.step_motor_stage = stop;
}

void Print_GPS_Data(void){
	UART1_Put_String_with_space("GPS");
	UART1_Put_String_with_space(GPGGA_Dat.latitude);
	UART1_Put_Char_with_space(GPGGA_Dat.NorS);
	UART1_Put_String_with_space(GPGGA_Dat.longittude);
	UART1_Put_Char_with_space(GPGGA_Dat.EorW);
	UART1_Put_String_with_space(GPRMC_Dat.ground_speed);
	UART1_Put_String_with_space(GPGGA_Dat.altitute);
	UART1_Put_String_with_space(GPGSA_Dat.PDOP_accuracy);
	UART1_Print_timestamp();
}


// meaning:first byte is the integrate part of the data ,and the second one is the decimal part 

void Print_IMU_Data(void){
	char int_part[7]={0},dec_part[3]={0};
	char vel_L[6]={0},vel_R[6]={0};
	
	sprintf(vel_L,"%d",vol_wL);
	sprintf(vel_R,"%d",vol_wR);
	
	UART1_Put_String_with_space("IMU");
	
	Crack_float(IMU_DAT.ax,int_part,dec_part);				// imu data accelaration x 
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(IMU_DAT.ay,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(IMU_DAT.az,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);

	Crack_float(IMU_DAT.gx,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(IMU_DAT.gy,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	Crack_float(IMU_DAT.gz,int_part,dec_part);
	UART1_Put_String_with_space(int_part);
	UART1_Put_String_with_space(dec_part);
	Clear_arr(int_part,dec_part);
	
	UART1_Put_String_with_space(vel_L);
	UART1_Put_String_with_space(vel_R);
	
	UART1_Print_timestamp();
}

void Crack_float(float fp,char int_part[7],char dec_part[3]){
	int dec=0;
	int int_part_var=0;
	int_part_var = (int)fp;
	dec = my_abs((fp - int_part_var)*100);
	sprintf(int_part,"%d",int_part_var);
	sprintf(dec_part,"%d",dec);
}

void Clear_arr(char int_part[7],char dec_part[3]){
	u8 i;
	for(i=0;i<7;i++)
		int_part[i]=0;
	for(i=0;i<3;i++)
		dec_part[i]=0;
}






