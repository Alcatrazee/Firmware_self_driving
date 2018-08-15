v1.0.4.20180703
第三个步进电机已经成功加入，待加入转盘上的机构以及电路板使整个设备更整洁。
v1.0.3.20180627
第二个步进电机已经加入成功，待加入第三个步进电机，下一版本先加入光电开关以及各种限位开关
v1.0.2.20180623
整体程序完成，待加入第二个步进电机程序
v1.0.1.20180615
GPS信息接收程序完成，可以直接输出处理后的角度制的GPS信息，
v1.0.0.20180608
建立完整的基于ucos iii的底盘运动控制程序，模块单独工作正常，未完整调试。




外设  			引脚定义		使用定时器	
led				PB9				
exti			PE13 
				PE9
				PE11
				PE14
usart1 TX		PA9
usart1 RX       PA10
usart2 TX 		PD5	
usart2 RX		PD6
usart3 TX		PB11
usart3 RX 		PB10
IIC SCL			PB6
IIC SDA			PB7
ADC 1			PA5
ADC 2 			PA7
RELAY1 			PC11
RELAY2			PC12
MOTOR			PA0~PA4 PC2		TIM5
INPUTCAPTURE	PD12~PD15		TIM4
PWM(not using)  PA0_PA4 PC2		TIM5
STEP_MOTOR1 PUL PA6				TIM2&TIM3
STEP_MOTOR1 DIR PA7
STEP_MOTOR1 ENA	not assigned yet

STEP_MOTOR2 PUL PB0				TIM2&TIM1
STEP_MOTOR2 DIR PB1
STEP_MOTOR2 ENA	not assigned yet

STEP_MOTOR2 PUL PC4				TIM2&TIM8
STEP_MOTOR3 DIR PC5
STEP_MOTOR3 ENA	not assigned yet

lim_switch(optc)PD0
lim_sw(theta)0	PD1
lim_sw(theta)F	PD2
lim_sw(strech_0)PD3
lim_sw(strech_F)PD4
lim_sw(W_0)		PD5
lim_sw(W_F)		PD6

conflict peripherals: MOTOR(PWM)

still lack 2-way pwm output for arm strecthing
