v1.0.4.20180703
��������������Ѿ��ɹ����룬������ת���ϵĻ����Լ���·��ʹ�����豸�����ࡣ
v1.0.3.20180627
�ڶ�����������Ѿ�����ɹ�������������������������һ�汾�ȼ����翪���Լ�������λ����
v1.0.2.20180623
���������ɣ�������ڶ��������������
v1.0.1.20180615
GPS��Ϣ���ճ�����ɣ�����ֱ����������ĽǶ��Ƶ�GPS��Ϣ��
v1.0.0.20180608
���������Ļ���ucos iii�ĵ����˶����Ƴ���ģ�鵥������������δ�������ԡ�




����  			���Ŷ���		ʹ�ö�ʱ��	
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
