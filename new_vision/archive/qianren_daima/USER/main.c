#include "led.h"
#include "delay.h"
#include "sys.h"
#include "key.h"
#include "EXIT.h"
#include "LobotSerialServo.h"
#include "action_new.h"
#include "stm32f10x.h"
#include <stdio.h>

#include "usart.h"
 
#include "pwm.h" 

#include "bool.h"
#include "Bluetooth.h"

#include "Uart4.h"
#include "Usart3.h"
#include "jingzou.h"


//extern u8 Res;
int main(void)
{
	delay_ms(200);
	//NVIC优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	//大舵机初始化
	uartInit(115200);	
	//LED初始化
	LED_Init();
	//蓝牙初始化
	bluetooth_init(9600);
	//小舵机初始化
	servo_init(); 
	//外部中断+按键初始化
	EXTIX_Init();
	
	//串口3初始化
	//Usart3_init(9600);
	//串口4初始化
	Uart4_init(9600);
	
	//LED全部开启
	LED0=1;
	LED1=1;
	
	//归中
	new_guizhong();
	Guizhong_small();
	
//	//等待按下开始键
//	while(Start==0){}; 

while(1)
{
	LED0 = 1;
	LED1 = 1;
	delay_ms(100);
	LED0 = 0;
	LED1 = 0;
	delay_ms(100);
	
}

//	servo_angle4(90);
//	servo_angle3(90);
//	servo_angle2(40);
//	servo_angle1(40);
//	delay_ms(805); 
//	servo_angle4(10);
//	servo_angle3(10);
//	servo_angle2(100);
//	servo_angle1(80);
//	delay_ms(805); 

//  while(1)
//	{
//		delay_ms(3000);
//		if(a==0)
//		{
//			guizhong();
//		}
//		else if(a==1)
//		{
//			zouluend();
//		}
//		else if(a==2)
//		{
//			zuozhuanend();
//		}
//		else if(a==3)
//		{
//			youzhuanend();
//		}
//	}
 }
