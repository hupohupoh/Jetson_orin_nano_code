#include "led.h"


//初始化PC6和PC7为输出口.并使能这两个口的时钟, 仅作测试使用，可屏蔽


void LED_Init(void)//LED初始化函数，初始化PC6为低电平，PC7为高电平
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能GPIOC时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 //PC.6 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化PC.6
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);						 //PC.6 输出低电平

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	    		 //PC.7 端口配置, 推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //推挽输出，IO口速度为50MHz
	GPIO_SetBits(GPIOC,GPIO_Pin_7);							 //PC.7 输出高电平
}
 

