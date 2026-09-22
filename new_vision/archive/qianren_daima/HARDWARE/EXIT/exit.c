#include "EXIT.h"
#include "led.h"
#include "key.h"
#include "delay.h"


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//外部中断 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/01  
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved	  
////////////////////////////////////////////////////////////////////////////////// 	  

uint8_t Start;//启动程序标志位
 
//外部中断初始化函数
void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

	  KEY_Init();//初始化按键对应io模式

    //A8 中断线以及中断初始化配置
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);

//  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
//  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
//  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    //C8  中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    //C9	  中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource9);

   	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 

 
}

 

 void EXTI9_5_IRQHandler(void)
{	
	
	//exti_line8
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == 0)
		{
			while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == 0);
			if(Start != 0x01 )
			{
				Start  = 0x01;
			}
			else if(Start  == 0x01)
			{
				Start   = 0x00;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line8);		
	}
	
	//exti_line9	
	if(EXTI_GetITStatus(EXTI_Line9) == SET)
	{
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == 0)
		{
			while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == 0);
			if(Start != 0x01 )
			{
				Start  = 0x01;
			}
			else if(Start  == 0x01)
			{
				Start   = 0x00;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
}


