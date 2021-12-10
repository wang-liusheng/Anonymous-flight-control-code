#include  "exti.h"
#include "Drv_time.h"
//#include "led.h" 
//#include "key.h"
//#include "beep.h"

void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);//PD5 连接到中断线5
	
  /* 配置EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	
}

//void EXTI9_5_IRQHandler(void)  中断服务函数在stm32f4xx_it.c里
//{
////	delay_ms(10);	//消抖
////	if(KEY0==0)	 
////	{				 
////		LED0=!LED0;	
////		LED1=!LED1;	
////	}		 
//	 EXTI_ClearITPendingBit(EXTI_Line5);//清除LINE4上的中断标志位  
//}


	







