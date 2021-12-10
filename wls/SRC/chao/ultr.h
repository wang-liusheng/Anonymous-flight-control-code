#ifndef _ultr_H//如果没有_ultr_H
#define _ultr_H//创建_ultr_H

//# include "system.h"//位带操作的文件
#include "stm32f4xx.h"
//#include "usart.h"
#include "Drv_time.h"
//函数声明

//void ultr_Init(void);
//void UltrasonicWave_StartMeasure(void);
//float UltrasonicWave_Measure(void) ;
//void _TIM4_IRQHandler(void);
void TIM4_CH1_Cap_Init(u32 arr,u16 psc);

void T265_loc1(u8 dT_ms);

#endif

