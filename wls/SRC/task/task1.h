#ifndef _task1_H//如果没有_ultr_H
#define _task1_H//创建_ultr_H

//# include "system.h"//位带操作的文件
#include "stm32f4xx.h"
//#include "usart.h"
#include "Drv_time.h"
#include "Ano_Math.h"

//函数声明
//void ultr_Init(void);
//void UltrasonicWave_StartMeasure(void);
//float UltrasonicWave_Measure(void) ;
//void _TIM4_IRQHandler(void);
void TIM4_CH1_Cap_Init(u32 arr,u16 psc);
enum
{
x1 = 0,
x2 ,
x3 ,
x4 ,
x5 ,
x6 ,
x7 ,
x8 ,
x9 ,
x10 ,
x11 ,
x12,
x13,
x14,
x15,
x16,
x17,
x18,
x19,
x20,
x21,
x22,
x23,
x24,
x25,
x26,
x27,
x28,

 array_p,//8
};

enum
{
y1 = 0,
y2 ,
y3 ,
y4 ,
y5 ,
y6 ,
y7 ,
y8,
y9,
y10,
y11,
y12,
y13,
y14,
y15,
y16, 
y17,
y18,
y19,
y20,
y21,
y22,
y23,
y24,
y25,
y26, 
y27,
y28,

 array_l,//8
};


///////////////////////////////////////////
typedef struct
{
   	
	int A_roll_exp, A_pitch_exp;
	int A_roll_fb,A_pitch_fb;
	int A_roll_err,A_pitch_err;
	

}_A;



void T265_loc1(u8 dT_ms);
void search_A(u8 dT_ms);

void coo(u8 sta,u8 sta1,u8 sta2);



#endif

