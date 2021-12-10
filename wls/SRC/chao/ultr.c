# include "ultr.h"
//# include "SysTick.h"//延时函数的头文件
#include "stm32f4xx.h"
//#include "usart.h"
#include "Drv_time.h"

//Pd5管脚接trig  Pd12管脚接echa

void ultr_Init()//ultr初始化函数
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体变量
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//使能端口F时钟
	
	//GOIO初始化
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //输出模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;//管脚设置
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//速度设置
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化结构体
	GPIO_ResetBits(GPIOD,GPIO_Pin_5);//初始化led为低电平}
}

TIM_ICInitTypeDef  TIM4_ICInitStructure;

//定时器5通道1输入捕获配置
//arr：自动重装值(TIM2,TIM5是32位的!!)
//psc：时钟预分频数
void TIM4_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA0
	


	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //PA0复用位定时器5
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	

	//初始化TIM5输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM4,ENABLE ); 	//使能定时器5

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}
u8  TIM4CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM4CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)

void TIM4_IRQHandler(void)
{
	
 	if((TIM4CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM4CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM4CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM4CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM4CH1_CAPTURE_VAL=0XFFFF;
				}else TIM4CH1_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM4CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM4CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM4CH1_CAPTURE_VAL=TIM_GetCapture1(TIM4);//获取当前的捕获值.
	 			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM4CH1_CAPTURE_STA=0;			//清空
				TIM4CH1_CAPTURE_VAL=0;
				TIM4CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_Cmd(TIM4,DISABLE ); 	//关闭定时器5
	 			TIM_SetCounter(TIM4,0);
	 			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM4,ENABLE ); 	//使能定时器5
			}		    
		}			     	    					   
 	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位

}


