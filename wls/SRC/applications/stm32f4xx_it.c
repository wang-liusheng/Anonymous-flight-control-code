#include "include.h"
#include "Ano_FlightDataCal.h"
#include "Drv_RcIn.h"

void NMI_Handler(void)
{
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

//extern int a ;
void EXTI9_5_IRQHandler(void)  
{  
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)  
    {  
      EXTI_ClearITPendingBit(EXTI_Line7);  
			
			//Fc_Sensor_Get();
    }  
		
	////////////////////////////////////////////////		
//		if(EXTI_GetITStatus(EXTI_Line6) != RESET)  
//    {  
////			a = 20;
//      EXTI_ClearITPendingBit(EXTI_Line6);  
//			
//			//Fc_Sensor_Get();
//    }  
//				 
		
/////////////////////////////////////////////////////		
//		
//		
//		if(EXTI_GetITStatus(EXTI_Line6) == 1)  
//				{  
//					a=15;
//					while  (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == 0)//等待echo的高电平到来	
//				{

//				}
//								TIM_SetCounter(TIM4,0); //清零计数
//								TIM_Cmd(TIM4, ENABLE);		//使能定时器2,开始计数

//				} 
//		else	
//				{
//										a=17;
//					
//			while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == 1)//等待echo的高电平结束
//		
//		{

//		}	

//			 TIM_Cmd(TIM4, DISABLE);	//失能定时器4,截止计数

//				}		
//				
//		
//    EXTI_ClearITPendingBit(EXTI_Line6);  
		
				
				
}


void TIM3_IRQHandler(void)
{
	PPM_IRQH();
}

//void USART1_IRQHandler(void)
//{
//	Uart1_GPS_IRQ();
//}

void USART2_IRQHandler(void)
{
	Usart2_IRQ();
}

void USART3_IRQHandler(void)
{
	Usart3_IRQ();
}

void UART4_IRQHandler(void)
{
	Uart4_IRQ();
}

void UART5_IRQHandler(void)
{
	Uart5_IRQ();
}
void USART6_IRQHandler(void)
{
	Sbus_IRQH();
}
