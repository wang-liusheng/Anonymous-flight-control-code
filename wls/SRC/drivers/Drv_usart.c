/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：串口驱动
**********************************************************************************/
#include "Drv_usart.h"
#include "Ano_DT.h"
#include "Ano_OF.h"
#include "Drv_OpenMV.h"
#include "Drv_laser.h"
#include "include.h"
#include "Drv_UP_Flow.h"


Upload_Data1 Send_Data1, Recive_Data1;

 
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
int ac=1;
void uart_init(u32 bound)
	
	{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
   ac =1;
#endif
	
}


u8 Tx1Buffer[256];
u8 Tx1Counter = 0;
u8 count1 = 0;

u8 cc,bp;

s16 T265_x,T265_y,T265_z;

s16 T265_lx,T265_ly,T265_lz;


float V_x=0,V_y=0,V_z=0;
float L_x=0,L_y=0,L_z=0;

//int ix=0,iy=0,iz=0;

unsigned char Rcount = 0;
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	
	
	u8 Res;
	ac =12;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	ac =2;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		Recive_Data1.buffer[Rcount] = USART_ReceiveData(USART1);
		//Data_depack(&Res);
		ac =3;
		(Recive_Data1.buffer[0] == 0xFE)?(Rcount++):(Rcount = 0);
		if (Rcount == 29)	//验证数据包的长度
		{
//			if(Recive_Data.Header == 0xffffffff)	//验证数据包的头部校验信息
//			{
				if(Recive_Data1.End_flag == 0xfe)	//验证数据包的尾部校验信息
				{
					//接收上位机控制命令，使机器人产生相应的运动
				//	Kinematics_Positive(Recive_Data.Sensor_Str.X_speed, Recive_Data.Sensor_Str.Y_speed, Recive_Data.Sensor_Str.Z_speed);
					
					T265_x = V_x = Recive_Data1.fvalue[0];
					
					V_y = Recive_Data1.fvalue[1];
					
					T265_y = V_z = Recive_Data1.fvalue[2];
					
					
					T265_lx = L_x = Recive_Data1.fvalue[3];
					
					T265_ly = L_y = Recive_Data1.fvalue[4];
					
					T265_lz = L_z = Recive_Data1.fvalue[5];
					
					
					bp=77;

				}
			//}
			Rcount = 0;
		}
		//cc=Res;
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				ac=5;
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					ac =4;
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
		
  }
//发送（进入移位）中断
    if (USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART1->DR = Tx1Buffer[Tx1Counter++]; //写DR清除中断标志
        if (Tx1Counter == count1)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }	
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 

#endif	

///////////////////////////////////////////////////////////////////////////




//#if EN_USART1_RX   

//u8 USART_RX_BUF[USART_REC_LEN];     

//u16 USART_RX_STA=0;       
//int ac=0;

//float KP2 = 225.9f; 
//float KI2 =  150.0f; 
//float KD2 =  50.0f;	
//_Moto_Str Moto1, Moto2, Moto3, Moto4;
//Upload_Data Send_Data, Recive_Data;
//void Huanyu_Usart1_Init(u32 bound)
//{
//	ac=1;

//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//Ê¹ÄÜUSART1Ê±ÖÓ

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9¸´ÓÃÎªUSART1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10¸´ÓÃÎªUSART1

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9ÓëGPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//¸´ÓÃ¹¦ÄÜ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ËÙ¶È50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //ÍÆÍì¸´ÓÃÊä³ö
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //ÉÏÀ­
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //³õÊ¼»¯PA9£¬PA10

//	USART_InitStructure.USART_BaudRate = bound;//²¨ÌØÂÊÉèÖÃ
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
//	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅ¼Ð£ÑéÎ»
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½
//  USART_Init(USART1, &USART_InitStructure); //³õÊ¼»¯´®¿Ú1
//	
//  USART_Cmd(USART1, ENABLE);  //Ê¹ÄÜ´®¿Ú1 
//	

//#if EN_USART1_RX	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//¿ªÆôÏà¹ØÖÐ¶Ï

//	//Usart1 NVIC ÅäÖÃ
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//´®¿Ú1ÖÐ¶ÏÍ¨µÀ
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//ÇÀÕ¼ÓÅÏÈ¼¶3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//×ÓÓÅÏÈ¼¶3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
//	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡¢
// 
//#endif
//	ac=2;
//}

///*
// @ describetion:usart send a char data
// @ param: b:data
// @ return: none
// @ author: Xuewei Zhou
// @ date : 2019-4-17
// @ note: 
// @ function: void USART1_SendChar(unsigned char b)
//*/
////void USART1_SendChar(unsigned char b)
////{
////    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
////			USART_SendData(USART1,b);
////}






////float send_data[4];
////void shanwai_send_data1(uint8_t *value,uint32_t size )
////{
////	USART1_SendChar(0x03);
////	USART1_SendChar(0xfc);
////	while(size)
////	{
////		USART1_SendChar(*value);
////		value++;
////		size--;
////	}
////	USART1_SendChar(0xfc);   
////	USART1_SendChar(0x03);
////}




////u8 Tx1Buffer[256];
////u8 Tx1Counter = 0;
////u8 count1 = 0;

////u8 cc;
////void USART1_IRQHandler(void)           
////{
////	
////	u8 Res;
////	ac =12;
////#if SYSTEM_SUPPORT_OS 		
////	OSIntEnter();    
////#endif
////	ac =222;
////	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
////	{
////		
////		Res =USART_ReceiveData(USART1);//(USART1->DR);	
////		//Data_depack(&Res);
////		ac =3;
////		cc=Res;
////		if((USART_RX_STA&0x8000)==0)
////		{
////			if(USART_RX_STA&0x4000)
////			{
////				ac=5;
////				if(Res!=0x0a)USART_RX_STA=0;
////				else USART_RX_STA|=0x8000;	
////			}
////			else //»¹Ã»ÊÕµ½0X0D
////			{	
////				if(Res==0x0d)USART_RX_STA|=0x4000;
////				else
////				{
////					ac =4;
////					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
////					USART_RX_STA++;
////					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;	  
////				}		 
////			}
////		}
////		
////  }

////    if (USART_GetITStatus(USART1, USART_IT_TXE))
////    {
////        USART1->DR = Tx1Buffer[Tx1Counter++]; 
////        if (Tx1Counter == count1)
////        {
////            USART1->CR1 &= ~USART_CR1_TXEIE;
////        }
////    }	
////#if SYSTEM_SUPPORT_OS 	
////	OSIntExit();  											 
////#endif
////} 

//////接收X轴，Y轴，TH角度
////int xx=0,yy=0,th=0;
////int xxx =0 ,yyy=0,ttt=0;
////void Data_depack()
////{ 
////	u8 len;
////	u8 t;
////	if(USART_RX_STA&0x8000)
////		{					   
////			len=USART_RX_STA&0x3fff;
////			for(t=0;t<len;t++)
////			{
////				 USART_RX_BUF[t];				 
////					xxx=USART_RX_BUF[0]<<24|USART_RX_BUF[1]<<16|USART_RX_BUF[2]<<8|+USART_RX_BUF[3];
////					yyy=USART_RX_BUF[4]<<24|USART_RX_BUF[5]<<16|USART_RX_BUF[6]<<8|+USART_RX_BUF[7];				 
////					ttt=USART_RX_BUF[8]<<24|USART_RX_BUF[9]<<16|USART_RX_BUF[10]<<8|+USART_RX_BUF[11];
////					if ((xxx-xx<10) && (xxx-xx>-10))
////					{
////						xx =xxx;
////					}
////					if ((yyy-yy<10) && (yyy-yy>-10))
////					{
////						yy =yyy;
////					}
////					if ((ttt<180) && (ttt>-180) && (ttt!= 0))
////					{
////						 th=ttt;
////					}
////			}
////		
////			USART_RX_STA=0;
////		}
////}


////#endif



/////
//extern u8 otherDataTmp[64];	//非循环发送数据临时缓冲






//====uart2
void Usart2_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART2, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART2_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource5, GPIO_AF_USART2 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource6, GPIO_AF_USART2 );

    //配置PD5作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
    //配置PD6作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置USART2
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART2时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART2, &USART_InitStructure );
    USART_ClockInit ( USART2, &USART_ClockInitStruct );

    //使能USART2接收中断
    USART_ITConfig ( USART2, USART_IT_RXNE, ENABLE );
    //使能USART2
    USART_Cmd ( USART2, ENABLE );
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//	}


}

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

u8 Rx_Buf[256];	//串口接收缓存

void Usart2_IRQ ( void )
{
    u8 com_data;

    if ( USART2->SR & USART_SR_ORE ) //ORE中断
    {
        com_data = USART2->DR;
    }

    //接收中断
    if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志

        com_data = USART2->DR;
        //AnoDTRxOneByteUart ( com_data );
				OpenMV_Byte_Get( com_data );

    }
    //发送（进入移位）中断
    if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
    {

        USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
        if ( TxCounter == count )
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }



}

void Usart2_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i );
    }

    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}


//====uart3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Ano_UWB.h"
void Usart3_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOB, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource10, GPIO_AF_USART3 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource11, GPIO_AF_USART3 );

    //配置PD5作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    //配置PD6作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    //配置USART2
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART2时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART3, &USART_InitStructure );
    USART_ClockInit ( USART3, &USART_ClockInitStruct );

    //使能USART2接收中断
    USART_ITConfig ( USART3, USART_IT_RXNE, ENABLE );
    //使能USART2
    USART_Cmd ( USART3, ENABLE );
}

u8 Tx3Buffer[256];
u8 Tx3Counter = 0;
u8 count3 = 0;
void Usart3_IRQ ( void )
{
	u8 com_data;
	
    if ( USART3->SR & USART_SR_ORE ) //ORE中断
        com_data = USART3->DR;

    //接收中断
    if ( USART_GetITStatus ( USART3, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( USART3, USART_IT_RXNE ); //清除中断标志
        com_data = USART3->DR;
//		Ano_UWB_Get_Byte ( com_data );
			OpenMV_Byte_Get( com_data );
    }
    //发送（进入移位）中断
    if ( USART_GetITStatus ( USART3, USART_IT_TXE ) )
    {
        USART3->DR = Tx3Buffer[Tx3Counter++]; //写DR清除中断标志
        if ( Tx3Counter == count3 )
        {
            USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }
    }
}

static void Usart3_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx3Buffer[count3++] = * ( DataToSend + i );
    }
    if ( ! ( USART3->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART3, USART_IT_TXE, ENABLE ); //打开发送中断
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//====uart4

void Uart4_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
//		USART_ClockInitTypeDef USART_ClockInitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART4, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART4_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART4_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource0, GPIO_AF_UART4 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource1, GPIO_AF_UART4 );

    //配置PC12作为UART5　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //配置PD2作为UART5　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //配置UART5
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init ( UART4, &USART_InitStructure );

    //使能UART5接收中断
    USART_ITConfig ( UART4, USART_IT_RXNE, ENABLE );
    //使能USART5
    USART_Cmd ( UART4, ENABLE );
}
u8 Tx4Buffer[256];
u8 Tx4Counter = 0;
u8 count4 = 0;

void Uart4_IRQ ( void )
{
    u8 com_data;

    //接收中断
    if ( USART_GetITStatus ( UART4, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART4, USART_IT_RXNE ); //清除中断标志

        com_data = UART4->DR;
				//====
				//匿名光流解析
				if(of_init_type!=2)
				{
					AnoOF_GetOneByte(com_data);
				}
				//优像光流解析
				if(of_init_type!=1)
				{
					OFGetByte(com_data);
				}
//		if(LASER_LINKOK)
//			Drv_Laser_GetOneByte( com_data);
//		else
//			AnoOF_GetOneByte ( com_data );
    }

    //发送（进入移位）中断
    if ( USART_GetITStatus ( UART4, USART_IT_TXE ) )
    {

        UART4->DR = Tx4Buffer[Tx4Counter++]; //写DR清除中断标志

        if ( Tx4Counter == count4 )
        {
            UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

}

void Uart4_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx4Buffer[count4++] = * ( DataToSend + i );
    }

    if ( ! ( UART4->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART4, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}

//====uart5

void Uart5_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART5_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART5_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

    //配置PC12作为UART5　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
    //配置PD2作为UART5　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置UART5
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init ( UART5, &USART_InitStructure );



    //使能UART5接收中断
    USART_ITConfig ( UART5, USART_IT_RXNE, ENABLE );
    //使能USART5
    USART_Cmd ( UART5, ENABLE );
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter = 0;
u8 count5 = 0;

void Uart5_IRQ ( void )
{
    //接收中断
    if ( USART_GetITStatus ( UART5, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART5, USART_IT_RXNE ); //清除中断标志

        u8 com_data = UART5->DR;
		//====
		//Drv_Laser_GetOneByte(com_data);
		AnoDTRxOneByteUart ( com_data );
    }

    //发送（进入移位）中断
    if ( USART_GetITStatus ( UART5, USART_IT_TXE ) )
    {

        UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志

        if ( Tx5Counter == count5 )
        {
            UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

}

void Uart5_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx5Buffer[count5++] = * ( DataToSend + i );
    }

    if ( ! ( UART5->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART5, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

