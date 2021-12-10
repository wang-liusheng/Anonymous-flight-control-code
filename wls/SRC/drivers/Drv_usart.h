#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"

///////////////////////////////
#define PROTOCOL_HEADER		0XFEFEFEFE
#define PROTOCOL_END		0XEE

#define PROTOCL_DATA_SIZE 77

#pragma pack(1)


#define ENCODER_TTL_COUNT_VALUE 	1300.0f			//轮子每圈的脉冲数
#define ROBOT_WHEEL_DIAMETER		0.085f			//主动轮直径
#define ROBOT_WHEEL_WIDTH			0.185f			//机器人水平轮距,宽度
#define ROBOT_WHEEL_LENGTH		 	0.175f			//机器人垂直轮距,长度

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 

extern u16 USART_RX_STA;  


//接收状态标记	
typedef struct __Mpu6050_Str_
{
	short X_data;
	short Y_data;
	short Z_data;
}Mpu6050_Str;






typedef struct _Moto_
{
	int Encoder_Value;
	float Current_Speed;
	float Target_Speed;
	short ESC_Output_PWM;
	float L_Error;
	float LL_Error;
}_Moto_Str;
extern _Moto_Str Moto1, Moto2, Moto3, Moto4;


typedef struct __Moto_Str_
{
	float Moto_CurrentSpeed;
	float Moto_TargetSpeed;
}Moto_Str;

      //00 00 00 80 00 00 00 80 00 00 00 00 00 00 00 80  

typedef union _Upload_Data_   
{
	unsigned char buffer[PROTOCL_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned int Header;	//EE FE FE FE FE
		float X_speed;			//00 00 00 00		
		float Y_speed;			//00 00 00 00
		float Z_speed;			//00 00 00 00
		float Source_Voltage;	//D7 E3 47 41
		
		Mpu6050_Str Link_Accelerometer;		//C4 06 F0 FD 6C 38
		Mpu6050_Str Link_Gyroscope;			//D8 FF 03 00 EB FF
		
		Moto_Str MotoStr[4];		//
		float PID_Param[3];					//00 00 00 00 00 00 00 00 00 00 00 00
		
		unsigned char End_flag;				//EE
	}Sensor_Str;
}Upload_Data;


///////////////////////////////////////////////////////////  T265

typedef union _Upload_Data1_   
{
		unsigned char buffer[29];
	unsigned int Header;
	float fvalue[6];
	unsigned char End_flag;	
}Upload_Data1;
///////////////////////////////////////////////////////

#pragma pack(4)
	
extern Upload_Data Send_Data, Recive_Data;
	
extern float send_data[4];	

//void Huanyu_Usart1_Init(u32 bound);
//void USART1_SendChar(unsigned char b);
//void Huanyu_SendTo_UbuntuPC(void);
//void Kinematics_Positive(float vx,float vy, float vth);
//void shanwai_send_data1(uint8_t *value, uint32_t size);
//void Data_depack(void);

void uart_init(u32 bound);
///////////////////

extern u8 Rx_Buf[];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Usart3_Init ( u32 br_num );
void Usart3_IRQ ( void );
static void Usart3_Send ( unsigned char *DataToSend , u8 data_num );

void Uart4_Init(u32 br_num);
void Uart4_IRQ(void);
void Uart4_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);



#endif
