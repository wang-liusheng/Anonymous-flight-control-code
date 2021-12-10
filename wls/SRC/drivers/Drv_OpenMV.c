//默认引用：
#include "Drv_OpenMV.h"
#include "Drv_usart.h"

//设定
#define OPMV_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
u16 offline_check_time;
u8 openmv_buf[20];
_openmv_data_st opmv;
/**********************************************************************************************************
*函 数 名: OpenMV_Byte_Get
*功能说明: OpenMV字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void OpenMV_Byte_Get(u8 bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	openmv_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(1)//(bytedata==0x29)未确定
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(bytedata==0x41 || bytedata==0x42 || bytedata==0x43 || bytedata==0x44)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<20)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += openmv_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			OpenMV_Data_Analysis(openmv_buf,len+6);
			//
			rec_sta=0;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else
	{
		//	
		rec_sta++;
	}
	
}

/**********************************************************************************************************
*函 数 名: OpenMV_Data_Analysis
*功能说明: OpenMV数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
#include "Ano_DT.h"
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len)
{
	if(*(buf_data+3)==0x41)
	{
		opmv.cb.color_flag = *(buf_data+5);
		opmv.cb.sta = *(buf_data+6);
		opmv.cb.pos_x = (s16)((*(buf_data+7)<<8)|*(buf_data+8));
		opmv.cb.pos_y = (s16)((*(buf_data+9)<<8)|*(buf_data+10));
		opmv.cb.dT_ms = *(buf_data+11);
		//
		opmv.mode_sta = 1;
	}
	else if(*(buf_data+3)==0x42)
	{
		opmv.lt.sta = *(buf_data+5);
		opmv.lt.angle = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
		opmv.lt.deviation = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
		opmv.lt.p_flag = *(buf_data+10);
		opmv.lt.pos_x = (s16)((*(buf_data+11)<<8)|*(buf_data+12));
		opmv.lt.pos_y = (s16)((*(buf_data+13)<<8)|*(buf_data+14));
		opmv.lt.dT_ms = *(buf_data+15);
		//
		opmv.mode_sta = 2;
	}

		else if(*(buf_data+3)==0x43)
		{
				opmv.user.data0 = (s16)((*(buf_data+5)<<8)|*(buf_data+6));
				opmv.user.data1 = (s16)((*(buf_data+7)<<8)|*(buf_data+8));
				opmv.user.dT_ms = *(buf_data+9);


		}

			else if(*(buf_data+3)==0x44)
		{
//				opmv.user1.data0 = (s16)((*(buf_data+5)<<8)|*(buf_data+6));
//				opmv.user1.data1 = (s16)((*(buf_data+7)<<8)|*(buf_data+8));
//				opmv.user1.data2 = (s16)((*(buf_data+9)<<8)|*(buf_data+10));
//				opmv.user1.data3 = (s16)((*(buf_data+11)<<8)|*(buf_data+12));				
//				opmv.user1.data4 = (s16)((*(buf_data+13)<<8)|*(buf_data+14));
//				opmv.user1.dT_ms = *(buf_data+11);
			
				opmv.user1.data0 = *(buf_data+5);
				opmv.user1.data1 = *(buf_data+6);//(s16)((*(buf_data+7)<<8)|*(buf_data+8));
				opmv.user1.data2 = *(buf_data+7);//(s16)((*(buf_data+9)<<8)|*(buf_data+10));
				opmv.user1.data3 = *(buf_data+8);//(s16)((*(buf_data+11)<<8)|*(buf_data+12));				
				opmv.user1.data4 = *(buf_data+9);//(s16)((*(buf_data+13)<<8)|*(buf_data+14));
//				opmv.user1.dT_ms = *(buf_data+11);


		}



	//
	OpenMV_Check_Reset();
}

/**********************************************************************************************************
*函 数 名: OpenMV_Offline_Check
*功能说明: OpenMV掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void OpenMV_Offline_Check(u8 dT_ms)
{
	if(offline_check_time<OPMV_OFFLINE_TIME_MS)
	{
		offline_check_time += dT_ms;
	}
	else
	{
		opmv.offline = 1;
		opmv.mode_sta = 0;
	}
	
}

/**********************************************************************************************************
*函 数 名: OpenMV_Check_Reset
*功能说明: OpenMV掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void OpenMV_Check_Reset()
{
	offline_check_time = 0;
	opmv.offline = 0;
}
////////////////////////////////////


u8 datasend[100];
void uasrt2_senddata(u8 sta)
{
	u8 _cnt = 0;
	datasend[_cnt++] = 0xAA;
	datasend[_cnt++] = 0xAF;
	datasend[_cnt++] = 0x05;
	datasend[_cnt++] = 0x44;
	
	datasend[_cnt++] = 0x06;

	datasend[_cnt++] = sta;
	
	datasend[_cnt++] = 0x00;

	
	u8 sc = 0;
	u8 ac = 0;
	for(u8 i= 0;i<datasend[3]+4;i++)
		{
			sc += datasend[i];
			ac += sc;
			
		}
		
	datasend[_cnt++] = sc;
	datasend[_cnt++] = ac;


	
	Usart2_Send(datasend,_cnt);



}

