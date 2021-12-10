#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==引用

#include "Ano_FcData.h"

//==定义
typedef struct
{
	//
	u8 color_flag;
	u8 sta;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 deviation;
	u8 p_flag;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_line_tracking_st;
///////////////////////////////////
typedef struct
{

	u16 data0;
	u16 data1;
	u8 dT_ms;
	

}_openmv_user_st;

///////////////////////////////////
typedef struct
{

	s16 data0;
	s16 data1;
	s16 data2;
	s16 data3;
	s16 data4;

	u8 dT_ms;
	

}_openmv_user1_st;

typedef struct
{
	u8 offline;
	u8 mode_cmd;
	u8 mode_sta;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
	_openmv_user_st user;
	_openmv_user1_st user1;

}_openmv_data_st;
//==数据声明
extern _openmv_data_st opmv;

//==函数声明

//static
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);

void OpenMV_Byte_Get(u8 bytedata);


void uasrt2_senddata(u8 sta);


#endif

