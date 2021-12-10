#include "task1.h"
#include "Drv_usart.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_LocCtrl.h"
#include "Drv_OpenMV.h"
#include "Ano_FlightCtrl.h"



extern s16 T265_x,T265_y;
extern s16 T265_lx,T265_ly,T265_lz;
extern u16 usart_flag;

s16 usart_flag1 = 0;
u16  ass = 0;  
u32  time = 0; // 28步计时 
//u32  time1 = 0; // 28步计时 


u32  time1 = 0;  // 一键起飞计时
u16  cut_flag = 0; //
u16  cut1_flag = 0; //

u16  cut2_flag = 0; //
int t1 = 0;
int t2 = 0;


/////////////////////////////////////////////////

int x_new;
int y_new;
int x_old;
int y_old;

u16 change_flag = 1;

u16 change_flag1 = 0;





u16  start_flag=0;///区别杆的标志位

_T265 T265;  //T265数据

_A search;// openmv数据找A


s16 exp_p = 0 , exp_l = 0;
////////////////////////////////////////////////////////
int array_x[30] ;
int array_y[30] ;	

void coo(u8 sta,u8 sta1,u8 sta2)
{
	
	
	int  num[56] = {
	
		 -1,0,-1,1,0,1,0,2,-1,2,-1,3,
		-1,4,-1,5,-1,6,0,6,1,6,1,5,0,5,
		0,4,0,3,1,3,1,4,2,4,2,3,3,3,
		3,4,3,5,3,6,4,6,4,5,4,4,4,3,4,3,
	
								};
						
		for(int a = 0; a < 27 ;  a++ )
			{
					
					array_x[a] =   sta  + sta2*num[2*a];
				
					array_y[a] =   sta1 + sta2*num[2*a+1];

			}					

}



s16 A_pitch = 0 , A_roll = 0;

 
void search_A(u8 dT_ms)
{
	
static u8 sta2 = 1;	
static	 u8 sta = 2;	
static	 u8 sta1 = 3;
//static	 u8 sta5 = 0;
	
	
	
	search.A_pitch_exp =  A_pitch;
	search.A_roll_exp  =  A_roll;
	

	search.A_pitch_fb = T265_lz;
	search.A_roll_fb = T265_lx;
	
	
	search.A_pitch_err = search.A_pitch_fb - search.A_pitch_exp ;
	
	search.A_roll_err =  search.A_roll_fb - search.A_roll_exp ; 
	
	

			
if(start_flag == 1)	
{
	
	
						time += dT_ms;
	
	
							if(time ==  0)	 //1	
						{
							
							

//							exp_p += 0;   //x后
//							exp_p -= 0;		//x前
//							exp_l += 0;		//y右
//							exp_l -= 0;		//y左		
							
						}
						else if( time >= 1000 && time < 5000 ) //2
						{

									
								if(flag.unlock_sta == 1)
									{
										one_key_take_off();  //now:50cm
									}
																		
						}
						
							else if( time >= 5000 && time < 9000 ) //2							
							{
							
//									cut_flag = 1;			
								
													A_pitch = -165;
													A_roll  = 60;
	
							}	
							else if( time >= 9000 && time < 13000 ) //2							
							{
						
								
									usart_flag= sta2;							
									uasrt2_senddata(sta2);//给openmv发送数据  1
								
									start_flag = 2;  //切换到28步	
								
								
	
							}	
			
										else if( time >= 13000 && time < 14000 ) //2	
										{
													time = 0;
										}										

								else
								{

								}
	
}


			if(start_flag == 2)
			{
					change_flag =0;
					cut2_flag = 1;
			}


		coo(0,0 ,45);



if(cut2_flag == 1)
{
	
	

//				int f = 1;
				time1 += dT_ms;
		
	
	
			for(int a = 0; a < 27 ;  a++ )
			{
				
									if(a*2000< time1)
									{
										
										 A_pitch  = array_x[a] + (-165);										
										 A_roll   = array_y[a] + 60 ;
																															
									}
														
										
			}				
			

			
						if (time1 >= 28*2000)
						{							
								uasrt2_senddata(sta);//让openmv发送数 2		
						}
						
									
//						if(time1 >= 30*2000 )
//						{
//							
//							Program_Ctrl_User_Set_HXYcmps(0,0);


//						}
						
						if(time1 >= 29*2000 )
						{
							
							
									if(change_flag1 == 0)
									{
											A_pitch = 0; 
											A_roll  = 0;
									}
																
									if(change_flag1 == 1)
									{
												uasrt2_senddata(sta1);//让openmv发送数 3
												A_pitch = 0; 
												A_roll  = 70;
									}
							
						}
			
			
			
						if(time1 >= 30*2000 )
						{
									if(change_flag1 == 0)
									{
												flag.auto_take_off_land = AUTO_LAND;//一键降落				
									}		
									
									if(change_flag1 == 1)
									{
												flag.auto_take_off_land = AUTO_LAND;//一键降落				
												A_pitch = 0; 
												A_roll  = 40;
									}
							
							
						}	
						
		}








////////////////////////////////////////////////////////
			
			
	// Program_Ctrl_User_Set_HXYcmps(10,0); //X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移）
	// Program_Ctrl_User_Set_Zcmps(20); //速度（厘米每秒，正为上升，负为下降）
	// Program_Ctrl_User_Set_YAWdps(10);//	参    数: 速度（度每秒，正为右转，负为左转）
	///实时数据  赋值飞机会一直上升  赋值为零则停止	
			
			if((search.A_pitch_err != 0)||(search.A_roll_err != 0))
			{
					Program_Ctrl_User_Set_HXYcmps( 0.5*search.A_pitch_err  ,  0.5*search.A_roll_err );
			}			
			else	
			{
					Program_Ctrl_User_Set_HXYcmps(0,0);
			}	
			

	
		

}



//									if(a*1000 < time1)
//									{
//										t2 = a*1000;
//	
////												if(f==1)
////												{
////														t1 = 8;
////												uasrt2_senddata(sta);//让openmv发送数 2		
////													f=0;
////												}
////												else
////												{
////													t1 = 5;
////													uasrt2_senddata(sta1);//让openmv发送数 3
////													f=1;
////													
////												}

//									 }

//void T265_loc1(u8 dT_ms)
//{



//	


//	
//			if(start_flag == 2)
//			{
//					change_flag =0;
//					cut2_flag = 1;
//			}
//			
//			
////			if(change_flag == 1)
////			{
////				 x_old = T265_lz;
////				 y_old = T265_lx;		
////			}
////			else 
////			{
////					x_new = T265_lz;
////					y_new = T265_lx;				
////			}
////			
//			

//																														
//	T265.pitch_exp = exp_p + x_old;
//	
//	T265.roll_exp = exp_l  + y_old;
//	
//	
//	T265.pitch_fb = T265_lz;
//	
//	T265.roll_fb = T265_lx;
//	
//	
//	T265.roll_err  = T265.roll_fb  - T265.roll_exp;
//	T265.pitch_err = T265.pitch_fb - T265.pitch_exp;
//	
//	
//	// Program_Ctrl_User_Set_HXYcmps(10,0); //X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移）
//	// Program_Ctrl_User_Set_Zcmps(20); //速度（厘米每秒，正为上升，负为下降）
//	// Program_Ctrl_User_Set_YAWdps(10);//	参    数: 速度（度每秒，正为右转，负为左转）
//	///实时数据  赋值飞机会一直上升  赋值为零则停止	

////		coo(x_old,y_old ,50);
////		
//		coo(0,0 ,50);

//if(cut2_flag == 1)
//{
//	
//	

//				int f = 1;
//				time1 += dT_ms;
//		
//	
//	
//			for(int a = 0; a < 28 ;  a++ )
//			{
//				


//									if(a*2000< time1)
//									{
//										
//										 exp_p   = array_x[a];										
//										 exp_l   = array_y[a];
//																															
//									}
//									
//									
//									if(a*1000 < time1)
//									{
//										t2 = a*1000;
//	
////												if(f==1)
////												{
////														t1 = 8;
////												uasrt2_senddata(sta);//让openmv发送数 2		
////													f=0;
////												}
////												else
////												{
////													t1 = 5;
////													uasrt2_senddata(sta1);//让openmv发送数 3
////													f=1;
////													
////												}

//									 }
//											
//										
//			}				
//			
//			
//						if (time1 >= 29*2000)
//						{
////										T265.pitch_exp   = 0;										
////										T265.roll_exp 	 = 0;
////							
////								A_pitch = 0; 
////								A_roll  = 0;	
//							
////									search.A_pitch_exp = 0;
////									search.A_roll_exp = 0;
//							
//							uasrt2_senddata(sta);//让openmv发送数 2	
//							
//						}
//						
//									
//						if(time1 >= 31*2000 )
//						{
//							
//							Program_Ctrl_User_Set_HXYcmps(0,0);


//						}
//						
//						if(time1 >= 32*2000 )
//						{
//							
//							
//									if(change_flag1 == 0)
//									{
//											search.A_pitch_exp = 165;
//											search.A_roll_exp = 60;
//									}
//																
//									if(change_flag1 == 1)
//									{
//											uasrt2_senddata(sta1);//让openmv发送数 3
//											search.A_pitch_exp = 165;
//											search.A_roll_exp = 93+60;
//									}
//							
//						}
//			
//			
//			
//						if(time1 >= 32*2000 )
//						{
//							
//							flag.auto_take_off_land = AUTO_LAND;//一键降落			
//							
//						}	

//					


//	
//			if((T265.roll_err != 0)||(T265.roll_err != 0))
//			{
//				Program_Ctrl_User_Set_HXYcmps( 0.9*T265.pitch_err  ,  0.5*T265.roll_err );  //   0.8/0.8
//			}			
//			else
//			{
//				Program_Ctrl_User_Set_HXYcmps(0,0);
//			}
//	
//			
//			
//}

//					

//				


//}



////////////////////////////////////
			
//int  num[4][7] = {
//	
//		(0,0),(-1,0),(-1,1),(0,1),(0,2),(-1,2),(-1,3),
//		(-1,4),(-1,5),(-1,6),(0,6),(1,6),(1,5),(0,5),
//		(0,4),(0,3),(1,3),(1,4),(2,4),(2,3),(3,3),
//		(3,4),(3,5),(3,6),(4,6),(4,5),(4,4),(4,3)  
//	
//								};		

 
//	
//int  num[56] = {
//	
//		0,0,-1,0,-1,1,0,1,0,2,-1,2,-1,3,
//		-1,4,-1,5,-1,6,0,6,1,6,1,5,0,5,
//		0,4,0,3,1,3,1,4,2,4,2,3,3,3,
//		3,4,3,5,3,6,4,6,4,5,4,4,4,3  
//	
////								};		
//	
//int array1[29] = {  
//										15,-50,-50,0,-50,-50,-50,
//										-50,-50,-50, 0,  50, 50,0,
//										0, 0, 50,50,100, 100,150,
//										150,150,150,200,200,200,200,0
//									};	 //x

//int array2[29]=  {  
//										-15,0,50,50,100,100,150,
//										200,250,300, 300, 300, 250,250,
//										200,150, 150,200, 200, 150,150,
//										200,250,300,300,250,200,150,0
//									};   //y





