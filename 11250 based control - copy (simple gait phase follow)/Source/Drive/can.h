#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
#endif


#ifndef __CAN_H__
#define __CAN_H__

#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

void CAN1_Configuration(void);

void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number);
//void matrixMul(int **A, int **B,int **C, int rowA, int columnB, int columnA);
//	
extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];

#endif














