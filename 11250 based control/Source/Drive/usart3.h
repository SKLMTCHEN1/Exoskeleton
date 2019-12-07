#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收

//如果想串口中断接收，请不要注释以下宏定义
void uart3_init(u32 bound);
void USART3_Put_Char(unsigned char DataToSend);
void USART3_Put_String(unsigned char *Str,unsigned char length);
#endif

//------------------End of File----------------------------



