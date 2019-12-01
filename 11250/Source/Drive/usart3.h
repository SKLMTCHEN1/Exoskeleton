#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart3_init(u32 bound);
void USART3_Put_Char(unsigned char DataToSend);
void USART3_Put_String(unsigned char *Str,unsigned char length);
#endif

//------------------End of File----------------------------



