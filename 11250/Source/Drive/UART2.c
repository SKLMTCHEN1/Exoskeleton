//#include <stdio.h>
//#include "stm32f4xx.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_usart.h"
//#include "stm32f4xx_rcc.h"
//#include "misc.h"
#include "UART2.h"
#include "sys.h"
#include "DIO.h"
#include <stdbool.h>
//#include <stdlib.h>//用到rand()函数
#include <time.h>   //用到clock()函数
static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
extern void CopeSerial2Data(unsigned char ucData);
u16 USART_RX_STA=0;       //接收状态标记	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
int bitstatus;
void Initial_UART2(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟	
	
		//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2?????????????????????????????
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	 
	   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = baudrate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	//USART_ClearFlag(USART2,USART_FLAG_TC);	             //from JY901
  
	USART_Cmd(USART2, ENABLE);  //使能串口2
	
	//USART_ClearFlag(USART2, USART_FLAG_TC);
	//USART_ITConfig(USART2, USART_IT_TXE, DISABLE);   
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


//	USART_InitStructure.USART_BaudRate = baudrate;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART2, &USART_InitStructure); 
//	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);    
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//	
//	USART_ClearFlag(USART2,USART_FLAG_TC);	
//	USART_Cmd(USART2, ENABLE);
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}	

void USART2_IRQHandler(void)
{	
	int begintime,endtime;
	int i = 0;
	int a[1002];
//		begintime=clock();	//计时开始		
	bool test = USART_GetITStatus(USART2, USART_IT_RXNE);

//	begintime=clock();	//计时开始
//	printf("test is%d\r\n",test);
//	endtime = clock();	//计时结束
//	printf("\n\nRunning Time：%dms\n", endtime-begintime);

//	printf("test is%d\r\n",test);
//	while(1)
//     {
//				
//	
//				};
		//	printf("reset is%d\r\n",RESET);
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    USART_SendData(USART2, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART2, USART_IT_TXE);
    if(TxCounter == count) 		
		{			
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
  }
			else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
//				printf("test is\r\n");
//		 printf("1111\r\n");
//		GPIO_ResetBits(GPIOF,GPIO_Pin_10);  //LED0对应引脚GPIOF.9拉低，亮  等同LED0=0;
//		delay_ms(50);  		   //延时300ms
//		GPIO_SetBits(GPIOF,GPIO_Pin_10);	   //LED0对应引脚GPIOF.0拉高，灭  等同LED0=1;
////			printf("接收\r\n");
		
//			endtime = clock();	//计时结束
//	printf("\n\nRunning Time：%dms\n", endtime-begintime);

CopeSerial2Data((unsigned char)USART2->DR);//处理数据
				//return;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART2,USART_IT_ORE);	

}



//bitstatus=USART_GetITStatus(USART2, USART_IT_RXNE);
//CopeSerial2Data((unsigned char)USART2->DR);














//void USART2_IRQHandler(void)                	//串口1中断服务程序
//{		

//	//printf("中断\r\n");
//	//printf("%x\r\n",USART_IT_RXNE);
//	u8 Res;
////	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
////  {   
////    USART_SendData(USART2, TxBuffer[TxCounter++]); 
////    USART_ClearITPendingBit(USART2, USART_IT_TXE);
////    if(TxCounter == count) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
////  }

//	if (USART_GetITStatus(USART2, 0x0525) != RESET)
//	{

//	}
//	
//			if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{

//		//Res =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
//		
////			printf("接收到了\r\n");
//			CopeSerial2Data((unsigned char)USART2->DR);//处理数据
//			//if(USART_RX_STA&0x4000)//接收到了0x0d
//			//{
//				
//				//if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
//				//else 
//					//USART_RX_STA|=0x8000;	//接收完成了 
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//			//}
////			else //还没收到0X0D
////			{	
////				if(Res==0x0d)USART_RX_STA|=0x4000;
////				else
////				{
////					printf("接收完成\r\n");
////					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
////					USART_RX_STA++;
////					if(USART_RX_STA>(200-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
////				}		 
////			}
//}

//USART_ClearITPendingBit(USART2,USART_IT_ORE);
//}
















void UART2_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
 // USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

void UART2_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART2_Put_Char(0x0d);
			else if(*Str=='\n')UART2_Put_Char(0x0a);
				else UART2_Put_Char(*Str);
		Str++;
	}
}