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
//#include <stdlib.h>//�õ�rand()����
#include <time.h>   //�õ�clock()����
static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
extern void CopeSerial2Data(unsigned char ucData);
u16 USART_RX_STA=0;       //����״̬���	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
int bitstatus;
void Initial_UART2(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��	
	
		//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2?????????????????????????????
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3
	 
	   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = baudrate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
	//USART_ClearFlag(USART2,USART_FLAG_TC);	             //from JY901
  
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
	
	//USART_ClearFlag(USART2, USART_FLAG_TC);
	//USART_ITConfig(USART2, USART_IT_TXE, DISABLE);   
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����


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
//		begintime=clock();	//��ʱ��ʼ		
	bool test = USART_GetITStatus(USART2, USART_IT_RXNE);

//	begintime=clock();	//��ʱ��ʼ
//	printf("test is%d\r\n",test);
//	endtime = clock();	//��ʱ����
//	printf("\n\nRunning Time��%dms\n", endtime-begintime);

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
//		GPIO_ResetBits(GPIOF,GPIO_Pin_10);  //LED0��Ӧ����GPIOF.9���ͣ���  ��ͬLED0=0;
//		delay_ms(50);  		   //��ʱ300ms
//		GPIO_SetBits(GPIOF,GPIO_Pin_10);	   //LED0��Ӧ����GPIOF.0���ߣ���  ��ͬLED0=1;
////			printf("����\r\n");
		
//			endtime = clock();	//��ʱ����
//	printf("\n\nRunning Time��%dms\n", endtime-begintime);

CopeSerial2Data((unsigned char)USART2->DR);//��������
				//return;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART2,USART_IT_ORE);	

}



//bitstatus=USART_GetITStatus(USART2, USART_IT_RXNE);
//CopeSerial2Data((unsigned char)USART2->DR);














//void USART2_IRQHandler(void)                	//����1�жϷ������
//{		

//	//printf("�ж�\r\n");
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
//			if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{

//		//Res =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
//		
////			printf("���յ���\r\n");
//			CopeSerial2Data((unsigned char)USART2->DR);//��������
//			//if(USART_RX_STA&0x4000)//���յ���0x0d
//			//{
//				
//				//if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				//else 
//					//USART_RX_STA|=0x8000;	//��������� 
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//			//}
////			else //��û�յ�0X0D
////			{	
////				if(Res==0x0d)USART_RX_STA|=0x4000;
////				else
////				{
////					printf("�������\r\n");
////					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
////					USART_RX_STA++;
////					if(USART_RX_STA>(200-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
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