/*
��д�ߣ�Kevin
��ַ��http://RobotControl.taobao.com
����E-mail��1609370741@qq.com
���뻷����MDK-Lite  Version: 5.17
����ʱ��: 2016-1-31
���ԣ� ���������ڡ������ǿء���STM32Coreƽ̨����ɲ���
���ܣ�
��STM32Coreƽ̨����2��ȡJY901�����ݣ�Ȼ��ͨ������1��ӡ����������,�������ֲ�����ҪѡΪ9600��
JY-901�Ĳ�����Ҫ�޸�Ϊ9600.
ע�⣺ʾ�������������ASCLL�룬��16���ƣ�HEX����ʾ�ǲ��ܿ���׼ȷ���ݵġ�
Ӳ�����ߣ�
USB-TTL����                 STM32Core               JY901
VCC          -----           VCC        ----         VCC
TX           -----           RX1���ܽ�10��     
RX           -----           TX1���ܽ�9��
GND          -----           GND        ----          GND
                             RX2 ���ܽ�3��       ----  TX
														 TX2 ���ܽ�2��       ----  RX
------------------------------------
 */
#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "usart.h"
#include "UART2.h"
#include "usart3.h"
#include "delay.h"
#include "JY901.h"
#include "DIO.h"
#include "lcd.h"
#include <stdbool.h>
#include "sys.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

struct STime		stcTime1;
struct SAcc 		stcAcc1;
struct SGyro 		stcGyro1;
struct SAngle 	stcAngle1;
struct SMag 		stcMag1;
struct SDStatus stcDStatus1;
struct SPress 	stcPress1;
struct SLonLat 	stcLonLat1;
struct SGPSV 		stcGPSV1;
struct SQ       stcQ1;

//char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
//char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����

////�ô���2��JYģ�鷢��ָ��
//void sendcmd(char cmd[])
//{
//	char i;
//	for(i=0;i<5;i++)
//		UART2_Put_Char(cmd[i]);
//}


//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	int begintime,endtime;
	int i = 0;
	int a[1002];
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}
void CopeSerial3Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer1[250];
	static unsigned char ucRxCnt1 = 0;	
	int i = 0;
	int a[1002];
	ucRxBuffer1[ucRxCnt1++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer1[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt1=0;
		return;
	}
	if (ucRxCnt1<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer1[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime1,&ucRxBuffer1[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc1,&ucRxBuffer1[2],8);break;
			case 0x52:	memcpy(&stcGyro1,&ucRxBuffer1[2],8);break;
			case 0x53:	memcpy(&stcAngle1,&ucRxBuffer1[2],8);break;
			case 0x54:	memcpy(&stcMag1,&ucRxBuffer1[2],8);break;
			case 0x55:	memcpy(&stcDStatus1,&ucRxBuffer1[2],8);break;
			case 0x56:	memcpy(&stcPress1,&ucRxBuffer1[2],8);break;
			case 0x57:	memcpy(&stcLonLat1,&ucRxBuffer1[2],8);break;
			case 0x58:	memcpy(&stcGPSV1,&ucRxBuffer1[2],8);break;
			case 0x59:	memcpy(&stcQ1,&ucRxBuffer1[2],8);break;
		}
		ucRxCnt1=0;//��ջ�����
	}

}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
}

int main(void)
{  		
	unsigned char i = 0;
	SysTick_init(168,10);//����ʱ��Ƶ��
//	Initial_UART1(115200);//��PC�Ĵ���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	uart_init(115200);
	Initial_UART2(115200);//��JY-901ģ��Ĵ���	
	uart3_init(115200);
	LCD_Init();
	while(1)
	{			
//		delay_ms(1000);
//		i++;
//		if(i>20)
//		{
//			i = 0;
//			printf("���ڽ��м��ٶ�У׼\r\n");
//			sendcmd(ACCCALSW);delay_ms(100);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
//			sendcmd(SAVACALSW);delay_ms(100);//���浱ǰ����
//		  printf("���ٶ�У׼���\r\n");
//		}

//		//������ٶ�
//		//���ڽ��ܵ��������Ѿ���������Ӧ�Ľṹ��ı������ˣ�����˵�����Э�飬�Լ��ٶ�Ϊ�� stcAcc.a[0]/32768*16����X��ļ��ٶȣ�
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//			delay_ms(10);
//		//������ٶ�
//		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//			delay_ms(10);
		//����Ƕ�
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
			delay_ms(10);
//		printf("Angle1:%.3f %.3f %.3f\r\n",(float)stcAngle1.Angle[0]/32768*180,(float)stcAngle1.Angle[1]/32768*180,(float)stcAngle1.Angle[2]/32768*180);
//			delay_ms(10);	
		
		LCD_ShowString(30,300,210,24,24,"X Y Z Angle:");	
		LCD_ShowNum(100,330,(float)stcAngle1.Angle[0]/32768*180,8,24);	
		LCD_ShowNum(100,360,(float)stcAngle1.Angle[1]/32768*180,8,24);				
		LCD_ShowNum(100,390,(float)stcAngle1.Angle[2]/32768*180,8,24);				
		delay_ms(10);
		
		
		
	}//��ѭ��
}



