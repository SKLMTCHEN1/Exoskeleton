/*
编写者：Kevin
网址：http://RobotControl.taobao.com
作者E-mail：1609370741@qq.com
编译环境：MDK-Lite  Version: 5.17
初版时间: 2016-1-31
测试： 本程序已在【君悦智控】的STM32Core平台上完成测试
功能：
用STM32Core平台串口2读取JY901的数据，然后通过串口1打印到串口助手,串口助手波特率要选为9600。
JY-901的波特率要修改为9600.
注意：示例程序输出的是ASCLL码，用16进制（HEX）显示是不能看到准确数据的。
硬件接线：
USB-TTL工具                 STM32Core               JY901
VCC          -----           VCC        ----         VCC
TX           -----           RX1（管脚10）     
RX           -----           TX1（管脚9）
GND          -----           GND        ----          GND
                             RX2 （管脚3）       ----  TX
														 TX2 （管脚2）       ----  RX
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

//char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
//char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

////用串口2给JY模块发送指令
//void sendcmd(char cmd[])
//{
//	char i;
//	for(i=0;i<5;i++)
//		UART2_Put_Char(cmd[i]);
//}


//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	int begintime,endtime;
	int i = 0;
	int a[1002];
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区
	}
}
void CopeSerial3Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer1[250];
	static unsigned char ucRxCnt1 = 0;	
	int i = 0;
	int a[1002];
	ucRxBuffer1[ucRxCnt1++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer1[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt1=0;
		return;
	}
	if (ucRxCnt1<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer1[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime1,&ucRxBuffer1[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt1=0;//清空缓存区
	}

}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}

int main(void)
{  		
	unsigned char i = 0;
	SysTick_init(168,10);//设置时钟频率
//	Initial_UART1(115200);//接PC的串口
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	uart_init(115200);
	Initial_UART2(115200);//接JY-901模块的串口	
	uart3_init(115200);
	LCD_Init();
	while(1)
	{			
//		delay_ms(1000);
//		i++;
//		if(i>20)
//		{
//			i = 0;
//			printf("正在进行加速度校准\r\n");
//			sendcmd(ACCCALSW);delay_ms(100);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
//			sendcmd(SAVACALSW);delay_ms(100);//保存当前配置
//		  printf("加速度校准完成\r\n");
//		}

//		//输出加速度
//		//串口接受到的数据已经拷贝到对应的结构体的变量中了，根据说明书的协议，以加速度为例 stcAcc.a[0]/32768*16就是X轴的加速度，
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//			delay_ms(10);
//		//输出角速度
//		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//			delay_ms(10);
		//输出角度
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
			delay_ms(10);
//		printf("Angle1:%.3f %.3f %.3f\r\n",(float)stcAngle1.Angle[0]/32768*180,(float)stcAngle1.Angle[1]/32768*180,(float)stcAngle1.Angle[2]/32768*180);
//			delay_ms(10);	
		
		LCD_ShowString(30,300,210,24,24,"X Y Z Angle:");	
		LCD_ShowNum(100,330,(float)stcAngle1.Angle[0]/32768*180,8,24);	
		LCD_ShowNum(100,360,(float)stcAngle1.Angle[1]/32768*180,8,24);				
		LCD_ShowNum(100,390,(float)stcAngle1.Angle[2]/32768*180,8,24);				
		delay_ms(10);
		
		
		
	}//主循环
}



