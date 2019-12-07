#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
//#include "key.h"  
#include "can.h"
#include "adc.h"
#include "string.h"
//#include "myiic.h" 
//#include "AHRSREG.h"
#include "math.h" 
//#include "arm_math.h" 

#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "usart.h"
#include "usart2.h"
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

short temp_pwm = 0;
//char DriverFeedback[10000];//太大的数组只能设计为全局变量  反馈信息存储数组
long Temp_Position = 0;
int pause = 1;
//short temp_current=0;
short temp_current = -1000;
int pause2 = 1;
short temp_current2 = -1000;

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//u16 adcx5;
u16 adcx7;
u16 adcx6;

//short CharToShort(unsigned char cData[])
//{
//	return ((short)cData[1]<<8)|cData[0];
//}

int main(void)
{  		
	u32 total,free;
	u8 t=0;	
	u8 res=0;	
	double Angle1[5],Angle[5],w[5];
	unsigned char chrTemp[32];
//	u16 adcx1;
//	u16 adcx2;
	u8 key;
//	u16 i=0;
//	u8 datatemp[SIZE];	
	SystemInit();
//	unsigned char i = 0;
	SysTick_init(168,10);//设置时钟频率
	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz 
//	Initial_UART1(115200);//接PC的串口
	uart_init(115200);
	uart2_init(115200);//接JY-901模块的串口	
	uart3_init(115200);
	LCD_Init();
	LED_Init();					//初始化LED 
//	 KEY_Init();					//按键初始化 
	Adc_Init(); 				//初始化ADC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps  
	
    delay_ms(500);                                      //刚开始要有足够的延时，确保驱动器已经初始化完成 
    CAN_RoboModule_DRV_Reset(0,0);                      //对0组所有驱动器进行复位 
    delay_ms(500);                                      //发送复位指令后的延时必须要有，等待驱动器再次初始化完成
   
    CAN_RoboModule_DRV_Mode_Choice(0,0,Current_Mode);  //0组的所有驱动器 都进入current模式
    delay_ms(500);                                      //发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
		// int history_position = 0;           //相对位置是否用得到？
	
    CAN_RoboModule_DRV_Config(0,1,50,0);               //1号驱动器配置为50ms传回一次电流速度位置数据
    delay_us(200);                                      //此处延时为了不让传回数据时候4个不一起传
    CAN_RoboModule_DRV_Config(0,2,50,0);               //2号驱动器配置为50ms传回一次电流速度位置数据
	
		  int n = 0;
	    temp_pwm = 2000;	
	/*************************************************************************
                      CTC: Define data for ANN recognize
*************************************************************************/
double maxI[6]={71.5500000000000,28.1800000000000,56.0900000000000,72,324.360000000000,297.026666666667};
double minI[6]={-77.4500000000000,-17.3600000000000,-25.8700000000000,-3.19000000000000,59.6933333333333,77.0266666666667};
float IWT[6][10]={{0.1395, 1.0379,-1.3604, 2.4140,-0.9580, 1.5454, 1.7737, 0.6487,-0.7336,-0.3763},
                  {-0.9679,-4.4419,-2.2720, 0.9888,-2.1144, 2.0687, 2.3875, 0.5981, 1.2985,-2.2708},
                  {2.8112,-1.8680,-2.2001,-0.6924,-2.0899,-1.1428,-1.3301,-3.9819,-4.3603, 4.2811},
                  {3.8932,-1.1728,-3.5339,-3.8035,-1.4623, 3.4080,-3.3968,-2.5376,-2.9899, 1.8201},
                  {1.0565, 0.5195,-1.1218, 2.5971, 0.2237, 0.2165, 1.3366, 0.1046,-0.9438,-3.4244},
                  {1.7877, 6.7357,-0.8970, 2.5617, 3.9784,-1.8455, 1.7373,-0.0945,-0.8018,-3.2426}};								 								 
float LW[7][10]={{ -0.2470,-0.0621,-0.4015,-0.4813,0.1754,-0.3696,0.2479,1.0110,-0.3584,-0.1144},
                 {0.2654,-1.2029,0.4542,1.0646,0.6157,0.4147,-0.8373,0.6774,0.0679,-0.3631},
                 {-0.5216,1.1897,-0.6011,0.0837,-0.5651,-0.5099,-0.6243,0.0403,-0.2972,-0.8677},
                 {1.3160,0.0064,0.9722,-0.0310,-0.3232,-0.4256,0.8248,-0.0183,-0.2530,1.4656},
                 {-1.4301,0.0082,-1.2553,0.0329,0.2565,0.1660,-0.2756,-0.1053,0.3011,0.0443},
                 {0.9566,-0.1809,1.2947,-0.4026,0.0033,1.1046,0.2953,0.8415,-0.8562,-0.3425},
                 {-0.3389,0.2416,-0.4628,-0.2662,-0.1626,-0.3799,0.3693,-2.4467,1.3958,0.1781}};
float b1[10][1]={{-4.3330},
                 {-3.2303},
                 {3.4873},
                 {0.0288},
                 {-1.6239},
                 {-3.4608},
                 {0.7518},
                 {-6.3825},
                 {-4.7032},
                 {-4.6753}};
float b2[7][1]={{0.5248},
                {-0.0939},
                {1.4998},
                {-1.4536},
                {1.2311},
                {-0.9962},
                {0.2874}};
								
int hidden_length=10;
int out_length=7;
				float hipAngVelo_raw=0;
				float hipAngle_raw=0;
				float kneeAngle_raw=0;
		  	float kneeAngleRelative_raw=0;
		   	float pressureback_raw=0;
		  	float pressurefront_raw=0;			
//CTC：After Normalization							
				float hipAngVelo=0;
				float hipAngle=0;
				float kneeAngle=0;
		  	float kneeAngleRelative=0;
		   	float pressureback=0;
		  	float pressurefront=0;					
			 
			float hiddenLayer[1][10]={0};
			float outLayer[1][7]={0};
			float outLayer1[1][10]={0};
		  float inputkT[1][6];
			float temp[1][10]={0};
			float sum[10]={0};
			float sumoutlayer[1][7]={0};
			float max = 0;
			int Index = 0; 
		  
/*************************************************************************
                      CTC：End define data for ANN recognize
*************************************************************************/
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
//		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
//			delay_ms(10);
//		printf("Angle1:%.3f %.3f %.3f\r\n",(float)stcAngle1.Angle[0]/32768*180,(float)stcAngle1.Angle[1]/32768*180,(float)stcAngle1.Angle[2]/32768*180);
//			delay_ms(10);	
		
		
/*************************************************************************
                      CTC：Motor feedback acquisition
*************************************************************************/
//		LED0=!LED0;

			    CanRxMsg rx_message;
    
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {
            if(rx_message.StdId == 0x1B)
            {
                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            LCD_ShowNum(100,100,Real_Position_Value[0],8,16);		
						}
            else if(rx_message.StdId == 0x2B)
            {
                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            LCD_ShowNum(200,200,Real_Position_Value[1],8,16);	
						}
            else if(rx_message.StdId == 0x3B)
            {
                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x4B)
            {
                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x1F)
            {
                Real_Online[0] = 1;
            }
            else if(rx_message.StdId == 0x2F)
            {
                Real_Online[1] = 1;
            }
            else if(rx_message.StdId == 0x3F)
            {
                Real_Online[2] = 1;
            }
            else if(rx_message.StdId == 0x4F)
            {
                Real_Online[3] = 1;
            }
            else if(rx_message.StdId == 0x1C)
            {
                Real_Ctl1_Value[0] = rx_message.Data[0];
                Real_Ctl2_Value[0] = rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x2C)
            {
                Real_Ctl1_Value[1] = rx_message.Data[0];
                Real_Ctl2_Value[1] = rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x3C)
            {
                Real_Ctl1_Value[2] = rx_message.Data[0];
                Real_Ctl2_Value[2] = rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x4C)
            {
                Real_Ctl1_Value[3] = rx_message.Data[0];
                Real_Ctl2_Value[3] = rx_message.Data[1];
            }
        }
		
/*************************************************************************
                      CTC：End of Motor feedback acquisition
                      CTC：Input IMU & Pressure Parameters
*************************************************************************/	

			adcx7=((4096-Get_Adc_Average(ADC_CH7,20))-40)*40;			
//	  	LCD_ShowxNum(134,130,adcx7,4,16,0);//??ADC??			
//		printf("adcx5 is %d\n",adcx5);
//		//adcx5=4096-adcx5;
//		LCD_ShowxNum(134,130,adcx5,4,16,0);//??ADC??
//		//temp=(float)adcx5*3.3/4096; 
      delay_ms(5);
//		
			adcx6=((4096-Get_Adc_Average(ADC_CH6,20))-40)*40;				
//	   	LCD_ShowxNum(134,530,adcx6,4,16,0);//??ADC??
//		adcx6=((4096-Get_Adc_Average(ADC_CH6,20))-40)*40;
//		//adcx6=4096-adcx6;
//		printf("adcx6 is %d\n",adcx6);
//		LCD_ShowxNum(134,530,adcx6,4,16,0);//??ADC??
//		//temp=(float)adcx6*3.3/4096; 
		//printf("temp is %f\n",temp);
		//adcx=temp;
		//LCD_ShowxNum(134,150,adcx,1,16,0);//?????
		//temp-=adcx;
		//temp*=1000;
		//printf("temp is %f\n",temp);
		//LCD_ShowxNum(150,150,temp,3,16,0X80);
								
//				if ( Angle[2]<-30)
//				{
//					LCD_ShowNum(100,400,Angle[2],8,16);	
//					temp_current = -2000;
//				}
//				if ( Angle[2]>50)
//				{
//					LCD_ShowNum(100,400,Angle[2],8,16);	
//					temp_current = 2000;
//				}				
//						
		
/*************************************************************************
                  CTC：End of Input IMU & Pressure Parameters			
                  CTC：Transfer Parameters to values
*************************************************************************/	
				 hipAngVelo_raw=(float)stcGyro.w[1]/32768*2000;//CHANGE!!!!
				 hipAngle_raw=(float)stcAngle.Angle[1]/32768*180;//CHANGE!!!!
				 kneeAngle_raw= (float)stcAngle1.Angle[1]/32768*180;//CHANGE!!!!
		  	 kneeAngleRelative_raw=((float)stcAngle.Angle[1]/32768*180)+((float)stcAngle1.Angle[1]/32768*180);//CHANGE!!!!
		   	 pressureback_raw=adcx7;//CHANGE!!!!
		  	 pressurefront_raw=adcx6;//CHANGE!!!!
				 
//		printf("hipAngVelo_raw:%.3f\r\n",hipAngVelo_raw);
//			delay_ms(10);				 
//		printf("hipAngle_raw:%.3f\r\n",hipAngle_raw);		
//			delay_ms(10);		
//		printf("%.3f\r\n",(float)stcAngle.Angle[1]/32768*180);		
		printf("kneeAngle_raw:%.3f\r\n",hipAngle_raw);		
//			delay_ms(10);				 
//		printf("kneeAngleRelative_raw:%.3f\r\n",kneeAngleRelative_raw);	
				 
/*************************************************************************				 
                 CTC：End of Transfer Parameters to values
                 CTC：Normalization of input parameters
*************************************************************************/	
				 hipAngVelo = (hipAngVelo_raw-minI[0])/(maxI[0]-minI[0]);
         hipAngle = (hipAngle_raw-minI[1])/(maxI[1]-minI[1]);
         kneeAngle = (kneeAngle_raw-minI[2])/(maxI[2]-minI[2]);				
         kneeAngleRelative = (kneeAngleRelative_raw-minI[3])/(maxI[3]-minI[3]);				
         pressureback = (pressureback_raw-minI[4])/(maxI[4]-minI[4]);				
         pressurefront = (pressurefront_raw-minI[5])/(maxI[5]-minI[5]);					 
				 
				 inputkT[0][0]=hipAngVelo;
		     inputkT[0][1]=hipAngle;
		     inputkT[0][2]=kneeAngle;
         inputkT[0][3]=kneeAngleRelative;
         inputkT[0][4]=pressureback;
         inputkT[0][5]=pressurefront;
				 
/*************************************************************************						 
                 CTC：End of Normalization of input parameters				 
                 CTC：ANN substitution processing
*************************************************************************/	
 /********************************************
 Matrix multiplication
*@para:A:矩阵A；B:矩阵B；C:相乘结果矩阵；rowA：A的行数；columnB：B的列数；columnA：A的列数
*@ret:void 
********************************************/
    for (int i=0;i<1;i++)
		{
        for (int j=0; j<10;j++)
			{
            temp[i][j] = 0;
            for (int k=0;k<6;k++)
				{
                temp[i][j]+=inputkT[i][k]*IWT[k][j];     //    temp=inputk'*IW';
            }
         }
     }
		 
		 for(int h=0;h<10;h++)
		 {
     hiddenLayer[0][h] = 1 / (1 + exp(-(temp[0][h]+b1[h][0])));   
		 }
    
 //above fouctions is according:       for i= 1:hidden_length[1]    hiddenLayer(i) = logsig(temp(i)+b1(i));

		for(int f=0;f<7;f++) 
	   {         
			 	for(int r=0;r<10;r++) 
				{
						outLayer1[0][r]= hiddenLayer[0][r]*LW[f][r];
				}	 
				for(int q=0;q<10;q++)
				{
						sumoutlayer[0][f]+= outLayer1[0][q];
				}
					outLayer[0][f] = sumoutlayer[0][f] + b2[f][0];
	   }
 //above fouctions is according:     for i= 1:out_length[1] outLayer(i)=purelin(sum(hiddenLayer.*LW(i,:))+b2(i))
				
			for (int e=0;e<6;e++) 
        {					
					if (outLayer[0][e]>outLayer[0][e+1]&&outLayer[0][e]>max)
					{
						  max=outLayer[0][e];
				       	Index=e+1;
					}
					else if (outLayer[0][e]<=outLayer[0][e+1]&&max<=outLayer[0][e+1])
					{
						  max=outLayer[0][e+1];
				    	Index=e+1+1;						
					}
					else 
					{
						max=max;
						Index=Index;
					}
				}
 //above fouctions is according:    [m , Index] = max(outLayer);
/*************************************************************************						 
                 CTC：End of ANN substitution processing	
                     The Index is the gait phase (1-7)		
			           CTC：The motor control are as follows:
*************************************************************************/			 		 
		if (Index==1||Index==2)	
		{
			temp_current = 1500;
			temp_current2 = 2500;			//the other leg is in index=4
		}
		else if (Index==3)
		{
			temp_current = -1500;
			temp_current2 = -2500;			//the other leg is in index=6     hip-->Netural knee-->back
		}
		else if (Index==4)	
		{
			temp_current = 2500;		
			temp_current2 = -2500;			//the other leg is in index=7
		}			
		else if (Index==5)	
		{
			temp_current = -1500;			
			temp_current2 = -1500;			//the other leg is in index=3			
		}			
		else if (Index==6||Index==7)					
			temp_current = -2500;						
			temp_current2 = 2500;			//the other leg is in index=4					
				
			
//				if (Index==1||Index==2)	
//		{
//			temp_current = 1500;
//			temp_current2 = -2000;			//the other leg is in index=4
//		}
//		else if (Index==3)
//		{
//			temp_current = -1500;
//			temp_current2 = -2000;			//the other leg is in index=6  hip-->Netural knee-->back
//		
//		}
//		else if (Index==4)	
//		{
//			temp_current = 2500;		
//				
//		}			
//		else if (Index==5)	
//		{
//			temp_current = -1500;			
//		}			
//		else if (Index==6||Index==7)					
//			temp_current = -2500;					
				
				
				
//        if (Index==1 && Real_Position_Value[0]<-70000)
//				{
//					if ( Real_Position_Value[1]<-70000)
//					{
//					Index = 2;
//					temp_current = 1500;
//					temp_current2 = 1500;
//					}
//					else
//					{					
//				 	 Index = 2;
//				   temp_current = 1500;
//					}					
//				}
//			  else if (Index==2 && Real_Position_Value[0]>-20000)
//				{
//					if (Real_Position_Value[1]>-20000)
//					{
//					Index = 1;
//					temp_current = -1500;
//					temp_current2 = -1500;
//					}
//					else
//					{
//					Index = 1;
//					temp_current = -1500;
//					}
//				}		
				
			  CAN_RoboModule_DRV_Current_Mode(0,1,temp_pwm,temp_current);
				CAN_RoboModule_DRV_Current_Mode(0,2,temp_pwm,temp_current2);				
				
//        n += 1;
//        if(n > 99)
//        {
//				    n=0;
//        // break;
//        } 				
				

		
		
		
		
		
	}//主循环
}



