#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>
#include "lcd.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

unsigned int CAN_Time_Out = 0;
static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}




unsigned char can_tx_success_flag = 0;
/*************************************************************************
                          CAN1_TX_IRQHandler
������CAN1�ķ����жϺ���
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}


/****************************************************************************************
                                       ��λָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     ģʽѡ��ָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04         //��������
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Mode;    
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   �ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15)) 
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  �ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  �����ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  ����λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  �����ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ����ָ��
Temp_Time1��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Temp_Time2��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�ر���λ�źŷ�������
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;
    
    tx_message.Data[0] = Temp_Time1;
    tx_message.Data[1] = Temp_Time2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ���߼��
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern  long Real_Position_Value[4];
extern  char Real_Online[4];
extern  char Real_Ctl1_Value[4];
extern  char Real_Ctl2_Value[4];

//���������ݵĺ�����Ĭ��Ϊ4����������������0�飬���Ϊ1��2��3��4
/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
        {
            if(rx_message.StdId == 0x1B)
            {
                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            LCD_ShowNum(100,100,Real_Position_Value[0],8,16);		

						}
            else if(rx_message.StdId == 0x2B)
            {
                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            LCD_ShowNum(200,200,Real_Position_Value[1],8,16);	
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
                
    }
}



/********************************************
 Matrix multiplication
*@para:A:����A��B:����B��C:��˽������rowA��A��������columnB��B��������columnA��A������
*@ret:void 
********************************************/
//float matrixMul(float **A, float **B, int rowA, int columnB, int columnA){
//	float C[rowA][columnB];
//    for (int i=0;i<rowA;i++){
//        for (int j=0; j<columnB;j++){
//            C[i][j] = 0;
//            for (int k=0;k<columnA;k++){
//                C[i][j]+=A[i][k]*B[k][j];
//            }
//         }
//     }
//		return C[rowA][columnB];
//}


//float matrixMul(float **A, float **B,int rowA, int columnB, int columnA){
//	float C[rowA][columnB];
//    for (int i=0;i<rowA;i++){
//        for (int j=0; j<columnB;j++){
//            C[i][j] = 0;
//            for (int k=0;k<columnA;k++){
//                C[i][j]+=A[i][k]*B[k][j];
//            }
//         }
//     }
//		return **C;
//}


//void matrixMul(int **A, int **B,int **C){
//    for (int i=0;i<1;i++){
//        for (int j=0; j<10;j++){
//            C[i][j] = 0;
//            for (int k=0;k<6;k++){
//                C[i][j]+=A[i][k]*B[k][j];
//            }
//         }
//     }
//}

 /**************************************************
 Matrix Transposition
*@para:matrix:ԭ����row:����������column:��������
*@ret:����ת�þ���
**************************************************/
//int** getTransposeMatrix(int** matrix,int row,int column)
//	{
//   int** matrixR=new int*[column];
//   for(int i=0;i<column;++i){
//        matrixR[i]=new int[rows];
//   }

//   for(int i=0;i<row;++i){
//        for(int j=0;j<column;++j){
//            matrixR[j][i]=matrix[i][j];
//        }
//   }
//   return matrixR;
//} 









