#include "adc.h"
#include "delay.h"		 
//#include "delay.h"
//#include "sys1.h"  //zj��ӵ�
#include "sys.h"  //zj��ӵ�
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									   
////////////////////////////////////////////////////////////////////////////////// 


//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ADC1_CH5																	   
void  Adc_Init(void)
{    
	//�ȳ�ʼ��IO��
 	RCC->APB2ENR|=1<<8;    	//ʹ��ADC1ʱ�� 
	RCC->AHB1ENR|=1<<0;    	//ʹ��PORTAʱ��	  
	GPIO_Set(GPIOA,PIN7,GPIO_MODE_AIN,0,0,GPIO_PUPD_PU);	//PA5,ģ������,����   
	GPIO_Set(GPIOA,PIN6,GPIO_MODE_AIN,0,0,GPIO_PUPD_PU);	//PA5,ģ������,����   

	RCC->APB2RSTR|=1<<8;   	//ADCs��λ
	RCC->APB2RSTR&=~(1<<8);	//��λ����	 
	ADC->CCR=3<<16;			//ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
 	
	ADC1->CR1=0;   			//CR1��������
	ADC1->CR2=0;   			//CR2��������
	ADC1->CR1|=0<<24;      	//12λģʽ
	ADC1->CR1|=0<<8;    	//��ɨ��ģʽ	
	
	ADC1->CR2&=~(1<<1);    	//����ת��ģʽ
 	ADC1->CR2&=~(1<<11);   	//�Ҷ���	
	ADC1->CR2|=0<<28;    	//�������
	
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1|=0<<20;     	//1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��5�Ĳ���ʱ��
	ADC1->SMPR2&=~(7<<(3*5));//ͨ��5����ʱ�����	  
 	ADC1->SMPR2|=7<<(3*5); 	//ͨ��5  480������,��߲���ʱ�������߾�ȷ��	 
 	ADC1->CR2|=1<<0;	   	//����ADת����	  
}				  
//���ADCֵ
//ch:ͨ��ֵ 0~16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	//����ת������	  		 
	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<30;       //��������ת��ͨ�� 
	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
	return ADC1->DR;		//����adcֵ	
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
#define ADC_CH6 6
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
}  









