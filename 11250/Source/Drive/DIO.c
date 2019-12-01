#include "DIO.h"

#define LED_GPIO_CLK   RCC_AHB1Periph_GPIOF 
#define LED_PORT   	   GPIOF
#define LED_PIN        GPIO_Pin_10

void DIO_Initial(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE);  
	
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin;
	GPIO_Init(GPIOx, &GPIO_InitStructure);	
}	

void IOSleep()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE);  
	
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_All;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	
	//GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	WriteDO(GPIOB,GPIO_Pin_8,0);//EN
//	WriteDO(GPIOB,GPIO_Pin_6,1);//??BRTS*
	
	WriteDO(GPIOB,GPIO_Pin_7,0);//EN
	WriteDO(GPIOB,GPIO_Pin_6,1);//??BRTS*
	
}	

void WriteDO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin,unsigned char ucData)
{
	if (ucData>0)
		GPIOx->BSRRL = GPIO_Pin;
	else
		GPIOx->BSRRH = GPIO_Pin;
}
unsigned char ReadDI(GPIO_TypeDef * GPIOx,uint32_t GPIO_Pin)
{
	return (GPIOx->IDR&GPIO_Pin)==GPIO_Pin;
}

/****************************************************
函数功能：LED初始化
输入参数：无
输出参数：无
备    注：调用此函数前，需要在LED.h修改宏定义LED引脚
****************************************************/
unsigned char ucLEDInitial=0;
void LED_Init(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能EXTI
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	RCC_AHB1PeriphClockCmd(LED_GPIO_CLK, ENABLE);
	DIO_Initial(LED_PORT,LED_PIN,GPIO_Mode_OUT,GPIO_Speed_2MHz);
	ucLEDInitial=1;
}



//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	RCC_AHBPeriphClockCmd(LED_GPIO_CLK, ENABLE);
//	DIO_Initial(LED_PORT,LED_PIN,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);
//	ucLEDInitial=1;
//}









/****************************************************
函数功能：LED开
输入参数：无
输出参数：无
****************************************************/
void LED_ON(void)
{
	if (ucLEDInitial==0) LED_Init();
	GPIO_SetBits(LED_PORT, LED_PIN);
	
}

/****************************************************
函数功能：LED关
输入参数：无
输出参数：无
****************************************************/
void LED_OFF(void)
{
	if (ucLEDInitial==0) LED_Init();
	GPIO_ResetBits(LED_PORT, LED_PIN);
}

void LED_REVERSE(void)
{
	if (ucLEDInitial==0) LED_Init();
	if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN))	
		GPIO_ResetBits(LED_PORT, LED_PIN);
	else
		GPIO_SetBits(LED_PORT, LED_PIN);
}
