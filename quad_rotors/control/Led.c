#include "stm32f10x.h"                  // Device header

void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //TIM2时钟的ch1和ch3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
}

void Led_On(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}

void Led_Off(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
}
	