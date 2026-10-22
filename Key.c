#include "main.h"                  // Device header
#include "gpio.h"

void Key_Init(void)
{
	 MX_GPIO_Init();
	/*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	*/
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
	{
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
		HAL_Delay(20);
		KeyNum = 3;
	}
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
	{
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
		HAL_Delay(20);
		KeyNum = 4;
	}
	return KeyNum;
}
