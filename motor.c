#include "main.h"
#include "gpio.h"
#include "tim.h"
void Left1_Speed(int16_t Speed)
{

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Speed);



}
void Left2_Speed(int16_t Speed)
{

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Speed);



}
void Right1_Speed(int16_t Speed)
{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Speed);



}
void Right2_Speed(int16_t Speed)
{

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Speed);



}
void Stop()
{
	               Left1_Speed(0);
	        	   Left2_Speed(0);
	        	   Right1_Speed(0);
	        	   Right2_Speed(0);
}


