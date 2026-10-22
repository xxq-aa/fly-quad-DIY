#include "main.h"
#include "joy.h"

uint16_t adc_value[4] = {0};
Joystick_TypeDef js;
void JOY(void)
{
    js.x = (adc_value[0] - 1956) * 200/4025;  // PB0 (通道8)
     js.y = (4025-adc_value[1] - 2035)*200/4025;  // PB1 (通道9)
    js.zx = (adc_value[2] - 2035) * 200/4026;
     js.zy =(4026 - adc_value[3]-2004)*200/4026;
/*     js.x = (adc_value[0] - 1956) * 200/4025;  // PB0 (通道8)
      js.y = (4025-adc_value[1] - 2035)*200/4025;  // PB1 (通道9)
     js.zx = (adc_value[2] - 2035) * 200/4026;
      js.zy =(4026 - adc_value[3]-2004)*200/4026;*/
}
uint8_t JOY_Key_Read(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11) == GPIO_PIN_RESET)
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11) == GPIO_PIN_RESET);
		return 1;
	}
	return 0;

}
