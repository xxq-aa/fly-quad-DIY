#include "main.h"
#include "joy.h"

uint16_t adc_value[4] = {0};
Joystick_TypeDef js;
Joystick_TypeDef js1;
void JOY(void)
{
    /* ========== 1. 原始映射到 -100 ~ +100 ========== */
    js1.x  = (adc_value[0] - 1956) * 200 / 4025;
    js1.y  = (4025 - adc_value[1] - 2035) * 200 / 4025;
    js1.zx = (adc_value[2] - 2035) * 200 / 4026;
    js1.zy = (4026 - adc_value[3] - 2004) * 200 / 4026;

    /* ========== 2. ★ 方向修正（反的轴加负号）========== */
    js1.x  = -js1.x;
    js1.y  = -js1.y;
    js1.zx = -js1.zx;
    js1.zy = -js1.zy;

    /* ========== 3. ★ 零偏修正（你实测的中位偏差）========== */
    js1.zx -= 4;    /* zx 中位偏 +4，减掉 */
    js1.y  -= 2;    /* y 中位偏 +2，减掉 */

    /* ========== 4. 死区 ========== */
    if(js1.x  < 5 && js1.x  > -5) js1.x  = 0;
    if(js1.y  < 5 && js1.y  > -5) js1.y  = 0;
    if(js1.zx < 5 && js1.zx > -5) js1.zx = 0;
    if(js1.zy < 5 && js1.zy > -5) js1.zy = 0;

    /* ========== 5. 加 100 偏移变成 0~200（发送用）========== */
    js.x  = js1.x  + 103;
    js.y  = js1.y  + 100;
    js.zx = js1.zx + 103;
    js.zy = js1.zy + 100;
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
