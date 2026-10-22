#ifndef __JOY_H
#define __JOY_H
void JOY(void);
uint8_t JOY_Key_Read();

typedef struct {
    int16_t x;        // PB0 (ADC1_IN8) 对应 X轴
    int16_t y;         // PB1 (ADC1_IN9) 对应 Y轴
    int8_t k;         //右
    int8_t zk;         //左
    int16_t zx;
    int16_t zy;
} Joystick_TypeDef;
extern Joystick_TypeDef js;
extern Joystick_TypeDef js1;
extern uint16_t adc_value[4];
#endif
