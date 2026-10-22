#ifndef SPL06_H
#define SPL06_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>   // 用于 powf

/* I2C 句柄外部引用（由用户提供） */
extern I2C_HandleTypeDef hi2c1;

/* SPL06-001 I2C 地址（SDO 接地时为 0x76，接高为 0x77） */
#define SPL06_I2C_ADDR          (0x77 << 1)   // 7位地址左移1位

/* 寄存器地址 */
#define SPL06_PSR_B2            0x00
#define SPL06_PSR_B1            0x01
#define SPL06_PSR_B0            0x02
#define SPL06_TMP_B2            0x03
#define SPL06_TMP_B1            0x04
#define SPL06_TMP_B0            0x05
#define SPL06_PSR_CFG           0x06
#define SPL06_TMP_CFG           0x07
#define SPL06_MEAS_CFG          0x08
#define SPL06_CFG_REG           0x09
#define SPL06_INT_STS           0x0A
#define SPL06_FIFO_STS          0x0B
#define SPL06_SOFT_RESET        0x0C
#define SPL06_ID                0x0D
#define SPL06_COEF_C0           0x10
#define SPL06_COEF_C0C1         0x11
#define SPL06_COEF_C1           0x12
#define SPL06_COEF_C00a         0x13
#define SPL06_COEF_C00b         0x14
#define SPL06_COEF_C00C10       0x15
#define SPL06_COEF_C10a         0x16
#define SPL06_COEF_C10b         0x17
#define SPL06_COEF_C01a         0x18
#define SPL06_COEF_C01b         0x19
#define SPL06_COEF_C11a         0x1A
#define SPL06_COEF_C11b         0x1B
#define SPL06_COEF_C20a         0x1C
#define SPL06_COEF_C20b         0x1D
#define SPL06_COEF_C21a         0x1E
#define SPL06_COEF_C21b         0x1F
#define SPL06_COEF_C30a         0x20
#define SPL06_COEF_C30b         0x21

/* 期望的产品 ID（通常 0x10） */
#define SPL06_PRODID            0x10

/* 枚举定义 */
typedef enum {
    SAMPLING_NONE = 0x00,
    SAMPLING_X2   = 0x01,
    SAMPLING_X4   = 0x02,
    SAMPLING_X8   = 0x03,
    SAMPLING_X16  = 0x04,
    SAMPLING_X32  = 0x05,
    SAMPLING_X64  = 0x06,
    SAMPLING_X128 = 0x07
} SPL06_Oversampling_t;

typedef enum {
    MODE_STANDBY      = 0x00,
    MODE_FORCED_P     = 0x01,
    MODE_FORCED_T     = 0x02,
    MODE_BACKGROUND_P = 0x05,
    MODE_BACKGROUND_T = 0x06,
    MODE_BACKGROUND_BOTH = 0x07
} SPL06_Mode_t;

typedef enum {
    RATE_1   = 0x00,
    RATE_2   = 0x01,
    RATE_4   = 0x02,
    RATE_8   = 0x03,
    RATE_16  = 0x04,
    RATE_32  = 0x05,
    RATE_64  = 0x06,
    RATE_128 = 0x07
} SPL06_Rate_t;

/* 校准系数结构 */
typedef struct {
    int16_t c0;   // 12-bit
    int16_t c1;   // 12-bit
    int32_t c00;  // 20-bit
    int32_t c10;  // 20-bit
    int16_t c01;  // 16-bit
    int16_t c11;  // 16-bit
    int16_t c20;  // 16-bit
    int16_t c21;  // 16-bit
    int16_t c30;  // 16-bit
} SPL06_Calib_t;

/* 传感器状态结构 */
typedef struct {
    uint8_t prod_id;
    uint8_t rev_id;
    SPL06_Calib_t calib;
    SPL06_Oversampling_t press_osr;
    SPL06_Oversampling_t temp_osr;
    SPL06_Rate_t press_rate;
    SPL06_Rate_t temp_rate;
    bool shift_enable;   // 是否启用数据右移（过采样>8时启用）
} SPL06_t;

/* 对外接口函数 */
uint8_t SPL06_Init(void);                                // 初始化，返回 0 成功，非0失败
void SPL06_Reset(void);
void SPL06_Config(SPL06_Mode_t mode,
                  SPL06_Oversampling_t press_osr,
                  SPL06_Oversampling_t temp_osr,
                  SPL06_Rate_t press_rate,
                  SPL06_Rate_t temp_rate);
float SPL06_ReadTemperature(void);                       // 单位 ℃
float SPL06_ReadPressure(void);                          // 单位 Pa
float SPL06_ReadAltitude(float sea_level_pressure);      // 单位 m，传入海平面气压（Pa）
float SPL06_ReadRelativeAltitude(float ref_pressure);    // 相对高度，传入参考气压（Pa）

#endif
