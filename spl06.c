#include "SPL06.h"
#include <string.h>   // for memset

/* 静态变量：保存状态和系数 */
static SPL06_t spl06;

/* 内部通信函数（I2C） */
static HAL_StatusTypeDef SPL06_WriteReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static HAL_StatusTypeDef SPL06_ReadReg(uint8_t reg, uint8_t *pData) {
    return HAL_I2C_Mem_Read(&hi2c1, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, 100);
}

static HAL_StatusTypeDef SPL06_ReadRegs(uint8_t reg, uint8_t *pData, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, len, 100);
}

/* 符号扩展辅助函数 */
static int32_t sign_extend24(int32_t val) {
    if (val & 0x800000) val |= 0xFF000000;
    return val;
}
static int16_t sign_extend12(int16_t val) {
    if (val & 0x0800) val |= 0xF000;
    return val;
}
static int32_t sign_extend20(int32_t val) {
    if (val & 0x00080000) val |= 0xFFF00000;
    return val;
}

/* 读取校准系数 */
static void SPL06_ReadCalib(void) {
    uint8_t buf[18];
    SPL06_ReadRegs(SPL06_COEF_C0, buf, 18);

    // c0 (12-bit)
    int16_t c0 = (buf[0] << 4) | (buf[1] >> 4);
    spl06.calib.c0 = sign_extend12(c0);

    // c1 (12-bit)
    int16_t c1 = ((buf[1] & 0x0F) << 8) | buf[2];
    spl06.calib.c1 = sign_extend12(c1);

    // c00 (20-bit)
    int32_t c00 = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    spl06.calib.c00 = sign_extend20(c00);

    // c10 (20-bit)
    int32_t c10 = ((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];
    spl06.calib.c10 = sign_extend20(c10);

    // 16-bit coefficients
    spl06.calib.c01 = (buf[8] << 8) | buf[9];
    spl06.calib.c11 = (buf[10] << 8) | buf[11];
    spl06.calib.c20 = (buf[12] << 8) | buf[13];
    spl06.calib.c21 = (buf[14] << 8) | buf[15];
    spl06.calib.c30 = (buf[16] << 8) | buf[17];
}

/* 读取产品 ID 和修订 ID */
static uint8_t SPL06_ReadID(void) {
    uint8_t id;
    if (SPL06_ReadReg(SPL06_ID, &id) != HAL_OK) return 0;
    spl06.prod_id = (id >> 4) & 0x0F;
    spl06.rev_id  = id & 0x0F;
    return spl06.prod_id;
}

/* 获取缩放因子（根据过采样率） */
static float SPL06_GetScaleFactor(SPL06_Oversampling_t osr) {
    switch (osr) {
        case SAMPLING_NONE: return 524288.0f;
        case SAMPLING_X2:   return 1572864.0f;
        case SAMPLING_X4:   return 3670016.0f;
        case SAMPLING_X8:   return 7864320.0f;
        case SAMPLING_X16:  return 253952.0f;
        case SAMPLING_X32:  return 516096.0f;
        case SAMPLING_X64:  return 1040384.0f;
        case SAMPLING_X128: return 2088960.0f;
        default: return 0.0f;
    }
}

/* 初始化 */
uint8_t SPL06_Init(void) {
    HAL_Delay(100);

    // 读取 ID（仅用于调试，不强制校验）
    uint8_t prod = SPL06_ReadID();
    // 如果你想知道实际 ID，可以把它显示出来，但这里我们跳过检查
    // if (prod != SPL06_PRODID) return 1;   // 注释掉这一行

    // 读取校准系数
    SPL06_ReadCalib();

    // 配置高精度模式
   // SPL06_Config(MODE_BACKGROUND_BOTH, SAMPLING_X16, SAMPLING_NONE, RATE_8, RATE_4);
  //  SPL06_Config(MODE_BACKGROUND_BOTH, SAMPLING_X64, SAMPLING_NONE, RATE_2, RATE_4);
 //   SPL06_Config(MODE_BACKGROUND_BOTH, SAMPLING_X16, SAMPLING_NONE, RATE_16, RATE_4);
    SPL06_Config(MODE_BACKGROUND_BOTH, SAMPLING_X16, SAMPLING_NONE, RATE_16, RATE_16);
    uint8_t tmp_cfg_read;
    SPL06_ReadReg(SPL06_TMP_CFG, &tmp_cfg_read);
    if((tmp_cfg_read & 0x80) == 0)
    {
        // TMP_EXT 没生效，重写
        uint8_t tmp_cfg = (1 << 7) | (RATE_16 << 4) | SAMPLING_NONE;
        SPL06_WriteReg(SPL06_TMP_CFG, tmp_cfg);
        HAL_Delay(10);
    }
    return 0; // 总是成功
}

/* 软件复位 */
void SPL06_Reset(void) {
    SPL06_WriteReg(SPL06_SOFT_RESET, 0x09);
    HAL_Delay(50);
}

/* 配置函数（修正了 shift 判断） */
void SPL06_Config(SPL06_Mode_t mode,
                  SPL06_Oversampling_t press_osr,
                  SPL06_Oversampling_t temp_osr,
                  SPL06_Rate_t press_rate,
                  SPL06_Rate_t temp_rate) {
    // 保存配置供后续使用
    spl06.press_osr = press_osr;
    spl06.temp_osr  = temp_osr;
    spl06.press_rate = press_rate;
    spl06.temp_rate  = temp_rate;

    // 判断是否需要右移（过采样 > 8 倍）
    bool press_shift = (press_osr > SAMPLING_X8);
    bool temp_shift  = (temp_osr  > SAMPLING_X8);
    spl06.shift_enable = press_shift || temp_shift; // 只要任一需要，就设置（其实两个寄存器位独立）

    // 构建 CFG_REG (0x09)
    // 位7: INT_HL (0, active low)
    // 位6: INT_FIFO (0)
    // 位5: INT_PRS (0)
    // 位4: INT_TMP (0)
    // 位3: T_SHIFT (temp_shift)
    // 位2: P_SHIFT (press_shift)
    // 位1: FIFO_EN (0)
    // 位0: SPI3W_EN (0)
    uint8_t cfg = (temp_shift << 3) | (press_shift << 2);
    SPL06_WriteReg(SPL06_CFG_REG, cfg);

    // 构建 PRS_CFG (0x06)
    // 位7: 保留0
    // 位6-4: PM_RATE (press_rate)
    // 位3-0: PM_PRC (press_osr)
    uint8_t prs_cfg = (press_rate << 4) | press_osr;
    SPL06_WriteReg(SPL06_PSR_CFG, prs_cfg);

    // 构建 TMP_CFG (0x07)
    // 位7: TMP_EXT (1, 使用外部MEMS温度)
    // 位6-4: TMP_RATE (temp_rate)
    // 位3: 保留0
    // 位2-0: TMP_PRC (temp_osr)
    uint8_t tmp_cfg = (1 << 7) | (temp_rate << 4) | temp_osr;
    SPL06_WriteReg(SPL06_TMP_CFG, tmp_cfg);

    // 构建 MEAS_CFG (0x08)
    // 位7-4: 状态只读，写0
    // 位3-0: MEAS_CTRL (mode)
    SPL06_WriteReg(SPL06_MEAS_CFG, mode);
}

/* 读取原始温度（24位补码） */
static int32_t SPL06_ReadRawTemp(void) {
    uint8_t buf[3];
    SPL06_ReadRegs(SPL06_TMP_B2, buf, 3);
    int32_t raw = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    return sign_extend24(raw);
}

/* 读取原始压力（24位补码） */
static int32_t SPL06_ReadRawPress(void) {
    uint8_t buf[3];
    SPL06_ReadRegs(SPL06_PSR_B2, buf, 3);
    int32_t raw = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    return sign_extend24(raw);
}

/* 等待数据就绪（轮询 PRS_RDY 和 TMP_RDY） */
static void SPL06_WaitForData(void) {
    uint8_t meas;
    do {
        SPL06_ReadReg(SPL06_MEAS_CFG, &meas);
        // 等待 PRS_RDY (bit4) 置位
    } while (!(meas & 0x10));
}
/* 读取温度（补偿后，单位 ℃） */
float SPL06_ReadTemperature(void) {
    SPL06_WaitForData();

    int32_t rawT = SPL06_ReadRawTemp();
    float kT = SPL06_GetScaleFactor(spl06.temp_osr);
    float traw_sc = rawT / kT;
    float temp = spl06.calib.c0 * 0.5f + spl06.calib.c1 * traw_sc;
    return temp;
}

/* 读取压力（补偿后，单位 Pa） */
float SPL06_ReadPressure(void) {
    SPL06_WaitForData();

    int32_t rawP = SPL06_ReadRawPress();
    int32_t rawT = SPL06_ReadRawTemp();  // 需要同时读取温度用于补偿

    float kP = SPL06_GetScaleFactor(spl06.press_osr);
    float kT = SPL06_GetScaleFactor(spl06.temp_osr);

    float praw_sc = rawP / kP;
    float traw_sc = rawT / kT;

    float c00 = spl06.calib.c00;
    float c10 = spl06.calib.c10;
    float c01 = spl06.calib.c01;
    float c11 = spl06.calib.c11;
    float c20 = spl06.calib.c20;
    float c21 = spl06.calib.c21;
    float c30 = spl06.calib.c30;

    float press = c00
                + praw_sc * (c10 + praw_sc * (c20 + praw_sc * c30))
                + traw_sc * c01
                + traw_sc * praw_sc * (c11 + praw_sc * c21);
    return press;
}

/* 计算绝对海拔（单位米），需传入海平面气压（单位 Pa） */
float SPL06_ReadAltitude(float sea_level_pressure) {
    float p = SPL06_ReadPressure();
    if (p <= 0.0f) return -999.0f;
    float ratio = p / sea_level_pressure;
    if (ratio <= 0.0f) return -999.0f;
    return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}

/* 计算相对高度（相对于参考气压），单位米 */
float SPL06_ReadRelativeAltitude(float ref_pressure) {
    float p = SPL06_ReadPressure();
    if (p <= 0.0f || ref_pressure <= 0.0f) return -999.0f;
    float ratio1 = p / 101325.0f;
    float ratio2 = ref_pressure / 101325.0f;
    if (ratio1 <= 0.0f || ratio2 <= 0.0f) return -999.0f;
    float alt1 = 44330.0f * (1.0f - powf(ratio1, 1.0f / 5.255f));
    float alt2 = 44330.0f * (1.0f - powf(ratio2, 1.0f / 5.255f));
    return alt1 - alt2;
}
