#include "gpio.h"
#include "i2c.h"
#include "main.h"

#define MPU6050_ADDRESS      0xD0
#define ACCEL_XOUT_H         0x3B
#define PWR_MGMT_1			0x6B
#define GYRO_XOUT_H			0x43
#define MPU6050_WHO_AM_I_REG 0x75


void MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    uint8_t init_data[] = {
        PWR_MGMT_1, 0x00,      // 唤醒设备
        0x19, 0x04,            // 采样率分频
		0x1A, 0x03,            // DLPF配置
        0x1B, 0x18,            // 陀螺仪量程
        0x1C, 0x18             // 加速度计量程
    };

    // 使用连续写入方式
    for (int i = 0; i < sizeof(init_data); i += 2) {
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, init_data[i],I2C_MEMADD_SIZE_8BIT, &init_data[i+1], 1, 100);
    }


}




uint8_t MPU6050_ReadDeviceID(I2C_HandleTypeDef*hi2c)
{
	uint8_t device_id;
	HAL_I2C_Mem_Read(hi2c,MPU6050_ADDRESS,MPU6050_WHO_AM_I_REG,1,&device_id,1,HAL_MAX_DELAY);
	return device_id;
}

int16_t MPU6050_ReadAll(I2C_HandleTypeDef* hi2c,
                           int16_t* ax, int16_t* ay, int16_t* az,
                           int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buffer[14];

   /*if (HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDRESS, 1, 1) != HAL_OK) {
         return -1;  // 设备无响应，直接失败
     }*/
    // 减小超时时间到1ms（1000个tick）
    if(HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, buffer, 14, 1) != HAL_OK)
    {
        return -1;
    }

    // 快速处理数据（使用指针运算）
    *ax = ((int16_t)buffer[0] << 8) | buffer[1];
    *ay = ((int16_t)buffer[2] << 8) | buffer[3];
    *az = ((int16_t)buffer[4] << 8) | buffer[5];
    *gx = ((int16_t)buffer[8] << 8) | buffer[9];
    *gy = ((int16_t)buffer[10] << 8) | buffer[11];
    *gz = ((int16_t)buffer[12] << 8) | buffer[13];

    return 0;
}
void MPU6050_ReadAccel(I2C_HandleTypeDef*hi2c,int16_t *ax,int16_t *ay,int16_t *az)
{
	 int16_t gx, gy, gz; // 占位符
	 MPU6050_ReadAll(hi2c, ax, ay, az, &gx, &gy, &gz);
}
void MPU6050_ReadGyro(I2C_HandleTypeDef*hi2c,int16_t *gx,int16_t *gy,int16_t *gz)
{
	 int16_t ax, ay, az; // 占位符
	    MPU6050_ReadAll(hi2c, &ax, &ay, &az, gx, gy, gz);

}




