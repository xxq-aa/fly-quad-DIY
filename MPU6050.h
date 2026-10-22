#ifndef __MPU6050_H
#define __MPU6050_H




void MPU6050_Init(I2C_HandleTypeDef*hi2c);
int16_t MPU6050_ReadAll(I2C_HandleTypeDef*hi2c,int16_t* ax, int16_t* ay, int16_t* az,
        int16_t* gx, int16_t* gy, int16_t* gz);

uint8_t MPU6050_ReadDeviceID(I2C_HandleTypeDef*hi2c);

int16_t MPU6050_ReadData(I2C_HandleTypeDef*hi2c,uint8_t reg);

void MPU6050_ReadAccel(I2C_HandleTypeDef*hi2c,int16_t *ax,int16_t *ay,int16_t *az);

void MPU6050_ReadGyro(I2C_HandleTypeDef*hi2c,int16_t *gx,int16_t *gy,int16_t *gz);




#endif
