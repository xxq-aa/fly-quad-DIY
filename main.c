/*USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "joy.h"
#include "key.h"
#include "NRF24L01.h"
#include "motor.h"
#include "BMP280.h"
#include "math.h"
#include "OLED.h"
#include "spl06.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//#include "MPU6050.h"
//volatile uint32_t mpu_cycles = 0;
//volatile uint32_t mpu_max = 0;
//volatile uint32_t mpu_min = 0xFFFFFFFF;
//
//volatile uint32_t tim2_period = 0;
//volatile uint32_t tim2_max = 0;
//volatile uint32_t tim2_min = 0xFFFFFFFF;


#include <stdio.h>
#include "NRF24L01.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "dmpKey.h"
//#include "dmpmap.h"
#include "MPU6050.h"

	float az_mps2 = 0 ;
static float acc_vertical_lpf = 0.0f;
#define outmax    25
#define outmin   -25
float acc_z;
int16_t S;
int16_t l,s2,s3,s4,s5;
int16_t ax,ay,az,gx,gy,gz;
uint8_t m1;
uint8_t m2;
char BlueSerial_RxPacket[100];
volatile int16_t BlueSerial_RxFlag=0;
uint8_t a = 0;
float ax_lin,ay_lin,az_lin;
float az_bias = 0;
extern I2C_HandleTypeDef hi2c1;
//BMP280_HandleTypeDef hbmp280;
uint8_t RunFlag = 0,key1flag=0,key2flag=0;
uint8_t ReadFlag=0;
float takeoff_altitude;
float fused_altitude;      // 融合后高度（厘米）
float vertical_velocity = 0.0f;    // 垂直速度（厘米/秒）
float acc_z_offset = 0.0f;
float acc_alpha = 0.002;           // 加速度计权重（小值）
const float alpha = 0.90f;
float alpha1 = 0.8f;
float altitude;// 气压计权重
float high;
float e1,e2,e3;
float w = 0.2;
static float smooth_vel = 0;
volatile	float kalman_altitude;
volatile   float kalman_velocity;
uint8_t Run;
float baro_alt;
float fused_height ;
float fused_velocity ;

float acc_p_raw, acc_r_raw;




// 加速度计Z轴零偏（静止时校准）
#define SEA_LEVEL_PRESSURE 101325.0f  // 标准海平面气压 (Pa)

float pressure_to_altitude(float pressure_pa) {
    // 输入气压（Pa），输出海拔（米）
    return 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE, 0.190295f));
}
float ax1,ay1,az1,gx1,gy1,gz1,acc_p,acc_r,acc_y;
float complementary_alpha = 0.99;
float gyro_p,gyro_r,gyro_y,gyro_p1,gyro_r1,gyro_y1,p_angle,r_angle,y_angle;
float dt = 0.005f;
volatile float Target = 0,Target1 = 0,Target2 = 0;
volatile uint8_t m,n,h = 0;
float Error0,Error1,ErrorInt,E0,E1,EI,E3,E4,EI1,Error0_r,Error1_r,ErrorInt_r,E0_r,E1_r,EI_r,E3_r,E4_r,EI1_r;
volatile float Actual,Actual1,Out1,Actual2,Out2,Out3,Out4,Actual_r,Actual1_r,Actual2_r,Target1_r,Target_r,Target2_r,A_h,t_h,E0_h,
E1_h,EI_h,Ar_h,tr_h,E0r_h,E1r_h,EIr_h;
float Out ;//= 60.0;
typedef struct{
	float p;
	float i;
	float d;
	float Out;
}PID_typedef;
	PID_typedef pi = {3.7,0.01,0};
	PID_typedef ro ={3.7,0.01,0};
	PID_typedef ya = {3.7,0.01,0};
	PID_typedef pi_r = {0.08,0,0.1};
	PID_typedef ro_r = {0.08,0,0.1};
	PID_typedef ya_r = {0.15,0,0};
	PID_typedef hig  = {1.2,0,0};
	PID_typedef hig_r = {1.5,0,0};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t Duty;
uint32_t capture;
uint8_t  Key;

static float base_alt = 0.0f;
static uint8_t first_measure = 1;

float gyro_x_offset ;
float gyro_y_offset ;
float gyro_z_offset ;



//二阶滤波结构体
typedef struct{
	float b0,b1,b2;    //分子系数
	float a1,a2;		//分母系数
	float x1,x2;		//输入历史值
	float y1,y2;		//输出历史值
}biqued_filter_t;
biqued_filter_t filter_pitch, filter_roll;
biqued_filter_t filter_yaw;
biqued_filter_t filter_ax, filter_ay, filter_az;
void Vofa_SendJustFloat(float *data, uint8_t channelCount) {
    // 帧尾固定 4 字节
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    for (uint8_t i = 0; i < channelCount; i++) {
        // 将 float 按小端字节序发送（STM32 是小端）
        uint8_t *bytePtr = (uint8_t*)&data[i];
        HAL_UART_Transmit(&huart3, bytePtr, 4, 0xFFFF);
    }
    // 发送帧尾
    HAL_UART_Transmit(&huart3, (uint8_t*)tail, 4, 0xFFFF);
}

typedef struct {
    float x;
    float p;
    float q;
    float r;
} Kalman_1D_t;

// 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟剿诧拷锟斤拷
static Kalman_1D_t  kalman = {
    .x = 0.0f,
    .p = 0.1f,
    .q = 0.005f,
    .r = 0.02f
};

static uint8_t kalman_initialized = 0;

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}


float Kalman_Update_Height(Kalman_1D_t *k, float measurement) {
    k->p = k->p + k->q;
    float gain = k->p / (k->p + k->r);
    k->x = k->x + gain * (measurement - k->x);
    k->p = (1.0f - gain) * k->p;
    return k->x;
}
//蓝牙
void BlueSerial_HAL_Printf_IT(UART_HandleTypeDef *huart, const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, strlen(buf));
}
void BlueSerial_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart3,&Byte,1,10);

}
void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for(i=0;String[i]!= '\0';i++)
	{
		BlueSerial_SendByte(String[i]);
	}


}


void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	BlueSerial_SendString(String);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t rx_state = 0;
    static uint8_t rx_index = 0;

    if(huart->Instance == USART3)
    {
        if(rx_state == 0)
        {
            if(a == '[')
            {
                rx_state = 1;
                rx_index = 0;
                memset(BlueSerial_RxPacket, 0, sizeof(BlueSerial_RxPacket));
            }
        }
        else if(rx_state == 1)
        {
            if(a == ']')
            {
                rx_state = 0;
                BlueSerial_RxPacket[rx_index] = '\0';
                BlueSerial_RxFlag = 1;
            }
            else
            {
                if(rx_index < sizeof(BlueSerial_RxPacket) - 1)
                {
                    BlueSerial_RxPacket[rx_index++] = a;
                }
                else
                {

                    rx_state = 0;
                }
            }
        }


        HAL_UART_Receive_IT(&huart3, (uint8_t*)&a, 1);
    }
}


//初始化，截止频率50HZ，采样频率200HZ
void biquad_Init(biqued_filter_t*f,float cutoff,float sample_freq)
{
	float omega = 2.0f*3.14159f*cutoff/sample_freq;
	float sn = sinf(omega);
	float cs = cosf(omega);
	float alpha3 = sn/(2.0f*0.707f);
	float a0 = 1.0f + alpha3;
	f->b0 = (1.0f - cs)/2.0f/a0;
	f->b1 = (1.0f - cs)/a0;
	f->b2 = f->b0;
	f->a1 = -2.0f*cs/a0;
	f->a2 = (1.0f - alpha3)/a0;


	  // 清零历史
	f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;

}
float biquad_filter(biqued_filter_t*f,float input)
{
	float output = f->b0*input+f->b1*f->x1+f->b2*f->x2-f->a1*f->y1-f->a2*f->y2;
	f->x2 = f->x1;
	f->x1 = input;
	f->y2 = f->y1;
	f->y1 = output;
	return output;
}
float out(float outi,float oug,float oiu)
{
	if(outi > oiu)
		return oiu;
	if(outi < oug)
		return oug;
	return outi;

}

typedef struct {
    float x[2];     // 状态: [0]=高度(m), [1]=速度(m/s)
    float P[2][2];  // 协方差矩阵
    float Q[2][2];  // 过程噪声
    float R;        // 测量噪声
    float dt;       // 时间步长(s)
} KalmanHeight_t;
KalmanHeight_t kf_height;
// 加速度计原始值的一维卡尔曼滤波器（每个轴一个）
//static Kalman_1D_t kf_ax = { .x = 0.0f, .p = 1.0f, .q = 0.001f, .r = 100.0f };
//static Kalman_1D_t kf_ay = { .x = 0.0f, .p = 1.0f, .q = 0.001f, .r = 100.0f };
//static Kalman_1D_t kf_az = { .x = 2048.0f, .p = 1.0f, .q = 0.001f, .r = 100.0f };

static Kalman_1D_t kf_ax = { .x = 0.0f,    .p = 1.0f, .q = 0.1f,  .r = 10.0f };
static Kalman_1D_t kf_ay = { .x = 0.0f,    .p = 1.0f, .q = 0.1f,  .r = 10.0f };
static Kalman_1D_t kf_az = { .x = 2048.0f, .p = 1.0f, .q = 0.1f,  .r = 10.0f };

//static Kalman_1D_t kf_ax = { .x = 0.0f,    .p = 1.0f, .q = 0.01f, .r = 20.0f };
//static Kalman_1D_t kf_ay = { .x = 0.0f,    .p = 1.0f, .q = 0.01f, .r = 20.0f };
//static Kalman_1D_t kf_az = { .x = 2048.0f, .p = 1.0f, .q = 0.01f, .r = 20.0f };
// 高度一维卡尔曼滤波器
//static Kalman_1D_t kf_height_1d = {
//    .x = 0.0f,
//    .p = 1.0f,
//    .q = 0.001f,   // 过程噪声：越小越平滑
//    .r = 0.5f      // 测量噪声：越大越信任预测（越平滑）
//};
static Kalman_1D_t kf_height_1d = {
    .x = 0.0f, .p = 1.0f, .q = 0.01f, .r = 0.1f
};
// 注意：az 初始值给 2048（1g），因为重力始终作用在 Z 轴

void KalmanHeight_Init(KalmanHeight_t *kf, float dt, float Q_h, float Q_v, float R) {
    kf->dt = dt;
    // 初始状态
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    // 初始协方差（给较大值表示不确定性）
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
    // 过程噪声矩阵
    kf->Q[0][0] = Q_h;   kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;  kf->Q[1][1] = Q_v;
    // 测量噪声
    kf->R = R;
}

void KalmanHeight_Predict(KalmanHeight_t *kf, float acc_z) {
    float dt = kf->dt;
    // 状态预测
    float h_pred = kf->x[0] + kf->x[1]*dt + 0.5f*acc_z*dt*dt;
    float v_pred = kf->x[1] + acc_z*dt;
    // 预测协方差: P = F*P*F^T + Q
    float P00 = kf->P[0][0], P01 = kf->P[0][1], P10 = kf->P[1][0], P11 = kf->P[1][1];
    float F00 = 1.0f, F01 = dt, F10 = 0.0f, F11 = 1.0f;
    float P00_pred = F00*(F00*P00 + F01*P10) + F01*(F00*P01 + F01*P11) + kf->Q[0][0];
    float P01_pred = F00*(F10*P00 + F11*P10) + F01*(F10*P01 + F11*P11) + kf->Q[0][1];
    float P10_pred = F10*(F00*P00 + F01*P10) + F11*(F00*P01 + F01*P11) + kf->Q[1][0];
    float P11_pred = F10*(F10*P00 + F11*P10) + F11*(F10*P01 + F11*P11) + kf->Q[1][1];
    kf->P[0][0] = P00_pred; kf->P[0][1] = P01_pred;
    kf->P[1][0] = P10_pred; kf->P[1][1] = P11_pred;
    // 更新状态
    kf->x[0] = h_pred;
    kf->x[1] = v_pred;
}


void KalmanHeight_Update(KalmanHeight_t *kf, float baro_height) {
    // 观测矩阵 H = [1, 0]（只观测高度，不观测速度）
    float H0 = 1.0f, H1 = 0.0f;

    // 1. 计算卡尔曼增益
    float PHt0 = kf->P[0][0] * H0 + kf->P[0][1] * H1;
    float PHt1 = kf->P[1][0] * H0 + kf->P[1][1] * H1;
    float S = H0 * PHt0 + H1 * PHt1 + kf->R;
    float K0 = PHt0 / S;
    float K1 = PHt1 / S;

    // 2. 残差（测量值 - 预测值）
    float y = baro_height - (H0 * kf->x[0] + H1 * kf->x[1]);

    // 3. 更新状态
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    // 4. 更新协方差矩阵
    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];
    kf->P[0][0] = P00 - K0 * PHt0;
    kf->P[0][1] = P01 - K0 * PHt1;
    kf->P[1][0] = P10 - K1 * PHt0;
    kf->P[1][1] = P11 - K1 * PHt1;
}

// 一维卡尔曼：输入一个测量值，返回滤波后的估计值
float Kalman1D_Scalar(Kalman_1D_t *k, float measurement) {
    // 预测
    k->p = k->p + k->q;
    // 卡尔曼增益
    float gain = k->p / (k->p + k->r);
    // 更新
    k->x = k->x + gain * (measurement - k->x);
    k->p = (1.0f - gain) * k->p;
    return k->x;
}

typedef struct{
	float angle;       //估算的角度
	float bias;       //估计陀螺仪零飘
	float P[2][2];    //误差协方差矩阵
	float Q_angle;    //角度过程噪声
	float Q_bias;      //零飘过程噪声
	float R_angle;     //加速度计测量噪声

}Kalman_t;
Kalman_t kf_roll,kf_pitch,kf_yaw;

void Kalman_Init(Kalman_t*kf,float Q_angle,float Q_bias,float R_angle)
{

	kf -> angle = 0;
	kf -> bias = 0;
	kf -> P[0][0] = 1;   //初始化协方差,可以给大一些;
	kf -> P[0][1] = 0;
	kf -> P[1][0] = 0;
	kf -> P[1][1] = 1;
	kf -> Q_angle = Q_angle;
	kf -> Q_bias = Q_bias;
	kf -> R_angle = R_angle;
}
//预测
void Kalman_Update(Kalman_t*kf,float gyro_rate,float acc_angle,float dt)
{
	float angle_pred = kf->angle + (gyro_rate - kf->bias)*dt;
	float bias_pred = kf->bias;
	//预测协方差
	float P00 = kf->P[0][0];
	float P01 = kf->P[0][1];
	float P10 = kf->P[1][0];
	float P11 = kf->P[1][1];
	float P00_pred = P00 - dt*(P01 + P10)+dt*dt*P11+kf->Q_angle*dt;
	float P01_pred = P01 - dt*P11;
	float P10_pred = P10 - dt*P11;
	float P11_pred = P11 + kf->Q_bias*dt;
	// 计算卡尔曼增益
	float S = P00_pred + kf->R_angle;
	float k0 = P00_pred/S; //角度的增益
	float k1 = P10_pred/S;//零飘的增益
	//更新
	float y = acc_angle - angle_pred; //残差
	kf-> angle = angle_pred + k0*y;
	kf-> bias = bias_pred + k1*y;
	//更逊协方差
	float P00_New = (1 - k0) * P00_pred;
	float P01_New = (1 - k0) * P01_pred;
	float P10_New = P10_pred - k1 * P00_pred;   // 注意：不是 -K1*P00*P10
	float P11_New = P11_pred - k1 * P01_pred;
	kf->P[0][0] = P00_New;
	kf->P[0][1] = P01_New;
	kf->P[1][0] = P10_New;
	kf->P[1][1] = P11_New;
}

void Gyro_Calibrate(I2C_HandleTypeDef*hi2c)
{
    int samples = 200;  // 锟斤拷锟斤拷锟斤拷锟斤拷
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t gx, gy, gz;

    for (int i = 0; i < samples; i++) {
        MPU6050_ReadGyro(hi2c, &gx, &gy, &gz);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        HAL_Delay(5);  // 锟斤拷锟斤拷锟斤拷锟狡ワ拷洌�5ms = 200Hz锟斤拷
    }
    gyro_x_offset = (float)sum_gx / samples;
    gyro_y_offset = (float)sum_gy / samples;
    gyro_z_offset = (float)sum_gz / samples;
}

void MPU()
{



	 MPU6050_ReadAll(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);






  //   acc_z = az / 2048.0f;
   //  float acc_world_z = (acc_z - 1.0f - acc_z_offset)*980.0f;  // 近似，忽略姿态倾斜
//    ax1 = alpha1* ax1 + (1-alpha1)*ax;
//	 ay1 = alpha1* ay1 + (1-alpha1)*ay;
//	 az1 = alpha1* az1 + (1-alpha1)*az;
//	 acc_p = atan2(ax1,az1)*180.0f/M_PI;
//	acc_r = atan2(ay1,az1)*180.0f/M_PI;
	 // ★ 先算原始角度（用于对比显示）
	 float ax_raw = (float)ax;
	 float ay_raw = (float)ay;
	 float az_raw = (float)az;
	  acc_p_raw = atan2(ax_raw, az_raw) * 180.0f / M_PI;
	  acc_r_raw = atan2(ay_raw, az_raw) * 180.0f / M_PI;

	 // ★ 用二阶滤波处理原始加速度信号
	  ax1 = Kalman1D_Scalar(&kf_ax, (float)ax);
	  ay1 = Kalman1D_Scalar(&kf_ay, (float)ay);
	  az1 = Kalman1D_Scalar(&kf_az, (float)az - az_bias);



	 // ★ 用滤波后的数据计算角度
	 acc_p = atan2(ax1, az1) * 180.0f / M_PI;
	 acc_r = atan2(ay1, az1) * 180.0f / M_PI;
	//acc_y = atan2(ax1,ay1)*180.0f/M_PI;
//	 gyro_p1 = -(float)gy / 16.4f; //((float)gy1-gyro_y_offset) / 16.4f;//;
//	 gyro_r1 =(float)gx/ 16.4f;//((float)gx1-gyro_x_offset)/ 16.4f; ////Kalman_Update(&kalman_pitch, acc_p, gyro_p, dt);
	 gyro_p1 = -((float)gy - gyro_y_offset) / 16.4f;
	 gyro_r1 =  ((float)gx - gyro_x_offset) / 16.4f;
	 gyro_y = ((float)gz - gyro_z_offset) / 16.4f;
	 gyro_y = biquad_filter(&filter_yaw, gyro_y);
	 static float gyro_y_lpf = 0;
	 gyro_y_lpf = 0.95f * gyro_y_lpf + 0.05f * gyro_y;
	 gyro_y = gyro_y_lpf;
	 acc_y = ((float)gz - gyro_z_offset) / 16.4f;
	 gyro_p =  w*gyro_p+(1-w)*gyro_p1;
	 gyro_r = w*gyro_r+(1-w)*gyro_r1;
    y_angle += gyro_y * dt;


    Kalman_Update(&kf_pitch, gyro_p, acc_p, dt);
        Kalman_Update(&kf_roll,  gyro_r, acc_r, dt);

        p_angle = kf_pitch.angle;
        r_angle = kf_roll.angle;

  	  // 世界系变换用 ax1/ay1/az1
        float ax_mps2 = (ax1 / 2048.0f) * 9.8f;
  	float ay_mps2 = (ay1 / 2048.0f) * 9.8f;
  	 az_mps2 = (az1 / 2048.0f) * 9.8f;
  	float pitch = kf_pitch.angle * (M_PI / 180.0f);   // 用融合后的绝对角度
  	float roll  = kf_roll.angle  * (M_PI / 180.0f);

  	float gx_b = 9.8f * sinf(pitch);
  	float gy_b = 9.8f * sinf(roll) * cosf(pitch);
  	float gz_b = 9.8f * cosf(roll) * cosf(pitch);

  	ax_lin = ax_mps2 - gx_b;
  	ay_lin = ay_mps2 - gy_b;
  	az_lin = az_mps2 - gz_b;

        // 转到世界系 Z 轴
        float acc_world_z = az_lin * cosf(roll) * cosf(pitch)
                          + ax_lin * sinf(pitch)
                          + ay_lin * sinf(roll) * cosf(pitch);
  	//  float acc_vertical = acc_world_z - 9.8f;
        float acc_vertical = acc_world_z- acc_z_offset;

  	  acc_vertical_lpf = 0.05f * acc_vertical + 0.95f * acc_vertical_lpf;
  	 if (acc_vertical_lpf > -0.03f && acc_vertical_lpf < 0.03f)
  	        {
  	            acc_vertical_lpf = 0.0f;
  	        }
  	        if (acc_vertical_lpf > 2.0f)  acc_vertical_lpf = 2.0f;
  	        if (acc_vertical_lpf < -2.0f) acc_vertical_lpf = -2.0f;
  	  KalmanHeight_Predict(&kf_height, acc_vertical_lpf);
//角度外环
	 if(s2>=5)
	 {

		 s2 = 0;
	 Actual = p_angle;
	 Error1 = Error0;
	 Error0 = Target - Actual;
	 ErrorInt += Error0*dt*2;
		 if(ErrorInt>10)
	 {
		 ErrorInt = 10;
	 }
	 if(ErrorInt<-10)
	 {
		 ErrorInt = -10;
	 }
	 pi.Out = pi.p*Error0 + pi.i * ErrorInt+pi.d * (Error0 - Error1);
	    if(pi.Out >150)
	    {
	    	pi.Out =150;

	    }
	    if(pi.Out < -150)
	    {
	    	pi.Out = -150;
	    }
	    Target_r = pi.Out;
	    //Target_r = 50.0f;
 //横滚角度
		 Actual1 = r_angle;
		 E1 = E0;
		 E0 = Target1 - Actual1;
		 EI += E0*dt*2;
		 if(EI>10)
		 {
			 EI = 10;
		 }
		 if(EI<-10)
		 {
			 EI = -10;
		 }

		 ro.Out = ro.p*E0 + ro.i * EI+ro.d * (E0 - E1);

		    if(ro.Out >150)
		    {
		    	ro.Out =150;

		    }
		    if(ro.Out < -150)
		    {
		    	ro.Out = -150;
		    }
		    Target1_r = ro.Out;
	//偏航角度
			 Actual2 = y_angle;
			 E4 = E3;
			 E3 = Target2 - Actual2;
			 EI1 += E3*dt*2;
			 if(EI1>10)
			 {
				 EI1 = 10;
			 }
			 if(EI1<-10)
			 {
				 EI1 = -10;
			 }

			 ya.Out = ya.p*E3+ ya.i * EI1+ya.d * (E3 - E4);
			    if(ya.Out >90)
			    {
			    	ya.Out =90;

			    }
			    if(ya.Out < -90)
			    {
			    	ya.Out = -90;
			    }
			    Target2_r = ya.Out;

	 }
	 if((p_angle>-40)&&(p_angle<40)&&(r_angle>-40)&&(r_angle<40))
	 {
		 Run = 1;
	 }
	 else {
		Run = 0;


	}



//俯仰角速度内环
		 Actual_r = gyro_p;
		 Error1_r = Error0_r;
		 Error0_r = Target_r - Actual_r;
		 ErrorInt_r += Error0_r*dt;
		 if(ErrorInt_r>10)
		 {
			 ErrorInt_r = 10;
		 }
		 if(ErrorInt_r<-10)
		 {
			 ErrorInt_r = -10;
		 }
		 pi_r.Out = pi_r.p*Error0_r + pi_r.i * ErrorInt_r+pi_r.d * (Error0_r - Error1_r);

		    if(pi_r.Out >outmax)
		    {
		    	pi_r.Out =outmax;

		    }
		    if(pi_r.Out < outmin)
		    {
		    	pi_r.Out = outmin;
		    }
		    //高度环内环
		    if(s4>=2)
		    {
		    	s4 = 0;
		   	 Ar_h = fused_velocity * 100.0f;
		   	 E1r_h = E0r_h;
		   	 E0r_h = tr_h - Ar_h;
		   	 EIr_h += E0r_h*dt*5;
		   	 if(EIr_h>10)
		   	 {
		   		 EIr_h = 10;
		   	 }
		   	 if(EIr_h<-10)
		   	 {
		   		 EIr_h = -10;
		   	 }

		   	 hig_r.Out = hig_r.p*E0r_h + hig_r.i * EIr_h+hig_r.d * (E0r_h - E1r_h);

		   	    if(hig_r.Out >15)
		   	    {
		   	    	hig_r.Out =15;

		   	    }
		   	    if(hig_r.Out < -15)
		   	    {
		   	    	hig_r.Out = -15;
		   	    }


		    }
		    //高度外环
		    if(s5>6)
		    {
		    	s5 = 0;
		   	 A_h = (fused_height*100);
		   	 E1_h = E0_h;
		   	 E0_h = t_h - A_h;
		   	 EI_h += E0_h*dt*10;
		   	 if(EI_h>10)
		   	 {
		   		 EI_h = 10;
		   	 }
		   	 if(EI_h<-10)
		   	 {
		   		 EI_h = -10;
		   	 }

		   	 hig.Out = hig.p*E0_h + hig.i * EI_h+hig.d * (E0_h - E1_h);

		   	    if(hig.Out >50)
		   	    {
		   	    	hig.Out =50;

		   	    }
		   	    if(hig.Out < -50)
		   	    {
		   	    	hig.Out = -50;
		   	    }
tr_h= hig.Out;


		    }
    //横滚角速度内环
	 Actual1_r = gyro_r;
	 E1_r = E0_r;
	 E0_r = Target1_r - Actual1_r;
	 EI_r += E0_r*dt;
	 if(EI_r>10)
	 {
		 EI_r = 10;
	 }
	 if(EI_r<-10)
	 {
		 EI_r = -10;
	 }

	 ro_r.Out = ro_r.p*E0_r + ro_r.i * EI_r+ro_r.d * (E0_r - E1_r);

	    if(ro_r.Out >outmax)
	    {
	    	ro_r.Out =outmax;

	    }
	    if(ro_r.Out < outmin)
	    {
	    	ro_r.Out = outmin;
	    }
	 //偏航角速度内环
	 Actual2_r = gyro_y;
	 E4_r = E3_r;
	 E3_r = Target2_r - Actual2_r;
	 EI1_r += E3_r*dt;
	 if(EI1_r>10)
	 {
		 EI1_r = 10;
	 }
	 if(EI1_r<-10)
	 {
		 EI1_r = -10;
	 }

	 ya_r.Out = ya_r.p*E3_r+ ya_r.i * EI1_r+ya_r.d * (E3_r - E4_r);
	    if(ya_r.Out >20)
	    {
	    	ya_r.Out =20;

	    }
	    if(ya_r.Out < -20)
	    {
	    	ya_r.Out = -20;
	    }
	if(RunFlag == 1)
	{


		if(Run)
		{
			//Run = 0;

		Out1= Out + pi_r.Out - ro_r.Out - ya_r.Out+hig_r.Out;
		Out2= Out - pi_r.Out - ro_r.Out + ya_r.Out+hig_r.Out;
		Out3= Out - pi_r.Out + ro_r.Out - ya_r.Out+hig_r.Out;
		Out4= Out + pi_r.Out + ro_r.Out + ya_r.Out+hig_r.Out;
		if(Out1>100)
		{
			Out1 = 100;
		}
		if(Out1<2)
		{
			Out1 = 0;
		}
		if(Out2>100)
		{
			Out2 = 100;
		}
		if(Out2<2)
		{
			Out2 = 0;
		}
		if(Out3>100)
		{
			Out3 = 100;
		}
		if(Out3<2)
		{
			Out3 = 0;
		}
		if(Out4>100)
		{
			Out4 = 100;
		}
		if(Out4<2)
		{
			Out4 = 0;
		}

	 Left1_Speed(Out1 );
	 Left2_Speed(Out2 );
	 Right1_Speed(Out3);
	 Right2_Speed(Out4 );
		}
		if(Run == 0)
		{
			RunFlag = 0;
			Stop();
		}

	}




//	     float ax_mps2 = (ax / 2048.0f) * 9.8f;
//	         float ay_mps2 = (ay / 2048.0f) * 9.8f;
//	         float az_mps2 = (az / 2048.0f) * 9.8f;
//	         float pitch = p_angle * (M_PI / 180.0f);
//	          float roll  = r_angle * (M_PI / 180.0f);
//	          float acc_world_z = az_mps2 * cosf(pitch) * cosf(roll)
//	                                + ax_mps2 * sinf(pitch)
//	                                - ay_mps2 * sinf(roll);
//	          float acc_vertical = acc_world_z - 9.8f;
//	          static float acc_vertical_lpf = 0;
//	            const float acc_lpf = 0.02f;
//	            acc_vertical_lpf = acc_lpf * acc_vertical + (1 - acc_lpf) * acc_vertical_lpf;
//	            KalmanHeight_Predict(&kf_height, acc_vertical_lpf);

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim)
{
	if(htim->Instance == TIM2)
	{
		MPU();

		m++;
		n++;
		s2++;
		s3++;
		s4++;
		s5++;
//		  if(s2>100)
//		  {
//			  RunFlag = 0;
//			  Stop();
//		  }


	}

}


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //OLED_Init();

//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//  KalmanHeight_Init(&kf_height, dt, 0.1f, 0.5f, 0.05f);
//  KalmanHeight_Init(&kf_height, 0.005f, 0.001f, 0.01f, 0.5f);
  KalmanHeight_Init(&kf_height, 0.005f, 0.001f, 0.001f, 0.1f);
  if (SPL06_Init() != 0) {              while (1);
      }
  BlueSerial_Printf("[plot,%d]",(int16_t)1);
//  Kalman_Init(&kf_roll,0.001f,0.0001f,0.03f);
//  Kalman_Init(&kf_pitch,0.001f,0.0001f,0.03f);
  Kalman_Init(&kf_roll,  0.001f, 0.003f, 0.5f);
  Kalman_Init(&kf_pitch, 0.001f, 0.003f, 0.5f);
  MPU6050_Init(&hi2c1);
HAL_Delay(500);
Gyro_Calibrate(&hi2c1);
BlueSerial_Printf("[plot,%d]",(int16_t)2);
MPU6050_ReadAll(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);
kf_ax.x = ax;
kf_ay.x = ay;
kf_az.x = (float)az;   // az_bias = 0，不减

// ★ 融合卡尔曼角度也初始化到当前倾角
float p0 = atan2f((float)ax, (float)az) * 180.0f / M_PI;
float r0 = atan2f((float)ay, (float)az) * 180.0f / M_PI;
kf_pitch.angle = p0;
kf_roll.angle  = r0;

  NRF24L01_Init();
  biquad_Init(&filter_yaw,   30.0f, 200.0f);
  // ★ 新增加速度计二阶滤波（截止频率 20Hz，采样 200Hz）
//  biquad_Init(&filter_ax, 20.0f, 200.0f);
//  biquad_Init(&filter_ay, 20.0f, 200.0f);
//  biquad_Init(&filter_az, 20.0f, 200.0f);
  float acc_z_sum = 0;
   float p_rad = kf_pitch.angle * (M_PI / 180.0f);
   float r_rad = kf_roll.angle  * (M_PI / 180.0f);

   for(int i = 0; i < 500; i++)
   {
       MPU6050_ReadAll(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

       // 直接用原始值算
       float ax_mps2 = ((float)ax / 2048.0f) * 9.8f;
       float ay_mps2 = ((float)ay / 2048.0f) * 9.8f;
       float az_mps2 = ((float)az / 2048.0f) * 9.8f;

       float gx_b = 9.8f * sinf(p_rad);
       float gy_b = 9.8f * sinf(r_rad) * cosf(p_rad);
       float gz_b = 9.8f * cosf(r_rad) * cosf(p_rad);

       float ax_lin = ax_mps2 - gx_b;
       float ay_lin = ay_mps2 - gy_b;
       float az_lin = az_mps2 - gz_b;

       float acc_world_z = az_lin * cosf(r_rad) * cosf(p_rad)
                         + ax_lin * sinf(p_rad)
                         + ay_lin * sinf(r_rad) * cosf(p_rad);

       acc_z_sum += acc_world_z;
       HAL_Delay(5);
   }
   acc_z_offset = acc_z_sum / 500.0f;
   BlueSerial_Printf("[plot,%d]",(int16_t)3);
   // 这个值大概就是 -0.1
  HAL_TIM_Base_Start_IT(&htim2);



  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//if(ReadFlag)
//{
//	ReadFlag = 0;
//	    // 1. 读取气压计原始高度
//	    float alt_raw = SPL06_ReadAltitude(101325.0f);
//	    if (first_measure) {
//	        base_alt = alt_raw;      // 记录起飞基准
//	        first_measure = 0;
//	    }
//	    baro_alt = alt_raw - base_alt;   // 气压计相对高度 (m)
//
//	    // 2. 卡尔曼更新（融合气压计）
//	    KalmanHeight_Update(&kf_height, baro_alt);
//
//	    // 3. 获取融合结果
//	   fused_height = kf_height.x[0];   // 融合高度 (m)
//	   fused_velocity = kf_height.x[1]; // 垂直速度 (m/s)
//}
//	  if(ReadFlag)
//	  {
//	      ReadFlag = 0;
//	      float alt_raw = SPL06_ReadAltitude(101325.0f);
//	      if (first_measure) {
//	          base_alt = alt_raw;
//	          first_measure = 0;
//	          kf_height_1d.x = 0.0f;   // 一维卡尔曼初始化为 0
//	      }
//	      baro_alt = alt_raw - base_alt;
//
//	      // ★ 只对气压计高度做一维卡尔曼滤波
//	      fused_height = Kalman1D_Scalar(&kf_height_1d, baro_alt);

	      // 速度用差分估算（可选）
//	      static float last_height = 0.0f;
//	      static uint32_t last_tick = 0;
//	      uint32_t now = HAL_GetTick();
//	      if (last_tick != 0) {
//	          float dt_h = (now - last_tick) / 1000.0f;
//	          if (dt_h > 0.001f) {
//	              fused_velocity = (fused_height - last_height) / dt_h;
//	          }
//	      }
//	      last_height = fused_height;
//	      last_tick = now;
//	  }
	  if(ReadFlag)
	  {
	      ReadFlag = 0;
	      float alt_raw = SPL06_ReadAltitude(101325.0f);

	      if(first_measure)
	      {
	          base_alt = alt_raw;
	          first_measure = 0;

	          // 二维卡尔曼从 0 开始
	          kf_height.x[0] = 0.0f;
	          kf_height.x[1] = 0.0f;
	          kf_height.P[0][0] = 1.0f;
	          kf_height.P[0][1] = 0.0f;
	          kf_height.P[1][0] = 0.0f;
	          kf_height.P[1][1] = 1.0f;

	          kf_height_1d.x = 0.0f;   // 一维仅保留对比，可删
	      }

	      baro_alt = alt_raw - base_alt;

	      // ★ 二维卡尔曼：气压计观测
	      KalmanHeight_Update(&kf_height, baro_alt);

	      // ★ 融合输出
	      fused_height   = kf_height.x[0];   // m
	      fused_velocity = kf_height.x[1];   // m/s
	  }


if(s3>5)
{
	s3=0;
    // 4. 发送 VOFA 数据（6个通道）
//    float vofaData[6] = {
//        baro_alt,          // 0: 原始气压相对高度
//        fused_height,      // 1: 融合高度
//        fused_velocity,    // 2: 融合速度
//        p_angle,           // 3: 俯仰角
//        r_angle,           // 4: 横滚角
//        y_angle            // 5: 偏航角
//    };
//
//  Vofa_SendJustFloat(vofaData, 6);
//  BlueSerial_Printf("[plot,%d,%d,%d,%d,%d]",(int16_t)(acc_p),(int16_t)(gyro_p),(int16_t)gyro_y,(int16_t)acc_r,(int16_t)acc_y);
	if(key1flag)
	{

		BlueSerial_Printf("[plot,%d,%d,%d,%d,%d]",(int16_t)(acc_p),(int16_t)(gyro_p),(int16_t)gyro_y,(int16_t)acc_r,(int16_t)acc_y);
		 // BlueSerial_Printf("[plot,%d,%d,%d,%d,%d]",(int16_t)hig.Out,(int16_t)(fused_height*100),(int16_t)Out,(int16_t)ya_r.Out,(int16_t)ro_r.Out);
	}
	else if(key2flag)
	{

		  BlueSerial_Printf("[plot,%d,%d,%d,%d,%d,%d]",
		       (int16_t)(ax),
		       (int16_t)(ax1),
		       (int16_t)(ay),
		       (int16_t)(ay1),
		       (int16_t)(az),(int16_t)(az1));
	}
	else {
		// BlueSerial_Printf("[plot,%d,%d,%d,%d,%d,%d]",(int16_t)(fused_height*100),(int16_t)(p_angle),(int16_t)r_angle,(int16_t)hig_r.Out,(int16_t)(kf_height.x[1]*10000),(int16_t)(acc_vertical_lpf*1000));
		 //BlueSerial_Printf("[plot,%d,%d,%d,%d,%d,%d]",(int16_t)az,(int16_t)az1,(int16_t)(kf_pitch.angle * 10),(int16_t)(kf_roll.angle*10),(int16_t)(az_mps2*1000),(int16_t)(az_lin*1000));
		 BlueSerial_Printf("[plot,%d,%d,%d]",(int16_t)(fused_height*100),(int16_t)(kf_height.x[1]*100),(int16_t)hig_r.Out);
	}

}

if(n>4)
{n = 0;

    ReadFlag = 1;
		 //

	          }



if(m>6)
{
	m = 0;

		if(NRF24L01_Receive() == 1)
		{
			//s2 = 0;
			Key = NRF24L01_RxPacket[5];
			float e3 = (NRF24L01_RxPacket[2]-100);

			/*NRF24L01_TxPacket[1] = js.x;
			NRF24L01_TxPacket[2] = js.y;*/
			e1 = (NRF24L01_RxPacket[1]-100);
			e2= (NRF24L01_RxPacket[4]-100);
			if((e1<5)&&(e1>-5))
			{
				e1 = 0;
			}
			if((e2<5)&&(e2>-5))
			{
				e2 = 0;
			}
             Target1  = e3*0.2;
             Target   = e1*0.2;
             if((e3<5)&&(e3>-5))
             			{
             				e3 = 0;
             			}
                 t_h = e2;

                  if(t_h > 30)       Out = 55;
                 else if(t_h > 25)  Out = 30;
                 else if(t_h > 15)  Out = 15;
                 else if(t_h > 10)  Out = 10;
                 else               Out = 0;

      		  if(NRF24L01_RxPacket[0] == 0x00 ||NRF24L01_RxPacket[0] == 0x01)
      		  {
      		  if(NRF24L01_RxPacket[0] == 0x01)
      		  {


      			NRF24L01_TxPacket[0] = 0x02;
      			*(float *)&NRF24L01_TxPacket[1] = p_angle;
      			*(float *)&NRF24L01_TxPacket[5] = r_angle;
      			*(float *)&NRF24L01_TxPacket[9] = y_angle;
      			*(float *)&NRF24L01_TxPacket[13] = (fused_height);

      			NRF24L01_Send();

      		  }

      		  if(Key == 2)
      		  {
      			 base_alt = SPL06_ReadAltitude(101325.0f);
      			    baro_alt = 0.0f; // 相对高度清零

      			    // 2. 重置二维卡尔曼高度融合器
      			    kf_height.x[0] = 0.0f;        // 融合高度清零
      			    kf_height.x[1] = 0.0f;        // 融合速度清零
      			    kf_height.P[0][0] = 1.0f;
      			    kf_height.P[0][1] = 0.0f;
      			    kf_height.P[1][0] = 0.0f;
      			    kf_height.P[1][1] = 1.0f;

      			    fused_height = 0.0f;
      			    fused_velocity = 0.0f;
      			    acc_vertical_lpf = 0.0f;

      			    // 3. 清空高度环 PID 的历史状态和积分 (防止积分饱和)
      			    E0_h = 0; E1_h = 0; EI_h = 0;
      			    hig.Out = 0;

      			    E0r_h = 0; E1r_h = 0; EIr_h = 0;
      			    hig_r.Out = 0;
      			    tr_h = 0;

      			    // 4. 安全保护：将目标高度清零，防止遥控器没回中直接起飞
      			    t_h = 0;
      			    Out = 0;

      			    // 5. 解锁电机

      			  RunFlag = 1;


      		  }
     		  if(Key == 6)
      		  {
key2flag =1;
key1flag = 0;
      		  }
           if(Key == 5)
           {
        	   RunFlag = 0;
        	   Stop();
        	   key2flag = 0;
        	   key1flag = 0;
           }
           if(Key == 1)
           {
        	   key1flag = 1;
        	   key2flag = 0;
           }

		}

		}

  }
  }
		//OLED_ShowNum(2,11,NRF24L01_Receive(),2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
