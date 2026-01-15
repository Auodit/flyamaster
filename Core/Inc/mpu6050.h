#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// MPU6050 I2C 地址 (AD0 接地)
#define MPU6050_ADDR (0x68 << 1)

// 寄存器地址
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

// 陀螺仪校准参数
typedef struct {
    float gyro_offset_x;    // 陀螺仪 X 轴零偏 (rad/s)
    float gyro_offset_y;    // 陀螺仪 Y 轴零偏 (rad/s)
    float gyro_offset_z;    // 陀螺仪 Z 轴零偏 (rad/s)
    float accel_offset_x;   // 加速度计 X 轴零偏 (g)
    float accel_offset_y;   // 加速度计 Y 轴零偏 (g)
    float accel_offset_z;   // 加速度计 Z 轴零偏 (g)
    uint8_t calibrated;     // 校准完成标志
} MPU6050_Calibration_t;

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Ax; // 单位: g (重力加速度)
    float Ay;
    float Az;
    float Gx; // rad/s
    float Gy;
    float Gz;
} MPU6050_t;

// 校准样本数量
#define MPU6050_CALIBRATION_SAMPLES  2000

// 全局校准数据 - Risk #078 修复：添加 volatile 修饰符（RTOS多任务共享变量）
extern volatile MPU6050_Calibration_t MPU6050_Calibration;

// 全局传感器数据 - Risk #078 修复：添加 volatile 修饰符（RTOS多任务共享变量）
extern volatile MPU6050_t MPU6050_Data;

// 基础函数
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);

// 校准函数
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All_Calibrated(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
uint8_t MPU6050_IsCalibrated(void);

#endif
