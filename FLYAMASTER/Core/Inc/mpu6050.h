/**
 * @file mpu6050.h
 * @brief MPU6050 IMU 驱动模块 (DMA 非阻塞模式)
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 实现 MPU6050 六轴传感器的 DMA 读取：
 * 1. 使用 I2C1 DMA 进行非阻塞数据读取
 * 2. 通过信号量与 imu_task 同步
 * 3. 支持 500Hz 采样率
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* I2C 地址 (7-bit 左移) */
#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_ADDR_ALT        (0x69 << 1)  /**< AD0 = HIGH */

/* 寄存器地址 */
#define MPU6050_REG_SMPLRT_DIV      0x19    /**< 采样率分频器 */
#define MPU6050_REG_CONFIG          0x1A    /**< 配置寄存器 */
#define MPU6050_REG_GYRO_CONFIG     0x1B    /**< 陀螺仪配置 */
#define MPU6050_REG_ACCEL_CONFIG    0x1C    /**< 加速度计配置 */
#define MPU6050_REG_INT_PIN_CFG     0x37    /**< 中断引脚配置 */
#define MPU6050_REG_INT_ENABLE      0x38    /**< 中断使能 */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B    /**< 加速度 X 高字节 */
#define MPU6050_REG_TEMP_OUT_H      0x41    /**< 温度高字节 */
#define MPU6050_REG_GYRO_XOUT_H     0x43    /**< 陀螺仪 X 高字节 */
#define MPU6050_REG_PWR_MGMT_1      0x6B    /**< 电源管理 1 */
#define MPU6050_REG_PWR_MGMT_2      0x6C    /**< 电源管理 2 */
#define MPU6050_REG_WHO_AM_I        0x75    /**< 设备 ID */

/* 设备 ID */
#define MPU6050_WHO_AM_I_VAL        0x68

/* 量程配置 */
#define MPU6050_GYRO_FS_250         0x00    /**< ±250 °/s */
#define MPU6050_GYRO_FS_500         0x08    /**< ±500 °/s */
#define MPU6050_GYRO_FS_1000        0x10    /**< ±1000 °/s */
#define MPU6050_GYRO_FS_2000        0x18    /**< ±2000 °/s */

#define MPU6050_ACCEL_FS_2G         0x00    /**< ±2g */
#define MPU6050_ACCEL_FS_4G         0x08    /**< ±4g */
#define MPU6050_ACCEL_FS_8G         0x10    /**< ±8g */
#define MPU6050_ACCEL_FS_16G        0x18    /**< ±16g */

/* 低通滤波器配置 */
#define MPU6050_DLPF_BW_256         0x00    /**< 256Hz */
#define MPU6050_DLPF_BW_188         0x01    /**< 188Hz */
#define MPU6050_DLPF_BW_98          0x02    /**< 98Hz */
#define MPU6050_DLPF_BW_42          0x03    /**< 42Hz */
#define MPU6050_DLPF_BW_20          0x04    /**< 20Hz */
#define MPU6050_DLPF_BW_10          0x05    /**< 10Hz */
#define MPU6050_DLPF_BW_5           0x06    /**< 5Hz */

/* 灵敏度系数 */
#define MPU6050_GYRO_SENS_250       131.0f  /**< LSB/(°/s) */
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

#define MPU6050_ACCEL_SENS_2G       16384.0f /**< LSB/g */
#define MPU6050_ACCEL_SENS_4G       8192.0f
#define MPU6050_ACCEL_SENS_8G       4096.0f
#define MPU6050_ACCEL_SENS_16G      2048.0f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MPU6050 原始数据结构
 */
typedef struct {
    int16_t accel_x;    /**< 加速度 X 原始值 */
    int16_t accel_y;    /**< 加速度 Y 原始值 */
    int16_t accel_z;    /**< 加速度 Z 原始值 */
    int16_t temp;       /**< 温度原始值 */
    int16_t gyro_x;     /**< 陀螺仪 X 原始值 */
    int16_t gyro_y;     /**< 陀螺仪 Y 原始值 */
    int16_t gyro_z;     /**< 陀螺仪 Z 原始值 */
} MPU6050_RawData_t;

/**
 * @brief MPU6050 物理数据结构
 */
typedef struct {
    float accel_x;      /**< 加速度 X (g) */
    float accel_y;      /**< 加速度 Y (g) */
    float accel_z;      /**< 加速度 Z (g) */
    float temp;         /**< 温度 (°C) */
    float gyro_x;       /**< 陀螺仪 X (°/s) */
    float gyro_y;       /**< 陀螺仪 Y (°/s) */
    float gyro_z;       /**< 陀螺仪 Z (°/s) */
} MPU6050_Data_t;

/**
 * @brief MPU6050 校准数据
 */
typedef struct {
    float gyro_offset_x;    /**< 陀螺仪 X 偏移 */
    float gyro_offset_y;    /**< 陀螺仪 Y 偏移 */
    float gyro_offset_z;    /**< 陀螺仪 Z 偏移 */
    float accel_offset_x;   /**< 加速度 X 偏移 */
    float accel_offset_y;   /**< 加速度 Y 偏移 */
    float accel_offset_z;   /**< 加速度 Z 偏移 */
} MPU6050_Calibration_t;

/**
 * @brief MPU6050 状态枚举
 */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR,
    MPU6050_BUSY,
    MPU6050_TIMEOUT,
    MPU6050_NOT_FOUND
} MPU6050_Status_t;

/* Exported variables --------------------------------------------------------*/
extern MPU6050_Data_t g_mpu6050_data;           /**< 全局 IMU 数据 */
extern MPU6050_Calibration_t g_mpu6050_calib;   /**< 全局校准数据 */
extern volatile bool g_mpu6050_data_ready;       /**< 数据就绪标志 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 MPU6050
 * @return MPU6050_Status_t 初始化状态
 */
MPU6050_Status_t MPU6050_Init(void);

/**
 * @brief 检测 MPU6050 是否存在
 * @return bool true=存在, false=不存在
 */
bool MPU6050_IsConnected(void);

/**
 * @brief 启动 DMA 读取 (非阻塞)
 * @return MPU6050_Status_t 操作状态
 * @note 读取完成后会触发 HAL_I2C_MemRxCpltCallback
 */
MPU6050_Status_t MPU6050_ReadDMA(void);

/**
 * @brief 处理 DMA 读取完成 (在回调中调用)
 * @note 将原始数据转换为物理量并存入 g_mpu6050_data
 */
void MPU6050_ProcessDMAData(void);

/**
 * @brief 阻塞式读取数据 (用于调试)
 * @param data 输出数据指针
 * @return MPU6050_Status_t 操作状态
 */
MPU6050_Status_t MPU6050_ReadBlocking(MPU6050_Data_t *data);

/**
 * @brief 校准陀螺仪 (需要静止放置)
 * @param samples 采样次数 (建议 1000)
 * @return MPU6050_Status_t 操作状态
 */
MPU6050_Status_t MPU6050_CalibrateGyro(uint16_t samples);

/**
 * @brief 设置陀螺仪量程
 * @param fs 量程配置 (MPU6050_GYRO_FS_xxx)
 * @return MPU6050_Status_t 操作状态
 */
MPU6050_Status_t MPU6050_SetGyroFullScale(uint8_t fs);

/**
 * @brief 设置加速度计量程
 * @param fs 量程配置 (MPU6050_ACCEL_FS_xxx)
 * @return MPU6050_Status_t 操作状态
 */
MPU6050_Status_t MPU6050_SetAccelFullScale(uint8_t fs);

/**
 * @brief 设置低通滤波器带宽
 * @param bw 带宽配置 (MPU6050_DLPF_BW_xxx)
 * @return MPU6050_Status_t 操作状态
 */
MPU6050_Status_t MPU6050_SetDLPF(uint8_t bw);

/**
 * @brief 获取温度 (°C)
 * @return float 温度值
 */
float MPU6050_GetTemperature(void);

/**
 * @brief I2C DMA 接收完成回调 (需要在 stm32f4xx_it.c 中调用)
 */
void MPU6050_I2C_RxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */