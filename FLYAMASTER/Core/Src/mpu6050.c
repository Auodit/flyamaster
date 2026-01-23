/**
 * @file mpu6050.c
 * @brief MPU6050 IMU 驱动模块实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "i2c.h"
#include "cmsis_os.h"

/* I2C1 互斥锁 (在 freertos.c 中定义) */
extern osMutexId_t i2c1MutexHandle;

/* 互斥锁超时时间 (ms) */
#define I2C1_MUTEX_TIMEOUT  100

/* Private defines -----------------------------------------------------------*/
#define MPU6050_TIMEOUT     100     /**< I2C 超时时间 (ms) */
#define MPU6050_DATA_LEN    14      /**< 一次读取的数据长度 (Accel + Temp + Gyro) */

/* Private variables ---------------------------------------------------------*/
MPU6050_Data_t g_mpu6050_data = {0};
MPU6050_Calibration_t g_mpu6050_calib = {0};
volatile bool g_mpu6050_data_ready = false;

static uint8_t s_dma_rx_buffer[MPU6050_DATA_LEN];   /**< DMA 接收缓冲区 */
static float s_gyro_sensitivity = MPU6050_GYRO_SENS_2000;
static float s_accel_sensitivity = MPU6050_ACCEL_SENS_8G;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data);
static void MPU6050_ConvertRawData(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 MPU6050
 */
MPU6050_Status_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0;
    
    /* 检测设备 */
    if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return MPU6050_NOT_FOUND;
    }
    
    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        return MPU6050_NOT_FOUND;
    }
    
    /* 复位设备 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80) != HAL_OK) {
        return MPU6050_ERROR;
    }
    HAL_Delay(100);
    
    /* 唤醒设备，使用 PLL with X axis gyroscope reference */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) {
        return MPU6050_ERROR;
    }
    HAL_Delay(10);
    
    /* 设置采样率分频器: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
     * Gyroscope Output Rate = 1kHz (when DLPF is enabled)
     * SMPLRT_DIV = 1 -> Sample Rate = 500Hz
     */
    if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x01) != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    /* 设置低通滤波器: 98Hz 带宽 */
    if (MPU6050_SetDLPF(MPU6050_DLPF_BW_98) != MPU6050_OK) {
        return MPU6050_ERROR;
    }
    
    /* 设置陀螺仪量程: ±2000°/s (适合穿越机高速翻滚) */
    if (MPU6050_SetGyroFullScale(MPU6050_GYRO_FS_2000) != MPU6050_OK) {
        return MPU6050_ERROR;
    }
    
    /* 设置加速度计量程: ±8g */
    if (MPU6050_SetAccelFullScale(MPU6050_ACCEL_FS_8G) != MPU6050_OK) {
        return MPU6050_ERROR;
    }
    
    /* 禁用中断 (使用 DMA 轮询) */
    if (MPU6050_WriteReg(MPU6050_REG_INT_ENABLE, 0x00) != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    /* 初始化校准数据 */
    g_mpu6050_calib.gyro_offset_x = 0.0f;
    g_mpu6050_calib.gyro_offset_y = 0.0f;
    g_mpu6050_calib.gyro_offset_z = 0.0f;
    g_mpu6050_calib.accel_offset_x = 0.0f;
    g_mpu6050_calib.accel_offset_y = 0.0f;
    g_mpu6050_calib.accel_offset_z = 0.0f;
    
    return MPU6050_OK;
}

/**
 * @brief 检测 MPU6050 是否存在
 */
bool MPU6050_IsConnected(void)
{
    uint8_t who_am_i = 0;
    
    if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return false;
    }
    
    return (who_am_i == MPU6050_WHO_AM_I_VAL);
}

/**
 * @brief 启动 DMA 读取 (非阻塞，带互斥锁保护)
 */
MPU6050_Status_t MPU6050_ReadDMA(void)
{
    HAL_StatusTypeDef status;
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return MPU6050_BUSY;
        }
    }
    
    /* 检查 I2C 是否忙 */
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        if (i2c1MutexHandle != NULL) {
            osMutexRelease(i2c1MutexHandle);
        }
        return MPU6050_BUSY;
    }
    
    /* 启动 DMA 读取 */
    status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                   MPU6050_ADDR,
                                   MPU6050_REG_ACCEL_XOUT_H,
                                   I2C_MEMADD_SIZE_8BIT,
                                   s_dma_rx_buffer,
                                   MPU6050_DATA_LEN);
    
    if (status != HAL_OK) {
        if (i2c1MutexHandle != NULL) {
            osMutexRelease(i2c1MutexHandle);
        }
        return MPU6050_ERROR;
    }
    
    /* 注意：互斥锁将在 DMA 完成回调中释放 */
    return MPU6050_OK;
}

/**
 * @brief 处理 DMA 读取完成
 */
void MPU6050_ProcessDMAData(void)
{
    MPU6050_ConvertRawData();
    g_mpu6050_data_ready = true;
}

/**
 * @brief 阻塞式读取数据 (带互斥锁保护)
 */
MPU6050_Status_t MPU6050_ReadBlocking(MPU6050_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[MPU6050_DATA_LEN];
    
    if (data == NULL) {
        return MPU6050_ERROR;
    }
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return MPU6050_BUSY;
        }
    }
    
    status = HAL_I2C_Mem_Read(&hi2c1,
                               MPU6050_ADDR,
                               MPU6050_REG_ACCEL_XOUT_H,
                               I2C_MEMADD_SIZE_8BIT,
                               buffer,
                               MPU6050_DATA_LEN,
                               MPU6050_TIMEOUT);
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
    
    if (status != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    /* 解析原始数据 */
    int16_t accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    /* 转换为物理量 */
    data->accel_x = (float)accel_x / s_accel_sensitivity - g_mpu6050_calib.accel_offset_x;
    data->accel_y = (float)accel_y / s_accel_sensitivity - g_mpu6050_calib.accel_offset_y;
    data->accel_z = (float)accel_z / s_accel_sensitivity - g_mpu6050_calib.accel_offset_z;
    data->temp = (float)temp / 340.0f + 36.53f;
    data->gyro_x = (float)gyro_x / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_x;
    data->gyro_y = (float)gyro_y / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_y;
    data->gyro_z = (float)gyro_z / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_z;
    
    return MPU6050_OK;
}

/**
 * @brief 校准陀螺仪
 */
MPU6050_Status_t MPU6050_CalibrateGyro(uint16_t samples)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    MPU6050_Data_t data;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (MPU6050_ReadBlocking(&data) != MPU6050_OK) {
            return MPU6050_ERROR;
        }
        
        sum_gx += data.gyro_x;
        sum_gy += data.gyro_y;
        sum_gz += data.gyro_z;
        
        HAL_Delay(2);  /* 500Hz */
    }
    
    g_mpu6050_calib.gyro_offset_x = sum_gx / samples;
    g_mpu6050_calib.gyro_offset_y = sum_gy / samples;
    g_mpu6050_calib.gyro_offset_z = sum_gz / samples;
    
    return MPU6050_OK;
}

/**
 * @brief 设置陀螺仪量程
 */
MPU6050_Status_t MPU6050_SetGyroFullScale(uint8_t fs)
{
    if (MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, fs) != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    /* 更新灵敏度系数 */
    switch (fs) {
        case MPU6050_GYRO_FS_250:
            s_gyro_sensitivity = MPU6050_GYRO_SENS_250;
            break;
        case MPU6050_GYRO_FS_500:
            s_gyro_sensitivity = MPU6050_GYRO_SENS_500;
            break;
        case MPU6050_GYRO_FS_1000:
            s_gyro_sensitivity = MPU6050_GYRO_SENS_1000;
            break;
        case MPU6050_GYRO_FS_2000:
            s_gyro_sensitivity = MPU6050_GYRO_SENS_2000;
            break;
        default:
            return MPU6050_ERROR;
    }
    
    return MPU6050_OK;
}

/**
 * @brief 设置加速度计量程
 */
MPU6050_Status_t MPU6050_SetAccelFullScale(uint8_t fs)
{
    if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, fs) != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    /* 更新灵敏度系数 */
    switch (fs) {
        case MPU6050_ACCEL_FS_2G:
            s_accel_sensitivity = MPU6050_ACCEL_SENS_2G;
            break;
        case MPU6050_ACCEL_FS_4G:
            s_accel_sensitivity = MPU6050_ACCEL_SENS_4G;
            break;
        case MPU6050_ACCEL_FS_8G:
            s_accel_sensitivity = MPU6050_ACCEL_SENS_8G;
            break;
        case MPU6050_ACCEL_FS_16G:
            s_accel_sensitivity = MPU6050_ACCEL_SENS_16G;
            break;
        default:
            return MPU6050_ERROR;
    }
    
    return MPU6050_OK;
}

/**
 * @brief 设置低通滤波器带宽
 */
MPU6050_Status_t MPU6050_SetDLPF(uint8_t bw)
{
    if (MPU6050_WriteReg(MPU6050_REG_CONFIG, bw) != HAL_OK) {
        return MPU6050_ERROR;
    }
    
    return MPU6050_OK;
}

/**
 * @brief 获取温度
 */
float MPU6050_GetTemperature(void)
{
    return g_mpu6050_data.temp;
}

/**
 * @brief I2C DMA 接收完成回调
 */
void MPU6050_I2C_RxCpltCallback(void)
{
    MPU6050_ProcessDMAData();
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 写入寄存器
 */
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, 
                              MPU6050_ADDR, 
                              reg, 
                              I2C_MEMADD_SIZE_8BIT, 
                              &data, 
                              1, 
                              MPU6050_TIMEOUT);
}

/**
 * @brief 读取寄存器
 */
static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, 
                             MPU6050_ADDR, 
                             reg, 
                             I2C_MEMADD_SIZE_8BIT, 
                             data, 
                             1, 
                             MPU6050_TIMEOUT);
}

/**
 * @brief 转换 DMA 缓冲区中的原始数据
 */
static void MPU6050_ConvertRawData(void)
{
    /* 解析原始数据 (大端序) */
    int16_t accel_x = (int16_t)((s_dma_rx_buffer[0] << 8) | s_dma_rx_buffer[1]);
    int16_t accel_y = (int16_t)((s_dma_rx_buffer[2] << 8) | s_dma_rx_buffer[3]);
    int16_t accel_z = (int16_t)((s_dma_rx_buffer[4] << 8) | s_dma_rx_buffer[5]);
    int16_t temp    = (int16_t)((s_dma_rx_buffer[6] << 8) | s_dma_rx_buffer[7]);
    int16_t gyro_x  = (int16_t)((s_dma_rx_buffer[8] << 8) | s_dma_rx_buffer[9]);
    int16_t gyro_y  = (int16_t)((s_dma_rx_buffer[10] << 8) | s_dma_rx_buffer[11]);
    int16_t gyro_z  = (int16_t)((s_dma_rx_buffer[12] << 8) | s_dma_rx_buffer[13]);
    
    /* 转换为物理量并应用校准 */
    g_mpu6050_data.accel_x = (float)accel_x / s_accel_sensitivity - g_mpu6050_calib.accel_offset_x;
    g_mpu6050_data.accel_y = (float)accel_y / s_accel_sensitivity - g_mpu6050_calib.accel_offset_y;
    g_mpu6050_data.accel_z = (float)accel_z / s_accel_sensitivity - g_mpu6050_calib.accel_offset_z;
    g_mpu6050_data.temp = (float)temp / 340.0f + 36.53f;
    g_mpu6050_data.gyro_x = (float)gyro_x / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_x;
    g_mpu6050_data.gyro_y = (float)gyro_y / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_y;
    g_mpu6050_data.gyro_z = (float)gyro_z / s_gyro_sensitivity - g_mpu6050_calib.gyro_offset_z;
}