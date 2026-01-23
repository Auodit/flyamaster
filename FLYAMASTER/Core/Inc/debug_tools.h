/**
 * @file debug_tools.h
 * @brief 调试工具模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 提供 I2C 总线扫描、磁力计电流补偿标定等调试功能。
 */

#ifndef __DEBUG_TOOLS_H
#define __DEBUG_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* 已知 I2C 设备地址 */
#define I2C_ADDR_MPU6050        0x68
#define I2C_ADDR_MPU6050_ALT    0x69
#define I2C_ADDR_QMC5883L       0x0D
#define I2C_ADDR_HMC5883L       0x1E
#define I2C_ADDR_SPL06          0x77
#define I2C_ADDR_SPL06_ALT      0x76
#define I2C_ADDR_BMP280         0x76
#define I2C_ADDR_BMP280_ALT     0x77
#define I2C_ADDR_MS5611         0x77
#define I2C_ADDR_EEPROM         0x50

/* Exported types ------------------------------------------------------------*/

/**
 * @brief I2C 扫描结果
 */
typedef struct {
    uint8_t address;        /**< 设备地址 */
    const char *name;       /**< 设备名称 */
    bool found;             /**< 是否找到 */
} I2C_DeviceInfo_t;

/**
 * @brief 磁力计电流补偿系数
 */
typedef struct {
    float k_x;              /**< X 轴补偿系数 */
    float k_y;              /**< Y 轴补偿系数 */
    float k_z;              /**< Z 轴补偿系数 */
    bool calibrated;        /**< 是否已校准 */
} MagCurrentCalib_t;

/* Exported variables --------------------------------------------------------*/
extern MagCurrentCalib_t g_mag_current_calib;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 扫描 I2C 总线
 * @param hi2c I2C 句柄
 * @return uint8_t 找到的设备数量
 */
uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c);

/**
 * @brief 扫描所有 I2C 总线
 */
void I2C_ScanAllBuses(void);

/**
 * @brief 获取设备名称
 * @param address 设备地址
 * @return const char* 设备名称
 */
const char* I2C_GetDeviceName(uint8_t address);

/**
 * @brief 开始磁力计电流补偿标定
 */
void MagCurrentCalib_Start(void);

/**
 * @brief 采集低油门数据
 * @note 在油门最低时调用
 */
void MagCurrentCalib_SampleLow(void);

/**
 * @brief 采集高油门数据
 * @note 在油门最高时调用
 */
void MagCurrentCalib_SampleHigh(void);

/**
 * @brief 完成标定并计算系数
 */
void MagCurrentCalib_Finish(void);

/**
 * @brief 打印标定结果
 */
void MagCurrentCalib_PrintResult(void);

/**
 * @brief 应用电流补偿
 * @param mag_x 原始 X 轴值
 * @param mag_y 原始 Y 轴值
 * @param mag_z 原始 Z 轴值
 * @param throttle 油门值 (0.0 ~ 1.0)
 */
void MagCurrentCalib_Apply(float *mag_x, float *mag_y, float *mag_z, float throttle);

/**
 * @brief 打印系统状态
 */
void Debug_PrintSystemStatus(void);

/**
 * @brief 打印任务状态
 */
void Debug_PrintTaskStatus(void);

/**
 * @brief 打印内存使用情况
 */
void Debug_PrintMemoryUsage(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_TOOLS_H */