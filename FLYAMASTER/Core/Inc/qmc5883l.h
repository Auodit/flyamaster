/**
 * @file qmc5883l.h
 * @brief QMC5883L 磁力计驱动模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * QMC5883L 是一款三轴磁力计，用于航向测量。
 * 
 * 硬件连接:
 * - I2C2 (PB10/PB11) - 独立总线，避免电机干扰
 * - 地址: 0x0D
 * - 数据速率: 10-200Hz
 */

#ifndef __QMC5883L_H
#define __QMC5883L_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define QMC5883L_ADDR           (0x0D << 1)     /**< I2C 地址 (7位左移) */

/* 寄存器地址 */
#define QMC5883L_REG_DATA_X_L   0x00    /**< X 轴数据低字节 */
#define QMC5883L_REG_DATA_X_H   0x01    /**< X 轴数据高字节 */
#define QMC5883L_REG_DATA_Y_L   0x02    /**< Y 轴数据低字节 */
#define QMC5883L_REG_DATA_Y_H   0x03    /**< Y 轴数据高字节 */
#define QMC5883L_REG_DATA_Z_L   0x04    /**< Z 轴数据低字节 */
#define QMC5883L_REG_DATA_Z_H   0x05    /**< Z 轴数据高字节 */
#define QMC5883L_REG_STATUS     0x06    /**< 状态寄存器 */
#define QMC5883L_REG_TEMP_L     0x07    /**< 温度低字节 */
#define QMC5883L_REG_TEMP_H     0x08    /**< 温度高字节 */
#define QMC5883L_REG_CTRL1      0x09    /**< 控制寄存器 1 */
#define QMC5883L_REG_CTRL2      0x0A    /**< 控制寄存器 2 */
#define QMC5883L_REG_SET_RESET  0x0B    /**< SET/RESET 周期 */
#define QMC5883L_REG_CHIP_ID    0x0D    /**< 芯片 ID */

/* 控制寄存器 1 配置 */
#define QMC5883L_MODE_STANDBY   0x00    /**< 待机模式 */
#define QMC5883L_MODE_CONT      0x01    /**< 连续测量模式 */

#define QMC5883L_ODR_10HZ       0x00    /**< 10Hz 输出速率 */
#define QMC5883L_ODR_50HZ       0x04    /**< 50Hz 输出速率 */
#define QMC5883L_ODR_100HZ      0x08    /**< 100Hz 输出速率 */
#define QMC5883L_ODR_200HZ      0x0C    /**< 200Hz 输出速率 */

#define QMC5883L_RNG_2G         0x00    /**< ±2 高斯量程 */
#define QMC5883L_RNG_8G         0x10    /**< ±8 高斯量程 */

#define QMC5883L_OSR_512        0x00    /**< 过采样率 512 */
#define QMC5883L_OSR_256        0x40    /**< 过采样率 256 */
#define QMC5883L_OSR_128        0x80    /**< 过采样率 128 */
#define QMC5883L_OSR_64         0xC0    /**< 过采样率 64 */

/* 状态寄存器位 */
#define QMC5883L_STATUS_DRDY    0x01    /**< 数据就绪 */
#define QMC5883L_STATUS_OVL     0x02    /**< 数据溢出 */
#define QMC5883L_STATUS_DOR     0x04    /**< 数据跳过 */

/* 芯片 ID */
#define QMC5883L_CHIP_ID        0xFF    /**< 芯片 ID 值 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 磁力计原始数据结构体
 */
typedef struct {
    int16_t x;      /**< X 轴原始值 */
    int16_t y;      /**< Y 轴原始值 */
    int16_t z;      /**< Z 轴原始值 */
} QMC5883L_RawData_t;

/**
 * @brief 磁力计校准数据结构体
 */
typedef struct {
    int16_t offset_x;   /**< X 轴偏移 */
    int16_t offset_y;   /**< Y 轴偏移 */
    int16_t offset_z;   /**< Z 轴偏移 */
    float scale_x;      /**< X 轴缩放 */
    float scale_y;      /**< Y 轴缩放 */
    float scale_z;      /**< Z 轴缩放 */
    bool calibrated;    /**< 校准完成标志 */
} QMC5883L_Calibration_t;

/**
 * @brief 磁力计数据结构体
 */
typedef struct {
    /* 原始数据 */
    QMC5883L_RawData_t raw;
    
    /* 校准后数据 (高斯) */
    float mag_x;        /**< X 轴磁场 (Gauss) */
    float mag_y;        /**< Y 轴磁场 (Gauss) */
    float mag_z;        /**< Z 轴磁场 (Gauss) */
    
    /* 航向角 */
    float heading;      /**< 航向角 (度, 0-360) */
    float heading_filtered; /**< 滤波后航向角 */
    
    /* 温度 */
    float temperature;  /**< 温度 (°C) */
    
    /* 状态 */
    bool initialized;   /**< 初始化标志 */
    bool data_ready;    /**< 数据就绪 */
    uint32_t update_count;  /**< 更新计数 */
    
    /* 校准 */
    QMC5883L_Calibration_t calibration;
} QMC5883L_Data_t;

/* Exported variables --------------------------------------------------------*/
extern QMC5883L_Data_t g_qmc5883l_data;     /**< 全局磁力计数据 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 QMC5883L
 * @return bool true=成功, false=失败
 */
bool QMC5883L_Init(void);

/**
 * @brief 读取磁力计数据
 * @return bool true=成功
 */
bool QMC5883L_Read(void);

/**
 * @brief 使用 DMA 读取数据
 * @return bool true=成功
 */
bool QMC5883L_ReadDMA(void);

/**
 * @brief 处理 DMA 接收完成的数据
 */
void QMC5883L_ProcessDMAData(void);

/**
 * @brief 计算航向角
 * @return float 航向角 (度, 0-360)
 */
float QMC5883L_CalculateHeading(void);

/**
 * @brief 重置航向角滤波器
 * @note 在重新初始化磁力计或需要清除滤波历史时调用
 */
void QMC5883L_ResetHeadingFilter(void);

/**
 * @brief 计算带倾斜补偿的航向角
 * @param roll 横滚角 (度)
 * @param pitch 俯仰角 (度)
 * @return float 补偿后的航向角
 */
float QMC5883L_CalculateTiltCompensatedHeading(float roll, float pitch);

/**
 * @brief 开始校准
 */
void QMC5883L_StartCalibration(void);

/**
 * @brief 更新校准数据
 * @note 在校准过程中持续调用
 */
void QMC5883L_UpdateCalibration(void);

/**
 * @brief 完成校准
 */
void QMC5883L_FinishCalibration(void);

/**
 * @brief 设置校准参数
 * @param cal 校准参数
 */
void QMC5883L_SetCalibration(const QMC5883L_Calibration_t *cal);

/**
 * @brief 获取校准参数
 * @param cal 输出校准参数
 */
void QMC5883L_GetCalibration(QMC5883L_Calibration_t *cal);

/**
 * @brief 检查数据是否就绪
 * @return bool true=就绪
 */
bool QMC5883L_IsDataReady(void);

/**
 * @brief 软复位
 */
void QMC5883L_SoftReset(void);

/**
 * @brief 写寄存器
 * @param reg 寄存器地址
 * @param data 数据
 * @return bool true=成功
 */
bool QMC5883L_WriteReg(uint8_t reg, uint8_t data);

/**
 * @brief 读寄存器
 * @param reg 寄存器地址
 * @param data 输出数据
 * @return bool true=成功
 */
bool QMC5883L_ReadReg(uint8_t reg, uint8_t *data);

/**
 * @brief 读取多个寄存器
 * @param reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param len 长度
 * @return bool true=成功
 */
bool QMC5883L_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __QMC5883L_H */