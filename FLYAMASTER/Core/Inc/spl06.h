/**
 * @file spl06.h
 * @brief SPL06-001 气压计驱动模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * SPL06-001 是一款高精度数字气压计，用于高度测量。
 * 
 * 硬件连接:
 * - I2C1 (PB6/PB7) - 与 MPU6050 共享总线
 * - 地址: 0x76 或 0x77
 * - 精度: ±0.06 hPa (±0.5m)
 */

#ifndef __SPL06_H
#define __SPL06_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define SPL06_ADDR_LOW          (0x76 << 1)     /**< I2C 地址 (SDO=GND) */
#define SPL06_ADDR_HIGH         (0x77 << 1)     /**< I2C 地址 (SDO=VDD) */
#define SPL06_ADDR              SPL06_ADDR_HIGH /**< 默认地址 */

/* 寄存器地址 */
#define SPL06_REG_PSR_B2        0x00    /**< 压力数据 [23:16] */
#define SPL06_REG_PSR_B1        0x01    /**< 压力数据 [15:8] */
#define SPL06_REG_PSR_B0        0x02    /**< 压力数据 [7:0] */
#define SPL06_REG_TMP_B2        0x03    /**< 温度数据 [23:16] */
#define SPL06_REG_TMP_B1        0x04    /**< 温度数据 [15:8] */
#define SPL06_REG_TMP_B0        0x05    /**< 温度数据 [7:0] */
#define SPL06_REG_PRS_CFG       0x06    /**< 压力配置 */
#define SPL06_REG_TMP_CFG       0x07    /**< 温度配置 */
#define SPL06_REG_MEAS_CFG      0x08    /**< 测量配置 */
#define SPL06_REG_CFG_REG       0x09    /**< 中断和 FIFO 配置 */
#define SPL06_REG_INT_STS       0x0A    /**< 中断状态 */
#define SPL06_REG_FIFO_STS      0x0B    /**< FIFO 状态 */
#define SPL06_REG_RESET         0x0C    /**< 软复位 */
#define SPL06_REG_ID            0x0D    /**< 产品和版本 ID */

/* 校准系数寄存器 */
#define SPL06_REG_COEF          0x10    /**< 校准系数起始地址 (0x10-0x21) */
#define SPL06_REG_COEF_SRCE     0x28    /**< 系数来源 */

/* 产品 ID */
#define SPL06_CHIP_ID           0x10    /**< SPL06-001 产品 ID */

/* 测量模式 */
#define SPL06_MODE_IDLE         0x00    /**< 空闲模式 */
#define SPL06_MODE_PRS_SINGLE   0x01    /**< 单次压力测量 */
#define SPL06_MODE_TMP_SINGLE   0x02    /**< 单次温度测量 */
#define SPL06_MODE_PRS_CONT     0x05    /**< 连续压力测量 */
#define SPL06_MODE_TMP_CONT     0x06    /**< 连续温度测量 */
#define SPL06_MODE_CONT_BOTH    0x07    /**< 连续压力和温度测量 */

/* 过采样率 */
#define SPL06_PM_RATE_1         0x00    /**< 1 次/秒 */
#define SPL06_PM_RATE_2         0x10    /**< 2 次/秒 */
#define SPL06_PM_RATE_4         0x20    /**< 4 次/秒 */
#define SPL06_PM_RATE_8         0x30    /**< 8 次/秒 */
#define SPL06_PM_RATE_16        0x40    /**< 16 次/秒 */
#define SPL06_PM_RATE_32        0x50    /**< 32 次/秒 */
#define SPL06_PM_RATE_64        0x60    /**< 64 次/秒 */
#define SPL06_PM_RATE_128       0x70    /**< 128 次/秒 */

/* 过采样次数 */
#define SPL06_PM_PRC_1          0x00    /**< 单次 (低精度) */
#define SPL06_PM_PRC_2          0x01    /**< 2 次 */
#define SPL06_PM_PRC_4          0x02    /**< 4 次 */
#define SPL06_PM_PRC_8          0x03    /**< 8 次 */
#define SPL06_PM_PRC_16         0x04    /**< 16 次 (标准) */
#define SPL06_PM_PRC_32         0x05    /**< 32 次 */
#define SPL06_PM_PRC_64         0x06    /**< 64 次 (高精度) */
#define SPL06_PM_PRC_128        0x07    /**< 128 次 (超高精度) */

/* 状态位 */
#define SPL06_MEAS_CFG_COEF_RDY 0x80    /**< 系数就绪 */
#define SPL06_MEAS_CFG_SENSOR_RDY 0x40  /**< 传感器就绪 */
#define SPL06_MEAS_CFG_TMP_RDY  0x20    /**< 温度数据就绪 */
#define SPL06_MEAS_CFG_PRS_RDY  0x10    /**< 压力数据就绪 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 校准系数结构体
 */
typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} SPL06_Coef_t;

/**
 * @brief 气压计数据结构体
 */
typedef struct {
    /* 原始数据 */
    int32_t raw_pressure;       /**< 原始压力值 */
    int32_t raw_temperature;    /**< 原始温度值 */
    
    /* 补偿后数据 */
    float pressure;             /**< 气压 (Pa) */
    float temperature;          /**< 温度 (°C) */
    
    /* 高度数据 */
    float altitude;             /**< 相对高度 (m) */
    float altitude_filtered;    /**< 滤波后高度 */
    float vertical_speed;       /**< 垂直速度 (m/s) */
    
    /* 参考值 */
    float sea_level_pressure;   /**< 海平面气压 (Pa) */
    float ground_pressure;      /**< 地面气压 (Pa) */
    float ground_altitude;      /**< 地面高度 (m) */
    
    /* 校准系数 */
    SPL06_Coef_t coef;
    
    /* 缩放因子 */
    float kP;                   /**< 压力缩放因子 */
    float kT;                   /**< 温度缩放因子 */
    
    /* 状态 */
    bool initialized;           /**< 初始化标志 */
    bool data_ready;            /**< 数据就绪 */
    uint32_t update_count;      /**< 更新计数 */
    uint32_t last_update_time;  /**< 上次更新时间 */
} SPL06_Data_t;

/* Exported variables --------------------------------------------------------*/
extern SPL06_Data_t g_spl06_data;       /**< 全局气压计数据 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SPL06
 * @return bool true=成功, false=失败
 */
bool SPL06_Init(void);

/**
 * @brief 读取气压和温度数据
 * @return bool true=成功
 */
bool SPL06_Read(void);

/**
 * @brief 使用 DMA 读取数据
 * @return bool true=成功
 */
bool SPL06_ReadDMA(void);

/**
 * @brief 处理 DMA 接收完成的数据
 */
void SPL06_ProcessDMAData(void);

/**
 * @brief 计算高度
 * @return float 相对高度 (m)
 */
float SPL06_CalculateAltitude(void);

/**
 * @brief 重置高度滤波器
 * @note 在重新初始化气压计或需要清除滤波历史时调用
 */
void SPL06_ResetAltitudeFilter(void);

/**
 * @brief 设置地面参考点
 * @note 在起飞前调用，将当前位置设为零点
 */
void SPL06_SetGroundReference(void);

/**
 * @brief 设置海平面气压
 * @param pressure 海平面气压 (Pa)
 */
void SPL06_SetSeaLevelPressure(float pressure);

/**
 * @brief 获取垂直速度
 * @return float 垂直速度 (m/s, 正值为上升)
 */
float SPL06_GetVerticalSpeed(void);

/**
 * @brief 检查数据是否就绪
 * @return bool true=就绪
 */
bool SPL06_IsDataReady(void);

/**
 * @brief 软复位
 */
void SPL06_SoftReset(void);

/**
 * @brief 写寄存器
 * @param reg 寄存器地址
 * @param data 数据
 * @return bool true=成功
 */
bool SPL06_WriteReg(uint8_t reg, uint8_t data);

/**
 * @brief 读寄存器
 * @param reg 寄存器地址
 * @param data 输出数据
 * @return bool true=成功
 */
bool SPL06_ReadReg(uint8_t reg, uint8_t *data);

/**
 * @brief 读取多个寄存器
 * @param reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param len 长度
 * @return bool true=成功
 */
bool SPL06_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SPL06_H */