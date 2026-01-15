/**
 * @file spl06.h
 * @brief SPL06 气压计驱动头文件
 * @details 通过 I2C2 读取气压和温度数据，计算海拔高度
 * @author AI Assistant
 * @date 2026-01-14
 */

#ifndef __SPL06_H
#define __SPL06_H

#include <stdint.h>
#include "main.h"

/*==============================================================================
 * I2C 地址
 *============================================================================*/
#define SPL06_ADDR              (0x76 << 1)  // SDO=GND: 0x76, SDO=VDD: 0x77
#define SPL06_ADDR_ALT          (0x77 << 1)  // 备用地址

/*==============================================================================
 * 寄存器地址
 *============================================================================*/
#define SPL06_REG_PSR_B2        0x00    // 压力数据 [23:16]
#define SPL06_REG_PSR_B1        0x01    // 压力数据 [15:8]
#define SPL06_REG_PSR_B0        0x02    // 压力数据 [7:0]
#define SPL06_REG_TMP_B2        0x03    // 温度数据 [23:16]
#define SPL06_REG_TMP_B1        0x04    // 温度数据 [15:8]
#define SPL06_REG_TMP_B0        0x05    // 温度数据 [7:0]
#define SPL06_REG_PRS_CFG       0x06    // 压力配置
#define SPL06_REG_TMP_CFG       0x07    // 温度配置
#define SPL06_REG_MEAS_CFG      0x08    // 测量配置
#define SPL06_REG_CFG_REG       0x09    // 配置寄存器
#define SPL06_REG_INT_STS       0x0A    // 中断状态
#define SPL06_REG_FIFO_STS      0x0B    // FIFO 状态
#define SPL06_REG_RESET         0x0C    // 软复位
#define SPL06_REG_ID            0x0D    // 产品 ID (应为 0x10)

// 校准系数寄存器 (18 字节)
#define SPL06_REG_COEF          0x10    // 0x10 ~ 0x21

/*==============================================================================
 * 配置常量
 *============================================================================*/
// 采样率配置 (PRS_CFG / TMP_CFG 的高 3 位)
#define SPL06_RATE_1            0x00    // 1 次/秒
#define SPL06_RATE_2            0x10    // 2 次/秒
#define SPL06_RATE_4            0x20    // 4 次/秒
#define SPL06_RATE_8            0x30    // 8 次/秒
#define SPL06_RATE_16           0x40    // 16 次/秒
#define SPL06_RATE_32           0x50    // 32 次/秒
#define SPL06_RATE_64           0x60    // 64 次/秒
#define SPL06_RATE_128          0x70    // 128 次/秒

// 过采样配置 (PRS_CFG / TMP_CFG 的低 4 位)
#define SPL06_OSR_1             0x00    // 单次
#define SPL06_OSR_2             0x01    // 2 倍过采样
#define SPL06_OSR_4             0x02    // 4 倍过采样
#define SPL06_OSR_8             0x03    // 8 倍过采样
#define SPL06_OSR_16            0x04    // 16 倍过采样
#define SPL06_OSR_32            0x05    // 32 倍过采样
#define SPL06_OSR_64            0x06    // 64 倍过采样
#define SPL06_OSR_128           0x07    // 128 倍过采样

// 测量模式 (MEAS_CFG 的低 3 位)
#define SPL06_MODE_IDLE         0x00    // 空闲模式
#define SPL06_MODE_PRESSURE     0x01    // 单次压力测量
#define SPL06_MODE_TEMPERATURE  0x02    // 单次温度测量
#define SPL06_MODE_CONTINUOUS   0x07    // 连续测量 (压力+温度)

// 产品 ID
#define SPL06_PRODUCT_ID        0x10

/*==============================================================================
 * 数据结构
 *============================================================================*/
typedef struct {
    // 校准系数 (从芯片读取)
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    
    // 测量结果
    float pressure;         // 气压 (Pa)
    float temperature;      // 温度 (°C)
    float altitude;         // 海拔高度 (m)
    
    // 参考值 (用于相对高度计算)
    float pressure_ref;     // 参考气压 (上电时的气压)
    float altitude_ref;     // 参考高度 (设为 0)
    
    // 状态标志
    uint8_t initialized;    // 初始化完成标志
    uint8_t data_ready;     // 数据就绪标志
    uint32_t last_update;   // 最后更新时间 (ms)
} SPL06_TypeDef;

/*==============================================================================
 * 全局变量
 *============================================================================*/
// Risk #066 修复：添加 volatile 修饰符（RTOS多任务共享变量）
extern volatile SPL06_TypeDef SPL06_Data;

/*==============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief 初始化 SPL06 气压计
 * @param hi2c I2C 句柄 (使用 hi2c2)
 * @return 0: 成功, 1: 失败
 */
uint8_t SPL06_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取气压和温度原始数据
 * @param hi2c I2C 句柄
 * @return 0: 成功, 1: 失败
 */
uint8_t SPL06_ReadRawData(I2C_HandleTypeDef *hi2c);

/**
 * @brief 计算补偿后的气压和温度
 * @param spl06 数据结构指针
 * @param raw_pressure 原始气压值
 * @param raw_temperature 原始温度值
 */
void SPL06_Compensate(volatile SPL06_TypeDef *spl06, int32_t raw_pressure, int32_t raw_temperature);

/**
 * @brief 根据气压计算海拔高度
 * @param pressure 当前气压 (Pa)
 * @param pressure_ref 参考气压 (Pa)
 * @return 相对高度 (m)
 */
float SPL06_PressureToAltitude(float pressure, float pressure_ref);

/**
 * @brief 获取当前海拔高度
 * @param spl06 数据结构指针
 * @return 相对高度 (m), 相对于参考点
 */
float SPL06_GetAltitude(volatile SPL06_TypeDef *spl06);

/**
 * @brief 设置参考高度 (零点校准)
 * @param spl06 数据结构指针
 */
void SPL06_SetReference(volatile SPL06_TypeDef *spl06);

/**
 * @brief 检查数据是否就绪
 * @param hi2c I2C 句柄
 * @return 1: 就绪, 0: 未就绪
 */
uint8_t SPL06_IsDataReady(I2C_HandleTypeDef *hi2c);

/**
 * @brief 软复位 SPL06
 * @param hi2c I2C 句柄
 */
void SPL06_Reset(I2C_HandleTypeDef *hi2c);

#endif /* __SPL06_H */
