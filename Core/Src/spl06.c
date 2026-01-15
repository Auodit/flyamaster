/**
 * @file spl06.c
 * @brief SPL06 气压计驱动实现
 * @details 通过 I2C2 读取气压和温度数据，计算海拔高度
 * @author AI Assistant
 * @date 2026-01-14
 */

#include "spl06.h"
#include <math.h>

/*==============================================================================
 * 全局变量
 *============================================================================*/
// Risk #066 修复：添加 volatile 修饰符（RTOS多任务共享变量）
volatile SPL06_TypeDef SPL06_Data = {0};

/*==============================================================================
 * 内部变量
 *============================================================================*/
// 过采样补偿因子 (根据 OSR 配置选择)
static const int32_t kT = 524288;   // OSR=1
static const int32_t kP = 524288;   // OSR=1

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 读取单个寄存器
 */
static uint8_t SPL06_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
    if (hi2c == NULL || data == NULL) return 1;
    
    if (HAL_I2C_Mem_Read(hi2c, SPL06_ADDR, reg, I2C_MEMADD_SIZE_8BIT, 
                         data, 1, 100) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 写入单个寄存器
 */
static uint8_t SPL06_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    if (hi2c == NULL) return 1;
    
    if (HAL_I2C_Mem_Write(hi2c, SPL06_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, 100) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 读取多个寄存器
 */
static uint8_t SPL06_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (hi2c == NULL || data == NULL) return 1;
    
    if (HAL_I2C_Mem_Read(hi2c, SPL06_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, 50) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 读取校准系数
 */
static uint8_t SPL06_ReadCoefficients(I2C_HandleTypeDef *hi2c, volatile SPL06_TypeDef *spl06)
{
    uint8_t coef[18];
    
    // Risk #065 修复：添加指针空检查
    if (hi2c == NULL || spl06 == NULL) return 1;
    
    if (SPL06_ReadRegs(hi2c, SPL06_REG_COEF, coef, 18) != 0) {
        return 1;
    }
    
    // 解析校准系数 (参考 SPL06 Datasheet)
    spl06->c0 = (int16_t)(((uint16_t)coef[0] << 4) | ((coef[1] >> 4) & 0x0F));
    if (spl06->c0 & 0x0800) spl06->c0 |= 0xF000;  // 符号扩展
    
    spl06->c1 = (int16_t)(((uint16_t)(coef[1] & 0x0F) << 8) | coef[2]);
    if (spl06->c1 & 0x0800) spl06->c1 |= 0xF000;
    
    spl06->c00 = (int32_t)(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | ((coef[5] >> 4) & 0x0F));
    if (spl06->c00 & 0x080000) spl06->c00 |= 0xFFF00000;  // 20 位符号扩展
    
    spl06->c10 = (int32_t)(((uint32_t)(coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7]);
    if (spl06->c10 & 0x080000) spl06->c10 |= 0xFFF00000;
    
    spl06->c01 = (int16_t)(((uint16_t)coef[8] << 8) | coef[9]);
    spl06->c11 = (int16_t)(((uint16_t)coef[10] << 8) | coef[11]);
    spl06->c20 = (int16_t)(((uint16_t)coef[12] << 8) | coef[13]);
    spl06->c21 = (int16_t)(((uint16_t)coef[14] << 8) | coef[15]);
    spl06->c30 = (int16_t)(((uint16_t)coef[16] << 8) | coef[17]);
    
    return 0;
}

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化 SPL06 气压计
 */
uint8_t SPL06_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    
    if (hi2c == NULL) return 1;
    
    // 软复位
    SPL06_Reset(hi2c);
    HAL_Delay(50);  // 等待复位完成
    
    // 读取产品 ID
    if (SPL06_ReadReg(hi2c, SPL06_REG_ID, &id) != 0) {
        return 1;
    }
    
    // 验证产品 ID
    if (id != SPL06_PRODUCT_ID) {
        return 1;  // 不是 SPL06
    }
    
    // 读取校准系数
    if (SPL06_ReadCoefficients(hi2c, &SPL06_Data) != 0) {
        return 1;
    }
    
    // 配置压力测量: 32Hz 采样率, 单次过采样 (低功耗)
    if (SPL06_WriteReg(hi2c, SPL06_REG_PRS_CFG, SPL06_RATE_32 | SPL06_OSR_1) != 0) {
        return 1;
    }
    
    // 配置温度测量: 32Hz 采样率, 单次过采样
    // 注意: TMP_CFG 的 bit7 必须设为 1 (使用内部温度传感器)
    if (SPL06_WriteReg(hi2c, SPL06_REG_TMP_CFG, 0x80 | SPL06_RATE_32 | SPL06_OSR_1) != 0) {
        return 1;
    }
    
    // 配置 FIFO 和中断 (禁用 FIFO, 禁用中断)
    if (SPL06_WriteReg(hi2c, SPL06_REG_CFG_REG, 0x00) != 0) {
        return 1;
    }
    
    // 启动连续测量模式 (压力 + 温度)
    if (SPL06_WriteReg(hi2c, SPL06_REG_MEAS_CFG, SPL06_MODE_CONTINUOUS) != 0) {
        return 1;
    }
    
    HAL_Delay(100);  // 等待首次测量完成
    
    // 读取初始数据并设置参考值
    if (SPL06_ReadRawData(hi2c) == 0) {
        SPL06_SetReference(&SPL06_Data);
        SPL06_Data.initialized = 1;
    }
    
    return 0;
}

/**
 * @brief 读取气压和温度原始数据
 */
uint8_t SPL06_ReadRawData(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6];
    int32_t raw_pressure, raw_temperature;
    
    if (hi2c == NULL) return 1;
    
    // 读取 6 字节数据 (压力 3 字节 + 温度 3 字节)
    if (SPL06_ReadRegs(hi2c, SPL06_REG_PSR_B2, buf, 6) != 0) {
        return 1;
    }
    
    // 解析原始压力值 (24 位有符号)
    raw_pressure = (int32_t)(((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]);
    if (raw_pressure & 0x800000) {
        raw_pressure |= 0xFF000000;  // 符号扩展
    }
    
    // 解析原始温度值 (24 位有符号)
    raw_temperature = (int32_t)(((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5]);
    if (raw_temperature & 0x800000) {
        raw_temperature |= 0xFF000000;
    }
    
    // 计算补偿后的值
    SPL06_Compensate(&SPL06_Data, raw_pressure, raw_temperature);
    
    // 更新高度
    SPL06_Data.altitude = SPL06_PressureToAltitude(SPL06_Data.pressure, SPL06_Data.pressure_ref);
    
    SPL06_Data.data_ready = 1;
    SPL06_Data.last_update = HAL_GetTick();
    
    return 0;
}

/**
 * @brief 计算补偿后的气压和温度
 * @details 使用 SPL06 Datasheet 中的补偿公式
 */
void SPL06_Compensate(volatile SPL06_TypeDef *spl06, int32_t raw_pressure, int32_t raw_temperature)
{
    if (spl06 == NULL) return;
    
    // 缩放原始值
    float Traw_sc = (float)raw_temperature / (float)kT;
    float Praw_sc = (float)raw_pressure / (float)kP;
    
    // 计算补偿温度 (°C)
    spl06->temperature = (float)spl06->c0 * 0.5f + (float)spl06->c1 * Traw_sc;
    
    // 计算补偿气压 (Pa)
    spl06->pressure = (float)spl06->c00 
                    + Praw_sc * ((float)spl06->c10 + Praw_sc * ((float)spl06->c20 + Praw_sc * (float)spl06->c30))
                    + Traw_sc * (float)spl06->c01 
                    + Traw_sc * Praw_sc * ((float)spl06->c11 + Praw_sc * (float)spl06->c21);
}

/**
 * @brief 根据气压计算海拔高度
 * @details 使用国际气压高度公式
 *          h = 44330 * (1 - (P/P0)^0.1903)
 */
float SPL06_PressureToAltitude(float pressure, float pressure_ref)
{
    if (pressure <= 0 || pressure_ref <= 0) return 0.0f;
    
    return 44330.0f * (1.0f - powf(pressure / pressure_ref, 0.1903f));
}

/**
 * @brief 获取当前海拔高度 (相对于参考点)
 */
float SPL06_GetAltitude(volatile SPL06_TypeDef *spl06)
{
    if (spl06 == NULL || !spl06->initialized) return 0.0f;
    
    return spl06->altitude - spl06->altitude_ref;
}

/**
 * @brief 设置参考高度 (零点校准)
 */
void SPL06_SetReference(volatile SPL06_TypeDef *spl06)
{
    if (spl06 == NULL) return;
    
    spl06->pressure_ref = spl06->pressure;
    spl06->altitude_ref = spl06->altitude;
}

/**
 * @brief 检查数据是否就绪
 */
uint8_t SPL06_IsDataReady(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    
    if (hi2c == NULL) return 0;
    
    if (SPL06_ReadReg(hi2c, SPL06_REG_MEAS_CFG, &status) != 0) {
        return 0;
    }
    
    // bit 6: PRS_RDY, bit 5: TMP_RDY
    return ((status & 0x60) == 0x60) ? 1 : 0;
}

/**
 * @brief 软复位 SPL06
 */
void SPL06_Reset(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return;
    
    SPL06_WriteReg(hi2c, SPL06_REG_RESET, 0x89);  // 软复位命令
}
