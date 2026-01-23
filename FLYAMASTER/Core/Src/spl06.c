/**
 * @file spl06.c
 * @brief SPL06-001 气压计驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "spl06.h"
#include "i2c.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

/* I2C1 互斥锁 (在 freertos.c 中定义) */
extern osMutexId_t i2c1MutexHandle;

/* 互斥锁超时时间 (ms) */
#define I2C1_MUTEX_TIMEOUT  100

/* Private defines -----------------------------------------------------------*/
#define SPL06_TIMEOUT           100     /**< I2C 超时 (ms) */

/* 标准大气参数 */
#define SEA_LEVEL_PRESSURE      101325.0f   /**< 标准海平面气压 (Pa) */
#define TEMP_LAPSE_RATE         0.0065f     /**< 温度递减率 (K/m) */
#define SEA_LEVEL_TEMP          288.15f     /**< 标准海平面温度 (K) */
#define GAS_CONSTANT            8.31447f    /**< 气体常数 */
#define MOLAR_MASS_AIR          0.0289644f  /**< 空气摩尔质量 (kg/mol) */
#define GRAVITY                 9.80665f    /**< 重力加速度 (m/s²) */

/* 低通滤波系数 */
#define ALTITUDE_LPF_ALPHA      0.1f

/* Private variables ---------------------------------------------------------*/
SPL06_Data_t g_spl06_data = {0};

/* DMA 接收缓冲区 */
static uint8_t spl_dma_buffer[6];

/* 上一次高度值 (用于计算垂直速度) */
static float prev_altitude = 0.0f;
static uint32_t prev_time = 0;

/* 缩放因子查找表 */
static const float scale_factors[] = {
    524288.0f,  /* 1 次过采样 */
    1572864.0f, /* 2 次过采样 */
    3670016.0f, /* 4 次过采样 */
    7864320.0f, /* 8 次过采样 */
    253952.0f,  /* 16 次过采样 */
    516096.0f,  /* 32 次过采样 */
    1040384.0f, /* 64 次过采样 */
    2088960.0f  /* 128 次过采样 */
};

/* Private function prototypes -----------------------------------------------*/
static bool SPL06_ReadCoefficients(void);
static void SPL06_CalculateCompensated(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 写寄存器 (带互斥锁保护)
 */
bool SPL06_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return false;
        }
    }
    
    status = HAL_I2C_Mem_Write(&hi2c1, SPL06_ADDR, reg,
                               I2C_MEMADD_SIZE_8BIT, &data, 1, SPL06_TIMEOUT);
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
    
    return (status == HAL_OK);
}

/**
 * @brief 读寄存器 (带互斥锁保护)
 */
bool SPL06_ReadReg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return false;
        }
    }
    
    status = HAL_I2C_Mem_Read(&hi2c1, SPL06_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, data, 1, SPL06_TIMEOUT);
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
    
    return (status == HAL_OK);
}

/**
 * @brief 读取多个寄存器 (带互斥锁保护)
 */
bool SPL06_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return false;
        }
    }
    
    status = HAL_I2C_Mem_Read(&hi2c1, SPL06_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, data, len, SPL06_TIMEOUT);
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
    
    return (status == HAL_OK);
}

/**
 * @brief 软复位
 */
void SPL06_SoftReset(void)
{
    SPL06_WriteReg(SPL06_REG_RESET, 0x89);  /* 软复位命令 */
    HAL_Delay(50);
}

/**
 * @brief 读取校准系数
 */
static bool SPL06_ReadCoefficients(void)
{
    uint8_t coef[18];
    
    /* 读取 18 字节校准系数 */
    if (!SPL06_ReadRegs(SPL06_REG_COEF, coef, 18)) {
        return false;
    }
    
    /* 解析系数 (参考数据手册) */
    g_spl06_data.coef.c0 = ((int16_t)coef[0] << 4) | ((coef[1] >> 4) & 0x0F);
    if (g_spl06_data.coef.c0 & 0x0800) {
        g_spl06_data.coef.c0 |= 0xF000;  /* 符号扩展 */
    }
    
    g_spl06_data.coef.c1 = ((int16_t)(coef[1] & 0x0F) << 8) | coef[2];
    if (g_spl06_data.coef.c1 & 0x0800) {
        g_spl06_data.coef.c1 |= 0xF000;
    }
    
    g_spl06_data.coef.c00 = ((int32_t)coef[3] << 12) | ((int32_t)coef[4] << 4) | ((coef[5] >> 4) & 0x0F);
    if (g_spl06_data.coef.c00 & 0x080000) {
        g_spl06_data.coef.c00 |= 0xFFF00000;
    }
    
    g_spl06_data.coef.c10 = ((int32_t)(coef[5] & 0x0F) << 16) | ((int32_t)coef[6] << 8) | coef[7];
    if (g_spl06_data.coef.c10 & 0x080000) {
        g_spl06_data.coef.c10 |= 0xFFF00000;
    }
    
    g_spl06_data.coef.c01 = ((int16_t)coef[8] << 8) | coef[9];
    g_spl06_data.coef.c11 = ((int16_t)coef[10] << 8) | coef[11];
    g_spl06_data.coef.c20 = ((int16_t)coef[12] << 8) | coef[13];
    g_spl06_data.coef.c21 = ((int16_t)coef[14] << 8) | coef[15];
    g_spl06_data.coef.c30 = ((int16_t)coef[16] << 8) | coef[17];
    
    return true;
}

/**
 * @brief 初始化 SPL06
 */
bool SPL06_Init(void)
{
    uint8_t chip_id;
    uint8_t meas_cfg;
    uint32_t timeout;
    
    /* 清空数据结构 */
    memset(&g_spl06_data, 0, sizeof(g_spl06_data));
    
    /* 默认参考气压 */
    g_spl06_data.sea_level_pressure = SEA_LEVEL_PRESSURE;
    
    /* 软复位 */
    SPL06_SoftReset();
    
    /* 读取芯片 ID */
    if (!SPL06_ReadReg(SPL06_REG_ID, &chip_id)) {
        return false;
    }
    
    /* 验证芯片 ID */
    if ((chip_id & 0xF0) != SPL06_CHIP_ID) {
        return false;
    }
    
    /* 等待传感器就绪 */
    timeout = HAL_GetTick() + 100;
    do {
        if (!SPL06_ReadReg(SPL06_REG_MEAS_CFG, &meas_cfg)) {
            return false;
        }
        if (HAL_GetTick() > timeout) {
            return false;
        }
    } while (!(meas_cfg & SPL06_MEAS_CFG_SENSOR_RDY));
    
    /* 等待系数就绪 */
    timeout = HAL_GetTick() + 100;
    do {
        if (!SPL06_ReadReg(SPL06_REG_MEAS_CFG, &meas_cfg)) {
            return false;
        }
        if (HAL_GetTick() > timeout) {
            return false;
        }
    } while (!(meas_cfg & SPL06_MEAS_CFG_COEF_RDY));
    
    /* 读取校准系数 */
    if (!SPL06_ReadCoefficients()) {
        return false;
    }
    
    /* 配置压力测量:
     * - 32 次/秒采样率
     * - 16 次过采样 (标准精度)
     */
    if (!SPL06_WriteReg(SPL06_REG_PRS_CFG, SPL06_PM_RATE_32 | SPL06_PM_PRC_16)) {
        return false;
    }
    g_spl06_data.kP = scale_factors[4];  /* 16 次过采样 */
    
    /* 配置温度测量:
     * - 32 次/秒采样率
     * - 8 次过采样
     * - 使用外部传感器
     */
    if (!SPL06_WriteReg(SPL06_REG_TMP_CFG, 0x80 | SPL06_PM_RATE_32 | SPL06_PM_PRC_8)) {
        return false;
    }
    g_spl06_data.kT = scale_factors[3];  /* 8 次过采样 */
    
    /* 配置 CFG_REG:
     * - 启用压力和温度结果位移 (过采样 > 8 时需要)
     */
    if (!SPL06_WriteReg(SPL06_REG_CFG_REG, 0x04 | 0x08)) {
        return false;
    }
    
    /* 启动连续测量模式 */
    if (!SPL06_WriteReg(SPL06_REG_MEAS_CFG, SPL06_MODE_CONT_BOTH)) {
        return false;
    }
    
    /* 等待首次数据就绪 */
    HAL_Delay(50);
    
    g_spl06_data.initialized = true;
    
    /* 设置地面参考 */
    SPL06_Read();
    SPL06_SetGroundReference();
    
    return true;
}

/**
 * @brief 检查数据是否就绪
 */
bool SPL06_IsDataReady(void)
{
    uint8_t status;
    if (!SPL06_ReadReg(SPL06_REG_MEAS_CFG, &status)) {
        return false;
    }
    return (status & (SPL06_MEAS_CFG_PRS_RDY | SPL06_MEAS_CFG_TMP_RDY)) == 
           (SPL06_MEAS_CFG_PRS_RDY | SPL06_MEAS_CFG_TMP_RDY);
}

/**
 * @brief 读取气压和温度数据 (阻塞方式)
 */
bool SPL06_Read(void)
{
    uint8_t buffer[6];
    
    if (!g_spl06_data.initialized) {
        return false;
    }
    
    /* 读取 6 字节数据 (PSR_B2, PSR_B1, PSR_B0, TMP_B2, TMP_B1, TMP_B0) */
    if (!SPL06_ReadRegs(SPL06_REG_PSR_B2, buffer, 6)) {
        return false;
    }
    
    /* 解析原始压力数据 (24位有符号) */
    g_spl06_data.raw_pressure = ((int32_t)buffer[0] << 16) | 
                                 ((int32_t)buffer[1] << 8) | 
                                 buffer[2];
    if (g_spl06_data.raw_pressure & 0x800000) {
        g_spl06_data.raw_pressure |= 0xFF000000;  /* 符号扩展 */
    }
    
    /* 解析原始温度数据 (24位有符号) */
    g_spl06_data.raw_temperature = ((int32_t)buffer[3] << 16) | 
                                    ((int32_t)buffer[4] << 8) | 
                                    buffer[5];
    if (g_spl06_data.raw_temperature & 0x800000) {
        g_spl06_data.raw_temperature |= 0xFF000000;
    }
    
    /* 计算补偿后的值 */
    SPL06_CalculateCompensated();
    
    g_spl06_data.data_ready = true;
    g_spl06_data.update_count++;
    g_spl06_data.last_update_time = HAL_GetTick();
    
    return true;
}

/**
 * @brief 使用 DMA 读取数据 (带互斥锁保护)
 */
bool SPL06_ReadDMA(void)
{
    HAL_StatusTypeDef status;
    
    if (!g_spl06_data.initialized) {
        return false;
    }
    
    /* 获取 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        if (osMutexAcquire(i2c1MutexHandle, I2C1_MUTEX_TIMEOUT) != osOK) {
            return false;
        }
    }
    
    /* 启动 DMA 读取 */
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, SPL06_ADDR,
                                  SPL06_REG_PSR_B2,
                                  I2C_MEMADD_SIZE_8BIT,
                                  spl_dma_buffer, 6);
    
    if (status != HAL_OK) {
        if (i2c1MutexHandle != NULL) {
            osMutexRelease(i2c1MutexHandle);
        }
        return false;
    }
    
    /* 注意：互斥锁将在 DMA 完成回调中释放 */
    return true;
}

/**
 * @brief 处理 DMA 接收完成的数据
 */
void SPL06_ProcessDMAData(void)
{
    /* 解析原始压力数据 */
    g_spl06_data.raw_pressure = ((int32_t)spl_dma_buffer[0] << 16) |
                                 ((int32_t)spl_dma_buffer[1] << 8) |
                                 spl_dma_buffer[2];
    if (g_spl06_data.raw_pressure & 0x800000) {
        g_spl06_data.raw_pressure |= 0xFF000000;
    }
    
    /* 解析原始温度数据 */
    g_spl06_data.raw_temperature = ((int32_t)spl_dma_buffer[3] << 16) |
                                    ((int32_t)spl_dma_buffer[4] << 8) |
                                    spl_dma_buffer[5];
    if (g_spl06_data.raw_temperature & 0x800000) {
        g_spl06_data.raw_temperature |= 0xFF000000;
    }
    
    /* 计算补偿后的值 */
    SPL06_CalculateCompensated();
    
    g_spl06_data.data_ready = true;
    g_spl06_data.update_count++;
    g_spl06_data.last_update_time = HAL_GetTick();
    
    /* 释放 I2C1 互斥锁 */
    if (i2c1MutexHandle != NULL) {
        osMutexRelease(i2c1MutexHandle);
    }
}

/**
 * @brief 计算补偿后的压力和温度
 */
static void SPL06_CalculateCompensated(void)
{
    SPL06_Coef_t *c = &g_spl06_data.coef;
    
    /* 缩放原始值 */
    float Traw_sc = (float)g_spl06_data.raw_temperature / g_spl06_data.kT;
    float Praw_sc = (float)g_spl06_data.raw_pressure / g_spl06_data.kP;
    
    /* 计算补偿温度 (°C) */
    g_spl06_data.temperature = (float)c->c0 * 0.5f + (float)c->c1 * Traw_sc;
    
    /* 计算补偿压力 (Pa) */
    g_spl06_data.pressure = (float)c->c00 + 
                            Praw_sc * ((float)c->c10 + Praw_sc * ((float)c->c20 + Praw_sc * (float)c->c30)) +
                            Traw_sc * (float)c->c01 +
                            Traw_sc * Praw_sc * ((float)c->c11 + Praw_sc * (float)c->c21);
    
    /* 计算高度 */
    SPL06_CalculateAltitude();
}

/* 高度滤波器状态 (模块级静态变量，支持重置) */
static float s_altitude_filtered = 0.0f;
static bool s_altitude_first_run = true;

/**
 * @brief 计算高度
 * @return float 相对高度 (m)
 */
float SPL06_CalculateAltitude(void)
{
    float altitude;
    
    /* 使用气压高度公式 (国际标准大气模型) */
    /* h = (T0/L) * (1 - (P/P0)^(R*L/(g*M))) */
    /* 简化公式: h = 44330 * (1 - (P/P0)^0.1903) */
    
    float pressure_ratio = g_spl06_data.pressure / g_spl06_data.sea_level_pressure;
    altitude = 44330.0f * (1.0f - powf(pressure_ratio, 0.1903f));
    
    /* 计算相对高度 */
    g_spl06_data.altitude = altitude - g_spl06_data.ground_altitude;
    
    /* 低通滤波 */
    if (s_altitude_first_run) {
        s_altitude_filtered = g_spl06_data.altitude;
        s_altitude_first_run = false;
    } else {
        s_altitude_filtered = s_altitude_filtered * (1.0f - ALTITUDE_LPF_ALPHA) +
                              g_spl06_data.altitude * ALTITUDE_LPF_ALPHA;
    }
    
    g_spl06_data.altitude_filtered = s_altitude_filtered;
    
    /* 计算垂直速度 */
    uint32_t current_time = HAL_GetTick();
    if (prev_time > 0) {
        float dt = (float)(current_time - prev_time) / 1000.0f;
        if (dt > 0.001f) {
            g_spl06_data.vertical_speed = (s_altitude_filtered - prev_altitude) / dt;
        }
    }
    prev_altitude = s_altitude_filtered;
    prev_time = current_time;
    
    return g_spl06_data.altitude;
}

/**
 * @brief 重置高度滤波器
 * @note 在重新初始化气压计或需要清除滤波历史时调用
 */
void SPL06_ResetAltitudeFilter(void)
{
    s_altitude_filtered = 0.0f;
    s_altitude_first_run = true;
    prev_altitude = 0.0f;
    prev_time = 0;
}

/**
 * @brief 设置地面参考点
 */
void SPL06_SetGroundReference(void)
{
    /* 多次采样取平均 */
    float sum_pressure = 0.0f;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        SPL06_Read();
        sum_pressure += g_spl06_data.pressure;
        HAL_Delay(20);
    }
    
    g_spl06_data.ground_pressure = sum_pressure / (float)samples;
    
    /* 计算地面绝对高度 */
    float pressure_ratio = g_spl06_data.ground_pressure / g_spl06_data.sea_level_pressure;
    g_spl06_data.ground_altitude = 44330.0f * (1.0f - powf(pressure_ratio, 0.1903f));
    
    /* 重置滤波器 */
    prev_altitude = 0.0f;
    prev_time = 0;
}

/**
 * @brief 设置海平面气压
 */
void SPL06_SetSeaLevelPressure(float pressure)
{
    if (pressure > 80000.0f && pressure < 120000.0f) {
        g_spl06_data.sea_level_pressure = pressure;
    }
}

/**
 * @brief 获取垂直速度
 */
float SPL06_GetVerticalSpeed(void)
{
    return g_spl06_data.vertical_speed;
}