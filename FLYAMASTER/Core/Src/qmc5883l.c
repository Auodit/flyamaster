/**
 * @file qmc5883l.c
 * @brief QMC5883L 磁力计驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "qmc5883l.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define QMC5883L_TIMEOUT        100     /**< I2C 超时 (ms) */
#define QMC5883L_SENSITIVITY_2G 12000.0f /**< 2G 量程灵敏度 (LSB/Gauss) */
#define QMC5883L_SENSITIVITY_8G 3000.0f  /**< 8G 量程灵敏度 (LSB/Gauss) */

#define DEG_TO_RAD              0.017453292519943f
#define RAD_TO_DEG              57.29577951308232f

/* 低通滤波系数 */
#define HEADING_LPF_ALPHA       0.1f

/* Private variables ---------------------------------------------------------*/
QMC5883L_Data_t g_qmc5883l_data = {0};

/* DMA 接收缓冲区 */
static uint8_t qmc_dma_buffer[6];

/* 校准过程中的最大最小值 */
static int16_t cal_min_x, cal_max_x;
static int16_t cal_min_y, cal_max_y;
static int16_t cal_min_z, cal_max_z;
static bool calibrating = false;

/* 当前灵敏度 */
static float current_sensitivity = QMC5883L_SENSITIVITY_2G;

/* Private function prototypes -----------------------------------------------*/
static void QMC5883L_ApplyCalibration(void);
static float QMC5883L_WrapAngle(float angle);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 写寄存器
 */
bool QMC5883L_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, reg, 
                               I2C_MEMADD_SIZE_8BIT, &data, 1, QMC5883L_TIMEOUT);
    return (status == HAL_OK);
}

/**
 * @brief 读寄存器
 */
bool QMC5883L_ReadReg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, data, 1, QMC5883L_TIMEOUT);
    return (status == HAL_OK);
}

/**
 * @brief 读取多个寄存器
 */
bool QMC5883L_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, data, len, QMC5883L_TIMEOUT);
    return (status == HAL_OK);
}

/**
 * @brief 软复位
 */
void QMC5883L_SoftReset(void)
{
    QMC5883L_WriteReg(QMC5883L_REG_CTRL2, 0x80);
    HAL_Delay(10);
}

/**
 * @brief 初始化 QMC5883L
 */
bool QMC5883L_Init(void)
{
    uint8_t chip_id;
    
    /* 清空数据结构 */
    memset(&g_qmc5883l_data, 0, sizeof(g_qmc5883l_data));
    
    /* 默认校准参数 */
    g_qmc5883l_data.calibration.scale_x = 1.0f;
    g_qmc5883l_data.calibration.scale_y = 1.0f;
    g_qmc5883l_data.calibration.scale_z = 1.0f;
    
    /* 软复位 */
    QMC5883L_SoftReset();
    
    /* 读取芯片 ID */
    if (!QMC5883L_ReadReg(QMC5883L_REG_CHIP_ID, &chip_id)) {
        return false;
    }
    
    /* 验证芯片 ID (QMC5883L 返回 0xFF) */
    if (chip_id != QMC5883L_CHIP_ID) {
        /* 可能是 HMC5883L 或其他兼容芯片，继续尝试 */
    }
    
    /* 设置 SET/RESET 周期 */
    if (!QMC5883L_WriteReg(QMC5883L_REG_SET_RESET, 0x01)) {
        return false;
    }
    
    /* 配置控制寄存器 1:
     * - 连续测量模式
     * - 100Hz 输出速率
     * - ±2G 量程
     * - 512 过采样率 (最高精度)
     */
    uint8_t ctrl1 = QMC5883L_MODE_CONT | QMC5883L_ODR_100HZ | 
                    QMC5883L_RNG_2G | QMC5883L_OSR_512;
    if (!QMC5883L_WriteReg(QMC5883L_REG_CTRL1, ctrl1)) {
        return false;
    }
    
    current_sensitivity = QMC5883L_SENSITIVITY_2G;
    
    /* 配置控制寄存器 2:
     * - 禁用中断
     * - 指针自动回滚
     */
    if (!QMC5883L_WriteReg(QMC5883L_REG_CTRL2, 0x00)) {
        return false;
    }
    
    /* 等待首次数据就绪 */
    HAL_Delay(10);
    
    g_qmc5883l_data.initialized = true;
    
    return true;
}

/**
 * @brief 检查数据是否就绪
 */
bool QMC5883L_IsDataReady(void)
{
    uint8_t status;
    if (!QMC5883L_ReadReg(QMC5883L_REG_STATUS, &status)) {
        return false;
    }
    return (status & QMC5883L_STATUS_DRDY) != 0;
}

/**
 * @brief 读取磁力计数据 (阻塞方式)
 */
bool QMC5883L_Read(void)
{
    uint8_t buffer[6];
    
    if (!g_qmc5883l_data.initialized) {
        return false;
    }
    
    /* 读取 6 字节数据 (X_L, X_H, Y_L, Y_H, Z_L, Z_H) */
    if (!QMC5883L_ReadRegs(QMC5883L_REG_DATA_X_L, buffer, 6)) {
        return false;
    }
    
    /* 解析原始数据 (小端序) */
    g_qmc5883l_data.raw.x = (int16_t)((buffer[1] << 8) | buffer[0]);
    g_qmc5883l_data.raw.y = (int16_t)((buffer[3] << 8) | buffer[2]);
    g_qmc5883l_data.raw.z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    /* 应用校准 */
    QMC5883L_ApplyCalibration();
    
    /* 更新校准数据 (如果正在校准) */
    if (calibrating) {
        QMC5883L_UpdateCalibration();
    }
    
    g_qmc5883l_data.data_ready = true;
    g_qmc5883l_data.update_count++;
    
    return true;
}

/**
 * @brief 使用 DMA 读取数据
 */
bool QMC5883L_ReadDMA(void)
{
    HAL_StatusTypeDef status;
    
    if (!g_qmc5883l_data.initialized) {
        return false;
    }
    
    /* 启动 DMA 读取 */
    status = HAL_I2C_Mem_Read_DMA(&hi2c2, QMC5883L_ADDR, 
                                  QMC5883L_REG_DATA_X_L,
                                  I2C_MEMADD_SIZE_8BIT,
                                  qmc_dma_buffer, 6);
    
    return (status == HAL_OK);
}

/**
 * @brief 处理 DMA 接收完成的数据
 * @note 在 HAL_I2C_MemRxCpltCallback 中调用
 */
void QMC5883L_ProcessDMAData(void)
{
    /* 解析原始数据 */
    g_qmc5883l_data.raw.x = (int16_t)((qmc_dma_buffer[1] << 8) | qmc_dma_buffer[0]);
    g_qmc5883l_data.raw.y = (int16_t)((qmc_dma_buffer[3] << 8) | qmc_dma_buffer[2]);
    g_qmc5883l_data.raw.z = (int16_t)((qmc_dma_buffer[5] << 8) | qmc_dma_buffer[4]);
    
    /* 应用校准 */
    QMC5883L_ApplyCalibration();
    
    /* 更新校准数据 */
    if (calibrating) {
        QMC5883L_UpdateCalibration();
    }
    
    g_qmc5883l_data.data_ready = true;
    g_qmc5883l_data.update_count++;
}

/**
 * @brief 应用校准参数
 */
static void QMC5883L_ApplyCalibration(void)
{
    QMC5883L_Calibration_t *cal = &g_qmc5883l_data.calibration;
    
    /* 应用偏移和缩放 */
    float cal_x = (float)(g_qmc5883l_data.raw.x - cal->offset_x) * cal->scale_x;
    float cal_y = (float)(g_qmc5883l_data.raw.y - cal->offset_y) * cal->scale_y;
    float cal_z = (float)(g_qmc5883l_data.raw.z - cal->offset_z) * cal->scale_z;
    
    /* 转换为高斯 */
    g_qmc5883l_data.mag_x = cal_x / current_sensitivity;
    g_qmc5883l_data.mag_y = cal_y / current_sensitivity;
    g_qmc5883l_data.mag_z = cal_z / current_sensitivity;
}

/* 航向角滤波器状态 (模块级静态变量，支持重置) */
static float s_heading_prev = 0;
static bool s_heading_first_run = true;

/**
 * @brief 计算航向角 (无倾斜补偿)
 */
float QMC5883L_CalculateHeading(void)
{
    float heading;
    
    /* 计算航向角 (atan2 返回 -π 到 π) */
    heading = atan2f(g_qmc5883l_data.mag_y, g_qmc5883l_data.mag_x) * RAD_TO_DEG;
    
    /* 转换为 0-360 度 */
    heading = QMC5883L_WrapAngle(heading);
    
    g_qmc5883l_data.heading = heading;
    
    /* 低通滤波 */
    if (s_heading_first_run) {
        s_heading_prev = heading;
        s_heading_first_run = false;
    }
    
    /* 处理 0/360 度跨越 */
    float diff = heading - s_heading_prev;
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    s_heading_prev += diff * HEADING_LPF_ALPHA;
    s_heading_prev = QMC5883L_WrapAngle(s_heading_prev);
    
    g_qmc5883l_data.heading_filtered = s_heading_prev;
    
    return heading;
}

/**
 * @brief 重置航向角滤波器
 * @note 在重新初始化磁力计或需要清除滤波历史时调用
 */
void QMC5883L_ResetHeadingFilter(void)
{
    s_heading_prev = 0;
    s_heading_first_run = true;
}

/**
 * @brief 计算带倾斜补偿的航向角
 * @param roll 横滚角 (度)
 * @param pitch 俯仰角 (度)
 */
float QMC5883L_CalculateTiltCompensatedHeading(float roll, float pitch)
{
    float roll_rad = roll * DEG_TO_RAD;
    float pitch_rad = pitch * DEG_TO_RAD;
    
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    
    /* 倾斜补偿公式 */
    float mag_x = g_qmc5883l_data.mag_x;
    float mag_y = g_qmc5883l_data.mag_y;
    float mag_z = g_qmc5883l_data.mag_z;
    
    float x_h = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch;
    float y_h = mag_y * cos_roll - mag_z * sin_roll;
    
    /* 计算航向角 */
    float heading = atan2f(y_h, x_h) * RAD_TO_DEG;
    
    /* 转换为 0-360 度 */
    heading = QMC5883L_WrapAngle(heading);
    
    g_qmc5883l_data.heading = heading;
    
    return heading;
}

/**
 * @brief 角度归一化到 0-360
 */
static float QMC5883L_WrapAngle(float angle)
{
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

/**
 * @brief 开始校准
 */
void QMC5883L_StartCalibration(void)
{
    /* 初始化最大最小值 */
    cal_min_x = cal_min_y = cal_min_z = 32767;
    cal_max_x = cal_max_y = cal_max_z = -32768;
    
    calibrating = true;
    g_qmc5883l_data.calibration.calibrated = false;
}

/**
 * @brief 更新校准数据
 * @note 在校准过程中持续调用，需要旋转传感器覆盖所有方向
 */
void QMC5883L_UpdateCalibration(void)
{
    if (!calibrating) return;
    
    int16_t x = g_qmc5883l_data.raw.x;
    int16_t y = g_qmc5883l_data.raw.y;
    int16_t z = g_qmc5883l_data.raw.z;
    
    /* 更新最大最小值 */
    if (x < cal_min_x) cal_min_x = x;
    if (x > cal_max_x) cal_max_x = x;
    if (y < cal_min_y) cal_min_y = y;
    if (y > cal_max_y) cal_max_y = y;
    if (z < cal_min_z) cal_min_z = z;
    if (z > cal_max_z) cal_max_z = z;
}

/**
 * @brief 完成校准
 */
void QMC5883L_FinishCalibration(void)
{
    if (!calibrating) return;
    
    calibrating = false;
    
    /* 计算偏移 (硬铁校准) */
    g_qmc5883l_data.calibration.offset_x = (cal_max_x + cal_min_x) / 2;
    g_qmc5883l_data.calibration.offset_y = (cal_max_y + cal_min_y) / 2;
    g_qmc5883l_data.calibration.offset_z = (cal_max_z + cal_min_z) / 2;
    
    /* 计算缩放 (软铁校准) */
    float range_x = (float)(cal_max_x - cal_min_x);
    float range_y = (float)(cal_max_y - cal_min_y);
    float range_z = (float)(cal_max_z - cal_min_z);
    
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    
    if (range_x > 0) {
        g_qmc5883l_data.calibration.scale_x = avg_range / range_x;
    }
    if (range_y > 0) {
        g_qmc5883l_data.calibration.scale_y = avg_range / range_y;
    }
    if (range_z > 0) {
        g_qmc5883l_data.calibration.scale_z = avg_range / range_z;
    }
    
    g_qmc5883l_data.calibration.calibrated = true;
}

/**
 * @brief 设置校准参数
 */
void QMC5883L_SetCalibration(const QMC5883L_Calibration_t *cal)
{
    if (cal != NULL) {
        memcpy(&g_qmc5883l_data.calibration, cal, sizeof(QMC5883L_Calibration_t));
    }
}

/**
 * @brief 获取校准参数
 */
void QMC5883L_GetCalibration(QMC5883L_Calibration_t *cal)
{
    if (cal != NULL) {
        memcpy(cal, &g_qmc5883l_data.calibration, sizeof(QMC5883L_Calibration_t));
    }
}