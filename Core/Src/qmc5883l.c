/**
 ******************************************************************************
 * @file    qmc5883l.c
 * @brief   QMC5883L 三轴磁力计驱动实现
 * @note    I2C1 地址: 0x0D, 与 MPU6050 (0x68) 并联
 ******************************************************************************
 */

#include "main.h"
#include "qmc5883l.h"
#include <math.h>
#include <stdio.h>

// ==================== 全局变量 ====================
// 添加 volatile 修饰符（Risk #033 修复）
     volatile QMC5883L_t QMC_Data = {0};
volatile QMC5883L_Calibration_t QMC_Calibration = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f,
    .calibrated = 0
};

// 校准相关静态变量
static uint8_t calibration_active = 0;
static uint32_t calibration_start_time = 0;
static float mag_min_x = 10000.0f, mag_max_x = -10000.0f;
static float mag_min_y = 10000.0f, mag_max_y = -10000.0f;
static float mag_min_z = 10000.0f, mag_max_z = -10000.0f;

#define I2C_TIMEOUT  20  // I2C 超时 20ms（Risk #029 优化：正常读取仅需0.5-1ms，20ms足够安全）

// ==================== 内部函数 ====================

/**
 * @brief 写单个寄存器
 */
static uint8_t QMC5883L_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    // Risk #081 修复：指针空检查
    if (hi2c == NULL) {
        return 1;
    }
    
    if (HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR, reg, 1, &data, 1, I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 读单个寄存器
 */
static uint8_t QMC5883L_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data) {
    // Risk #081 修复：指针空检查
    if (hi2c == NULL || data == NULL) {
        return 1;
    }
    
    if (HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, reg, 1, data, 1, I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 连续读取多个寄存器
 */
static uint8_t QMC5883L_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len) {
    // Risk #081 修复：指针空检查
    if (hi2c == NULL || data == NULL) {
        return 1;
    }
    
    if (HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, reg, 1, data, len, I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }
    return 0;
}

// ==================== 公共函数实现 ====================

/**
 * @brief 初始化 QMC5883L
 */
uint8_t QMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id = 0;
    
    // Risk #079 修复：指针空检查
    if (hi2c == NULL) {
        printf("[QMC5883L] Init failed: NULL I2C handle\r\n");
        return 1;
    }
    
    // 1. 检查芯片 ID (应该是 0xFF)
    if (QMC5883L_ReadReg(hi2c, QMC5883L_REG_CHIP_ID, &chip_id) != 0) {
        printf("[QMC5883L] I2C Read Failed!\r\n");
        return 1;
    }
    
    if (chip_id != 0xFF) {
        printf("[QMC5883L] Invalid Chip ID: 0x%02X (expected 0xFF)\r\n", chip_id);
        return 1;
    }
    
    // 2. 软复位
    if (QMC5883L_WriteReg(hi2c, QMC5883L_REG_CONTROL2, 0x80) != 0) {
        printf("[QMC5883L] Soft Reset Failed!\r\n");
        return 1;
    }
    
    HAL_Delay(10);  // 等待复位完成
    
    // 3. 配置 SET/RESET 周期寄存器 (推荐值 0x01)
    if (QMC5883L_WriteReg(hi2c, QMC5883L_REG_SET_RESET, 0x01) != 0) {
        return 1;
    }
    
    // 4. 配置控制寄存器 1
    // MODE: 连续测量 (0x01)
    // ODR: 50Hz (0x04)
    // RNG: ±2G (0x00)
    // OSR: 512 (0x00) - 低噪声
    uint8_t ctrl1 = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_50HZ | 
                    QMC5883L_RNG_2G | QMC5883L_OSR_512;
    
    if (QMC5883L_WriteReg(hi2c, QMC5883L_REG_CONTROL1, ctrl1) != 0) {
        printf("[QMC5883L] Config Failed!\r\n");
        return 1;
    }
    
    // 5. 禁用中断
    if (QMC5883L_WriteReg(hi2c, QMC5883L_REG_CONTROL2, 0x00) != 0) {
        return 1;
    }
    
    HAL_Delay(10);  // 等待配置生效
    
    // 6. 读取一次数据，激活传感器
    QMC5883L_ReadRaw(hi2c);
    
    QMC_Data.initialized = 1;
    printf("[QMC5883L] Init Success! (50Hz, ±2G, OSR=512)\r\n");
    
    return 0;
}

/**
 * @brief 读取原始磁场数据
 */
uint8_t QMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c) {
    uint8_t data[6];
    uint8_t status;
    
    // Risk #080 修复：指针空检查
    if (hi2c == NULL) {
        return 1;
    }
    
    // 1. 检查数据就绪标志
    if (QMC5883L_ReadReg(hi2c, QMC5883L_REG_STATUS, &status) != 0) {
        return 1;
    }
    
    // 数据未就绪，直接返回
    if (!(status & QMC5883L_STATUS_DRDY)) {
        QMC_Data.data_ready = 0;
        return 1;
    }
    
    // 检测数据溢出
    if (status & QMC5883L_STATUS_OVL) {
        printf("[QMC5883L] Warning: Data overflow!\r\n");
        // 继续读取，但标记可能不准确
    }
    
    // 2. 连续读取 6 字节数据 (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    if (QMC5883L_ReadRegs(hi2c, QMC5883L_REG_X_LSB, data, 6) != 0) {
        return 1;
    }
    
    // 3. 组合成 16 位有符号整数（小端序）
    QMC_Data.mag_x_raw = (int16_t)((data[1] << 8) | data[0]);
    QMC_Data.mag_y_raw = (int16_t)((data[3] << 8) | data[2]);
    QMC_Data.mag_z_raw = (int16_t)((data[5] << 8) | data[4]);
    
    // 4. 转换为 Gauss (±2G 量程，灵敏度 12000 LSB/Gauss)
    QMC_Data.mag_x = QMC_Data.mag_x_raw / 12000.0f;
    QMC_Data.mag_y = QMC_Data.mag_y_raw / 12000.0f;
    QMC_Data.mag_z = QMC_Data.mag_z_raw / 12000.0f;
    
    // 5. 应用校准参数（如果已校准）
    if (QMC_Calibration.calibrated) {
        QMC_Data.mag_x = (QMC_Data.mag_x - QMC_Calibration.offset_x) * QMC_Calibration.scale_x;
        QMC_Data.mag_y = (QMC_Data.mag_y - QMC_Calibration.offset_y) * QMC_Calibration.scale_y;
        QMC_Data.mag_z = (QMC_Data.mag_z - QMC_Calibration.offset_z) * QMC_Calibration.scale_z;
    }
    
    // 6. 更新状态
    QMC_Data.data_ready = 1;
    QMC_Data.last_update = HAL_GetTick();
    
    return 0;
}

/**
 * @brief 计算航向角（带倾斜补偿）
 */
float QMC5883L_CalculateHeading(Mahony_TypeDef *mahony) {
    if (mahony == NULL || !QMC_Data.data_ready) {
        return QMC_Data.heading;  // 返回上次计算的航向
    }
    
    // 1. 从 Mahony 四元数提取 Roll/Pitch（弧度）
    float q0 = mahony->q0, q1 = mahony->q1, q2 = mahony->q2, q3 = mahony->q3;
    
    float roll_rad = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float pitch_rad = asinf(2.0f * (q0 * q2 - q3 * q1));
    
    // 2. 倾斜补偿：将磁场投影到水平面
    // 参考公式：
    // Xh = X * cos(pitch) + Z * sin(pitch)
    // Yh = X * sin(roll) * sin(pitch) + Y * cos(roll) - Z * sin(roll) * cos(pitch)
    
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    
    QMC_Data.mag_x_h = QMC_Data.mag_x * cos_pitch + QMC_Data.mag_z * sin_pitch;
    QMC_Data.mag_y_h = QMC_Data.mag_x * sin_roll * sin_pitch + 
                       QMC_Data.mag_y * cos_roll - 
                       QMC_Data.mag_z * sin_roll * cos_pitch;
    
    // 3. 计算航向角（0~360°）
    float heading_rad = atan2f(QMC_Data.mag_y_h, QMC_Data.mag_x_h);
    
    // 转换为度数
    float heading_deg = heading_rad * 57.29578f;  // rad -> deg
    
    // 应用磁偏角修正
    heading_deg += MAG_DECLINATION;
    
    // 归一化到 0~360°
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    } else if (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }
    
    QMC_Data.heading = heading_deg;
    
    return heading_deg;
}

/**
 * @brief 启动 8 字校准
 */
void QMC5883L_Calibrate(void) {
    calibration_active = 1;
    calibration_start_time = HAL_GetTick();
    
    // 重置极值
    mag_min_x = mag_min_y = mag_min_z = 10000.0f;
    mag_max_x = mag_max_y = mag_max_z = -10000.0f;
    
    printf("[QMC5883L] Calibration Started! Rotate in figure-8 for 20 seconds...\r\n");
}

/**
 * @brief 更新校准数据（需在主循环中周期调用）
 */
uint8_t QMC5883L_CalibrateUpdate(void) {
    if (!calibration_active || !QMC_Data.data_ready) {
        return 0;
    }
    
    // 更新极值
    if (QMC_Data.mag_x < mag_min_x) mag_min_x = QMC_Data.mag_x;
    if (QMC_Data.mag_x > mag_max_x) mag_max_x = QMC_Data.mag_x;
    
    if (QMC_Data.mag_y < mag_min_y) mag_min_y = QMC_Data.mag_y;
    if (QMC_Data.mag_y > mag_max_y) mag_max_y = QMC_Data.mag_y;
    
    if (QMC_Data.mag_z < mag_min_z) mag_min_z = QMC_Data.mag_z;
    if (QMC_Data.mag_z > mag_max_z) mag_max_z = QMC_Data.mag_z;
    
    // 检查是否超时（20 秒）
    if (HAL_GetTick() - calibration_start_time >= 20000) {
        calibration_active = 0;
        
        // 计算硬磁偏移（极值中点）
        QMC_Calibration.offset_x = (mag_max_x + mag_min_x) / 2.0f;
        QMC_Calibration.offset_y = (mag_max_y + mag_min_y) / 2.0f;
        QMC_Calibration.offset_z = (mag_max_z + mag_min_z) / 2.0f;
        
        // 计算软磁缩放系数（椭球拟合简化版）
        float avg_delta = ((mag_max_x - mag_min_x) +
                          (mag_max_y - mag_min_y) +
                          (mag_max_z - mag_min_z)) / 3.0f;
        
        // Risk #030 修复：防止除零错误
        float delta_x = mag_max_x - mag_min_x;
        float delta_y = mag_max_y - mag_min_y;
        float delta_z = mag_max_z - mag_min_z;
        
        if (delta_x < 0.001f) delta_x = 0.001f;  // 防止除零
        if (delta_y < 0.001f) delta_y = 0.001f;
        if (delta_z < 0.001f) delta_z = 0.001f;
        
        QMC_Calibration.scale_x = avg_delta / delta_x;
        QMC_Calibration.scale_y = avg_delta / delta_y;
        QMC_Calibration.scale_z = avg_delta / delta_z;
        
        QMC_Calibration.calibrated = 1;
        
        printf("[QMC5883L] Calibration Complete!\r\n");
        printf("  Offset: X=%.3f, Y=%.3f, Z=%.3f\r\n", 
               QMC_Calibration.offset_x, 
               QMC_Calibration.offset_y, 
               QMC_Calibration.offset_z);
        printf("  Scale: X=%.3f, Y=%.3f, Z=%.3f\r\n",
               QMC_Calibration.scale_x,
               QMC_Calibration.scale_y,
               QMC_Calibration.scale_z);
        
        // 标记参数已修改，触发 Flash 保存
        FlashParams_MarkDirty();
        
        return 1;  // 校准完成
    }
    
    return 0;  // 校准中
}

/**
 * @brief 应用校准参数
 */
void QMC5883L_ApplyCalibration(void) {
    if (!QMC_Calibration.calibrated) {
        printf("[QMC5883L] Warning: No calibration data!\r\n");
        return;
    }
    
    // 重新应用校准（已在 ReadRaw 中自动应用）
    printf("[QMC5883L] Calibration applied.\r\n");
}

/**
 * @brief 磁干扰检测
 */
uint8_t QMC5883L_CheckInterference(void) {
    if (!QMC_Data.data_ready) {
        return 0;
    }
    
    // 计算磁场强度
    QMC_Data.mag_strength = sqrtf(QMC_Data.mag_x * QMC_Data.mag_x + 
                                   QMC_Data.mag_y * QMC_Data.mag_y + 
                                   QMC_Data.mag_z * QMC_Data.mag_z);
    
    // 地球磁场强度约 0.25~0.65 Gauss
    // 如果超出这个范围，判定为干扰
    if (QMC_Data.mag_strength < 0.15f || QMC_Data.mag_strength > 0.75f) {
        QMC_Data.interference_detected = 1;
        return 1;
    }
    
    QMC_Data.interference_detected = 0;
    return 0;
}

/**
 * @brief 互补滤波融合（陀螺仪 Yaw + 磁力计航向）
 */
float QMC5883L_ComplementaryFilter(float gyro_yaw_deg, float mag_heading, float dt, float alpha) {
    // Risk #035 修复：归一化输入到 0~360° 范围
    while (gyro_yaw_deg < 0.0f) gyro_yaw_deg += 360.0f;
    while (gyro_yaw_deg >= 360.0f) gyro_yaw_deg -= 360.0f;
    while (mag_heading < 0.0f) mag_heading += 360.0f;
    while (mag_heading >= 360.0f) mag_heading -= 360.0f;
    
    // 处理角度跨越 0°/360° 边界问题
    float diff = mag_heading - gyro_yaw_deg;
    
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    // 互补滤波公式：
    // yaw_fused = alpha * (gyro_yaw + gyro_rate * dt) + (1 - alpha) * mag_heading
    // 简化版（假设 gyro_yaw 已经是积分后的角度）：
    // yaw_fused = alpha * gyro_yaw + (1 - alpha) * mag_heading
    
    float yaw_fused = gyro_yaw_deg + (1.0f - alpha) * diff;
    
    // 归一化到 0~360°
    if (yaw_fused < 0.0f) {
        yaw_fused += 360.0f;
    } else if (yaw_fused >= 360.0f) {
        yaw_fused -= 360.0f;
    }
    
    QMC_Data.heading_filtered = yaw_fused;
    
    return yaw_fused;
}
