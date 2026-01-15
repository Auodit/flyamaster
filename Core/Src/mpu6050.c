#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

// ============= 全局校准数据 =============
// Risk #078 修复：添加 volatile 修饰符（RTOS多任务共享变量）
volatile MPU6050_Calibration_t MPU6050_Calibration = {
    .gyro_offset_x = 0.0f,
    .gyro_offset_y = 0.0f,
    .gyro_offset_z = 0.0f,
    .accel_offset_x = 0.0f,
    .accel_offset_y = 0.0f,
    .accel_offset_z = 0.0f,
    .calibrated = 0
};

// ============= 全局 MPU6050 数据 =============
// Risk #078 修复：添加 volatile 修饰符（RTOS多任务共享变量）
volatile MPU6050_t MPU6050_Data = {0};  // 传感器数据全局变量

/**
 * @brief 初始化 MPU6050
 * @return 0: 成功, 1: 失败
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // Risk #067 修复：指针空检查
    if (hi2c == NULL) {
        return 1;
    }

    // 1. 检查设备 ID (WHO_AM_I) - Risk #068 修复：检查HAL返回值
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 1000) != HAL_OK) {
        return 1;
    }

    if (check == 0x68) { // 默认 ID 为 0x68
        // 2. 唤醒传感器 (PWR_MGMT_1) - Risk #068 修复：检查HAL返回值
        data = 0x00; // 解除休眠，使用内部 8MHz 振荡器
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 1000) != HAL_OK) {
            return 1;
        }

        // 3. 设置采样率分频 (SMPLRT_DIV) - Risk #068 修复：检查HAL返回值
        // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        // Gyro Output Rate = 1kHz (当 DLPF 启用时)
        // ⚠️ 重要：采样率必须 >= imu_task 频率 (500Hz)
        // 否则会导致 PID D 项失效和姿态阶梯跳变！
        data = 0x01; // Sample Rate = 1000 / (1 + 1) = 500Hz (匹配 imu_task 2ms 周期)
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, 1000) != HAL_OK) {
            return 1;
        }

        // 4. 配置加速度计 (ACCEL_CONFIG) - Risk #068 修复：检查HAL返回值
        data = 0x00; // 量程 +/- 2g
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, 1000) != HAL_OK) {
            return 1;
        }

        // 5. 配置陀螺仪 (GYRO_CONFIG) - Risk #068 修复：检查HAL返回值
        data = 0x18; // 量程 +/- 2000 deg/s (0x18 = 00011000)
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 1000) != HAL_OK) {
            return 1;
        }
        
        // 6. 配置低通滤波器 (CONFIG) - Risk #068 修复：检查HAL返回值
        // DLPF_CFG = 3 (Bandwidth = 44Hz, Delay = 4.9ms)
        data = 0x03;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_CONFIG, 1, &data, 1, 1000) != HAL_OK) {
            return 1;
        }

        return 0; // 初始化成功
    }

    return 1; // 初始化失败
}

/**
 * @brief 读取加速度计数据
 */
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Risk #069 修复：指针空检查
    if (hi2c == NULL || DataStruct == NULL) {
        return;
    }

    // 读取 6 个字节 (X_H, X_L, Y_H, Y_L, Z_H, Z_L) - Risk #070 修复：检查HAL返回值
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6, 100) != HAL_OK) {
        return;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 转换为物理量 (m/s^2)
    // 量程 +/- 2g, 灵敏度 16384 LSB/g
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;
}

/**
 * @brief 读取陀螺仪数据
 */
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Risk #071 修复：指针空检查
    if (hi2c == NULL || DataStruct == NULL) {
        return;
    }

    // 读取 6 个字节 - Risk #072 修复：检查HAL返回值
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, Rec_Data, 6, 100) != HAL_OK) {
        return;
    }

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 转换为物理量 (deg/s -> rad/s)
    // 量程 +/- 2000 dps, 灵敏度 16.4 LSB/dps
    // 1 deg = 0.0174533 rad
    DataStruct->Gx = (DataStruct->Gyro_X_RAW / 16.4f) * 0.0174533f;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW / 16.4f) * 0.0174533f;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW / 16.4f) * 0.0174533f;
}

/**
 * @brief 读取所有数据 (推荐使用，效率更高)
 * @note 此函数不应用校准偏移，用于校准过程本身
 */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];

    // Risk #073 修复：指针空检查
    if (hi2c == NULL || DataStruct == NULL) {
        return;
    }

    // 一次性读取 14 个字节 (Accel + Temp + Gyro) - Risk #074 修复：检查HAL返回值
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 14, 100) != HAL_OK) {
        return;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    // Rec_Data[6] 和 [7] 是温度，这里暂时忽略

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;

    DataStruct->Gx = (DataStruct->Gyro_X_RAW / 16.4f) * 0.0174533f;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW / 16.4f) * 0.0174533f;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW / 16.4f) * 0.0174533f;
}

/**
 * @brief 陀螺仪零偏校准
 * @note 校准时飞机必须静止放置在水平面上！
 *       校准过程约需 2 秒 (2000 次采样 × 1ms)
 */
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c) {
    MPU6050_t temp_data;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    
    // Risk #075 修复：指针空检查
    if (hi2c == NULL) {
        printf("[MPU6050] Calibration failed: NULL I2C handle\r\n");
        return;
    }
    
    printf(">>> Gyro Calibration Started! Keep the drone STILL! <<<\r\n");
    
    // 等待传感器稳定
    HAL_Delay(500);
    
    // 采样 2000 次（使用 50ms 超时避免长时间卡死）
    for (int i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++) {
        // 注意：校准循环中使用较短超时，但保持非阻塞
        uint8_t Rec_Data[14];
        // Risk #076 修复：检查HAL返回值
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 14, 50) != HAL_OK) {
            printf("[MPU6050] I2C read failed during calibration at sample %d\r\n", i);
            continue;  // 跳过本次采样，继续校准
        }
        
        // 手动解析（避免递归调用 MPU6050_Read_All）
        temp_data.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        temp_data.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        temp_data.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        temp_data.Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        temp_data.Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        temp_data.Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
        
        temp_data.Ax = temp_data.Accel_X_RAW / 16384.0f;
        temp_data.Ay = temp_data.Accel_Y_RAW / 16384.0f;
        temp_data.Az = temp_data.Accel_Z_RAW / 16384.0f;
        temp_data.Gx = (temp_data.Gyro_X_RAW / 16.4f) * 0.0174533f;
        temp_data.Gy = (temp_data.Gyro_Y_RAW / 16.4f) * 0.0174533f;
        temp_data.Gz = (temp_data.Gyro_Z_RAW / 16.4f) * 0.0174533f;
        
        sum_gx += temp_data.Gx;
        sum_gy += temp_data.Gy;
        sum_gz += temp_data.Gz;
        
        sum_ax += temp_data.Ax;
        sum_ay += temp_data.Ay;
        sum_az += temp_data.Az;
        
        // 每 1ms 采样一次 (与 MPU6050 500Hz 采样率匹配)
        HAL_Delay(1);
        
        // 每 500 次打印进度
        if ((i + 1) % 500 == 0) {
            printf("Calibrating... %d/%d\r\n", i + 1, MPU6050_CALIBRATION_SAMPLES);
        }
    }
    
    // 计算平均值作为零偏
    MPU6050_Calibration.gyro_offset_x = sum_gx / MPU6050_CALIBRATION_SAMPLES;
    MPU6050_Calibration.gyro_offset_y = sum_gy / MPU6050_CALIBRATION_SAMPLES;
    MPU6050_Calibration.gyro_offset_z = sum_gz / MPU6050_CALIBRATION_SAMPLES;
    
    // 加速度计零偏 (假设 Z 轴朝上时应该是 1g)
    // X 和 Y 轴应该是 0，Z 轴应该是 1
    MPU6050_Calibration.accel_offset_x = sum_ax / MPU6050_CALIBRATION_SAMPLES;
    MPU6050_Calibration.accel_offset_y = sum_ay / MPU6050_CALIBRATION_SAMPLES;
    MPU6050_Calibration.accel_offset_z = (sum_az / MPU6050_CALIBRATION_SAMPLES) - 1.0f;  // 减去 1g
    
    MPU6050_Calibration.calibrated = 1;
    
    printf(">>> Calibration Complete! <<<\r\n");
    printf("Gyro Offset: X=%.4f, Y=%.4f, Z=%.4f rad/s\r\n",
           MPU6050_Calibration.gyro_offset_x,
           MPU6050_Calibration.gyro_offset_y,
           MPU6050_Calibration.gyro_offset_z);
    printf("Accel Offset: X=%.4f, Y=%.4f, Z=%.4f g\r\n",
           MPU6050_Calibration.accel_offset_x,
           MPU6050_Calibration.accel_offset_y,
           MPU6050_Calibration.accel_offset_z);
    
    // ========== 校准完成后标记参数已修改 ==========
    // 触发保存到 Flash（延迟 2 秒后自动保存）
    extern void FlashParams_MarkDirty(void);
    FlashParams_MarkDirty();
    printf("Gyro calibration data marked for Flash save.\r\n");
}

/**
 * @brief 读取带校准补偿的传感器数据
 * @note 必须先调用 MPU6050_CalibrateGyro() 完成校准
 */
void MPU6050_Read_All_Calibrated(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    // Risk #077 修复：指针空检查
    if (hi2c == NULL || DataStruct == NULL) {
        return;
    }
    
    // 先读取原始数据
    MPU6050_Read_All(hi2c, DataStruct);
    
    // 如果已校准，减去零偏
    if (MPU6050_Calibration.calibrated) {
        DataStruct->Gx -= MPU6050_Calibration.gyro_offset_x;
        DataStruct->Gy -= MPU6050_Calibration.gyro_offset_y;
        DataStruct->Gz -= MPU6050_Calibration.gyro_offset_z;
        
        DataStruct->Ax -= MPU6050_Calibration.accel_offset_x;
        DataStruct->Ay -= MPU6050_Calibration.accel_offset_y;
        DataStruct->Az -= MPU6050_Calibration.accel_offset_z;
    }
}

/**
 * @brief 检查陀螺仪是否已校准
 * @return 1: 已校准, 0: 未校准
 */
uint8_t MPU6050_IsCalibrated(void) {
    return MPU6050_Calibration.calibrated;
}
