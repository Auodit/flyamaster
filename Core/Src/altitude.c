/**
 * @file altitude.c
 * @brief 高度融合与定高控制模块实现
 * @details 使用互补滤波融合激光高度和加速度计
 * @author AI Assistant
 * @date 2026-01-14
 */

#include "altitude.h"
#include "main.h"
#include <math.h>

/*==============================================================================
 * 全局变量
 *============================================================================*/
Altitude_TypeDef Alt_Data;

/*==============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief 初始化高度融合模块
 */
void Altitude_Init(Altitude_TypeDef *alt) {
    alt->height = 0.0f;
    alt->velocity = 0.0f;
    alt->acc_z_earth = 0.0f;
    alt->height_laser = 0.0f;
    alt->height_baro = 0.0f;
    alt->laser_last_update = 0;
    alt->baro_last_update = 0;
    alt->cf_gain = ALT_CF_GAIN;
    alt->laser_valid = 0;
    alt->baro_valid = 0;
    alt->fusion_ready = 0;
}

/**
 * @brief 将机体坐标系加速度转换到地球坐标系 Z 轴
 * @details 使用四元数旋转矩阵进行坐标变换
 * 
 * 旋转矩阵 R 的第三行（地球 Z 轴在机体坐标系中的投影）:
 * R[2][0] = 2*(q1*q3 - q0*q2)
 * R[2][1] = 2*(q2*q3 + q0*q1)  
 * R[2][2] = 1 - 2*(q1*q1 + q2*q2)
 * 
 * 地球 Z 轴加速度 = R[2][0]*ax + R[2][1]*ay + R[2][2]*az
 */
float Altitude_TransformAccZ(Mahony_TypeDef *mahony, float ax, float ay, float az) {
    // Risk #047 修复：添加指针空检查
    if (mahony == NULL) return 0.0f;
    
    float q0 = mahony->q0;
    float q1 = mahony->q1;
    float q2 = mahony->q2;
    float q3 = mahony->q3;
    
    // 计算旋转矩阵第三行元素
    float r20 = 2.0f * (q1 * q3 - q0 * q2);
    float r21 = 2.0f * (q2 * q3 + q0 * q1);
    float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    
    // 地球坐标系 Z 轴加速度 (单位: g)
    float az_earth = r20 * ax + r21 * ay + r22 * az;
    
    // 去除重力 (1g)，转换为 m/s^2
    // 注意: 当飞机水平静止时, az_earth ≈ 1.0g, 去除后应为 0
    float az_compensated = (az_earth - 1.0f) * ALT_ACC_SCALE;
    
    return az_compensated;
}

/**
 * @brief 更新激光高度测量值
 */
void Altitude_UpdateLaser(Altitude_TypeDef *alt, float height_m) {
    // 范围限制
    if (height_m < ALT_MIN_HEIGHT) {
        height_m = ALT_MIN_HEIGHT;
    } else if (height_m > ALT_MAX_HEIGHT) {
        height_m = ALT_MAX_HEIGHT;
    }
    
    alt->height_laser = height_m;
    alt->laser_last_update = HAL_GetTick();
    alt->laser_valid = 1;
    
    // 首次收到有效激光数据，初始化高度估计
    if (!alt->fusion_ready) {
        alt->height = height_m;
        alt->velocity = 0.0f;
        alt->fusion_ready = 1;
    }
}

/**
 * @brief 更新气压计高度测量值
 * @note 气压计高度为相对高度（相对于开机时的海拔）
 */
void Altitude_UpdateBarometer(Altitude_TypeDef *alt, float height_m) {
    // 范围限制（气压计可以测更高）
    if (height_m < 0.0f) {
        height_m = 0.0f;
    } else if (height_m > 100.0f) {
        height_m = 100.0f;
    }
    
    alt->height_baro = height_m;
    alt->baro_last_update = HAL_GetTick();
    alt->baro_valid = 1;
    
    // 首次收到气压计数据（如果激光还未初始化）
    if (!alt->fusion_ready) {
        alt->height = height_m;
        alt->velocity = 0.0f;
        alt->fusion_ready = 1;
    }
}

/**
 * @brief 互补滤波融合更新（智能数据源切换版）
 * @details
 * 智能切换策略:
 * - 高度 < 1.8m: 100% 激光高度
 * - 高度 1.8m ~ 2.2m: 线性混合 (激光权重从 100% 降到 0%)
 * - 高度 > 2.2m: 100% 气压计高度
 *
 * 互补滤波公式:
 *   H_est = H_est + Vel_Z * dt
 *   Vel_Z = Vel_Z + Acc_Z * dt + K * (H_sensor - H_est)
 */
void Altitude_FusionUpdate(Altitude_TypeDef *alt, float acc_z_earth, float dt) {
    // 存储加速度
    alt->acc_z_earth = acc_z_earth;
    
    // 检查数据超时
    if ((HAL_GetTick() - alt->laser_last_update) > ALT_LASER_TIMEOUT) {
        alt->laser_valid = 0;
    }
    if ((HAL_GetTick() - alt->baro_last_update) > ALT_LASER_TIMEOUT) {
        alt->baro_valid = 0;
    }
    
    // 如果融合未就绪，等待第一个传感器测量
    if (!alt->fusion_ready) {
        return;
    }
    
    // ========== 智能数据源切换 ==========
    float height_sensor = 0.0f;  // 融合后的传感器高度
    float laser_trust = 0.0f;    // 激光信任度 (0.0 ~ 1.0)
    float baro_trust = 0.0f;     // 气压信任度 (0.0 ~ 1.0)
    uint8_t has_sensor = 0;      // 是否有有效传感器数据
    
    // 根据当前高度估计值决定数据源权重
    if (alt->height < ALT_LASER_ONLY_MAX) {
        // < 1.8m: 100% 激光
        laser_trust = 1.0f;
        baro_trust = 0.0f;
    } else if (alt->height < ALT_BARO_ONLY_MIN) {
        // 1.8m ~ 2.2m: 线性过渡
        float transition_ratio = (alt->height - ALT_LASER_ONLY_MAX) /
                                 (ALT_BARO_ONLY_MIN - ALT_LASER_ONLY_MAX);
        laser_trust = 1.0f - transition_ratio;  // 从 1.0 降到 0.0
        baro_trust = transition_ratio;          // 从 0.0 升到 1.0
    } else {
        // > 2.2m: 100% 气压
        laser_trust = 0.0f;
        baro_trust = 1.0f;
    }
    
    // 加权融合传感器数据
    if (alt->laser_valid && laser_trust > 0.0f) {
        height_sensor += alt->height_laser * laser_trust;
        has_sensor = 1;
    }
    if (alt->baro_valid && baro_trust > 0.0f) {
        height_sensor += alt->height_baro * baro_trust;
        has_sensor = 1;
    }
    
    // ========== 互补滤波更新 ==========
    if (has_sensor) {
        // 有传感器数据：完整互补滤波
        float height_error = height_sensor - alt->height;
        
        // 速度估计更新: 加速度积分 + 高度误差修正
        alt->velocity += acc_z_earth * dt + alt->cf_gain * height_error;
        
        // 高度估计更新: 速度积分
        alt->height += alt->velocity * dt;
    } else {
        // 无传感器数据：纯加速度积分 (会漂移，但短期可用)
        alt->velocity += acc_z_earth * dt;
        alt->height += alt->velocity * dt;
        
        // 防止积分发散
        if (alt->height < 0.0f) {
            alt->height = 0.0f;
            alt->velocity = 0.0f;
        }
        if (alt->height > 100.0f) {
            alt->height = 100.0f;
        }
    }
    
    // 速度限幅 (防止积分爆炸)
    if (alt->velocity > 5.0f) alt->velocity = 5.0f;
    if (alt->velocity < -5.0f) alt->velocity = -5.0f;
}

/**
 * @brief 获取当前估计高度
 */
float Altitude_GetHeight(Altitude_TypeDef *alt) {
    return alt->height;
}

/**
 * @brief 获取当前估计垂直速度
 */
float Altitude_GetVelocity(Altitude_TypeDef *alt) {
    return alt->velocity;
}

/**
 * @brief 检查高度融合是否就绪
 * @details 任一传感器有效即可使用定高功能
 */
uint8_t Altitude_IsReady(Altitude_TypeDef *alt) {
    return alt->fusion_ready && (alt->laser_valid || alt->baro_valid);
}
