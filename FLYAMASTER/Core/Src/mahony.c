/**
 * @file mahony.c
 * @brief Mahony 互补滤波姿态解算模块实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "mahony.h"
#include "mpu6050.h"
#include "qmc5883l.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Mahony_t g_mahony;
EulerAngle_t g_euler_angle = {0};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 快速平方根倒数 (Quake III 算法)
 * @note 使用 union 进行类型双关，避免违反严格别名规则
 */
float InvSqrt(float x)
{
    union {
        float f;
        int32_t i;
    } conv;
    
    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));   /* 第一次牛顿迭代 */
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));   /* 第二次牛顿迭代 (提高精度) */
    return conv.f;
}

/**
 * @brief 初始化 Mahony 滤波器
 */
void Mahony_Init(Mahony_t *mahony, float kp, float ki, float sample_freq)
{
    if (mahony == NULL) return;
    
    /* 初始化四元数为单位四元数 */
    mahony->q.q0 = 1.0f;
    mahony->q.q1 = 0.0f;
    mahony->q.q2 = 0.0f;
    mahony->q.q3 = 0.0f;
    
    /* 初始化欧拉角 */
    mahony->euler.roll = 0.0f;
    mahony->euler.pitch = 0.0f;
    mahony->euler.yaw = 0.0f;
    
    /* 初始化积分误差 */
    mahony->integralFBx = 0.0f;
    mahony->integralFBy = 0.0f;
    mahony->integralFBz = 0.0f;
    
    /* 设置增益 */
    mahony->kp = kp;
    mahony->ki = ki;
    mahony->sampleFreq = sample_freq;
    
    mahony->initialized = true;
}

/**
 * @brief 使用默认参数初始化
 */
void Mahony_InitDefault(void)
{
    Mahony_Init(&g_mahony, MAHONY_DEFAULT_KP, MAHONY_DEFAULT_KI, MAHONY_SAMPLE_FREQ);
}

/**
 * @brief 更新姿态 (仅使用 IMU 数据)
 */
void Mahony_UpdateIMU(Mahony_t *mahony, 
                       float gx, float gy, float gz,
                       float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    if (mahony == NULL || !mahony->initialized) return;
    
    /* 计算采样周期 */
    float dt = 1.0f / mahony->sampleFreq;
    
    /* 如果加速度计数据有效 (非零) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        /* 归一化加速度计测量值 */
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* 估计的重力方向 (从四元数计算) */
        halfvx = mahony->q.q1 * mahony->q.q3 - mahony->q.q0 * mahony->q.q2;
        halfvy = mahony->q.q0 * mahony->q.q1 + mahony->q.q2 * mahony->q.q3;
        halfvz = mahony->q.q0 * mahony->q.q0 - 0.5f + mahony->q.q3 * mahony->q.q3;
        
        /* 误差是估计方向与测量方向的叉积 */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        
        /* 计算并应用积分反馈 (如果启用) */
        if (mahony->ki > 0.0f) {
            mahony->integralFBx += 2.0f * mahony->ki * halfex * dt;
            mahony->integralFBy += 2.0f * mahony->ki * halfey * dt;
            mahony->integralFBz += 2.0f * mahony->ki * halfez * dt;
            gx += mahony->integralFBx;
            gy += mahony->integralFBy;
            gz += mahony->integralFBz;
        } else {
            mahony->integralFBx = 0.0f;
            mahony->integralFBy = 0.0f;
            mahony->integralFBz = 0.0f;
        }
        
        /* 应用比例反馈 */
        gx += 2.0f * mahony->kp * halfex;
        gy += 2.0f * mahony->kp * halfey;
        gz += 2.0f * mahony->kp * halfez;
    }
    
    /* 积分四元数微分方程 */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = mahony->q.q0;
    qb = mahony->q.q1;
    qc = mahony->q.q2;
    
    mahony->q.q0 += (-qb * gx - qc * gy - mahony->q.q3 * gz);
    mahony->q.q1 += (qa * gx + qc * gz - mahony->q.q3 * gy);
    mahony->q.q2 += (qa * gy - qb * gz + mahony->q.q3 * gx);
    mahony->q.q3 += (qa * gz + qb * gy - qc * gx);
    
    /* 归一化四元数 */
    Quaternion_Normalize(&mahony->q);
    
    /* 更新欧拉角 */
    mahony->euler = Quaternion_ToEuler(&mahony->q);
}

/**
 * @brief 更新姿态 (使用 IMU + 磁力计)
 */
void Mahony_UpdateAHRS(Mahony_t *mahony,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    if (mahony == NULL || !mahony->initialized) return;
    
    float dt = 1.0f / mahony->sampleFreq;
    
    /* 如果磁力计数据无效，使用 IMU 模式 */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        Mahony_UpdateIMU(mahony, gx, gy, gz, ax, ay, az);
        return;
    }
    
    /* 如果加速度计数据有效 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        /* 归一化加速度计 */
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* 归一化磁力计 */
        recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        /* 辅助变量 */
        q0q0 = mahony->q.q0 * mahony->q.q0;
        q0q1 = mahony->q.q0 * mahony->q.q1;
        q0q2 = mahony->q.q0 * mahony->q.q2;
        q0q3 = mahony->q.q0 * mahony->q.q3;
        q1q1 = mahony->q.q1 * mahony->q.q1;
        q1q2 = mahony->q.q1 * mahony->q.q2;
        q1q3 = mahony->q.q1 * mahony->q.q3;
        q2q2 = mahony->q.q2 * mahony->q.q2;
        q2q3 = mahony->q.q2 * mahony->q.q3;
        q3q3 = mahony->q.q3 * mahony->q.q3;
        
        /* 地球坐标系中的磁场参考方向 */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        
        /* 估计的重力和磁场方向 */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        
        /* 误差 */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        
        /* 积分反馈 */
        if (mahony->ki > 0.0f) {
            mahony->integralFBx += 2.0f * mahony->ki * halfex * dt;
            mahony->integralFBy += 2.0f * mahony->ki * halfey * dt;
            mahony->integralFBz += 2.0f * mahony->ki * halfez * dt;
            gx += mahony->integralFBx;
            gy += mahony->integralFBy;
            gz += mahony->integralFBz;
        } else {
            mahony->integralFBx = 0.0f;
            mahony->integralFBy = 0.0f;
            mahony->integralFBz = 0.0f;
        }
        
        /* 比例反馈 */
        gx += 2.0f * mahony->kp * halfex;
        gy += 2.0f * mahony->kp * halfey;
        gz += 2.0f * mahony->kp * halfez;
    }
    
    /* 积分四元数 */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = mahony->q.q0;
    qb = mahony->q.q1;
    qc = mahony->q.q2;
    
    mahony->q.q0 += (-qb * gx - qc * gy - mahony->q.q3 * gz);
    mahony->q.q1 += (qa * gx + qc * gz - mahony->q.q3 * gy);
    mahony->q.q2 += (qa * gy - qb * gz + mahony->q.q3 * gx);
    mahony->q.q3 += (qa * gz + qb * gy - qc * gx);
    
    Quaternion_Normalize(&mahony->q);
    mahony->euler = Quaternion_ToEuler(&mahony->q);
}

/**
 * @brief 使用全局 MPU6050 和 QMC5883L 数据更新姿态
 * @note 如果磁力计数据可用，使用 9 轴 AHRS 模式；否则使用 6 轴 IMU 模式
 */
void Mahony_Update(void)
{
    /* 将角速度从 °/s 转换为 rad/s */
    float gx = g_mpu6050_data.gyro_x * DEG_TO_RAD;
    float gy = g_mpu6050_data.gyro_y * DEG_TO_RAD;
    float gz = g_mpu6050_data.gyro_z * DEG_TO_RAD;
    
    /* 加速度已经是 g 单位 */
    float ax = g_mpu6050_data.accel_x;
    float ay = g_mpu6050_data.accel_y;
    float az = g_mpu6050_data.accel_z;
    
    /* 检查磁力计是否初始化且数据有效 */
    if (g_qmc5883l_data.initialized && g_qmc5883l_data.data_ready) {
        /* 使用 9 轴 AHRS 模式 (IMU + 磁力计) */
        float mx = g_qmc5883l_data.mag_x;
        float my = g_qmc5883l_data.mag_y;
        float mz = g_qmc5883l_data.mag_z;
        
        Mahony_UpdateAHRS(&g_mahony, gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        /* 使用 6 轴 IMU 模式 */
        Mahony_UpdateIMU(&g_mahony, gx, gy, gz, ax, ay, az);
    }
    
    /* 更新全局欧拉角 */
    g_euler_angle = g_mahony.euler;
}

/**
 * @brief 获取当前欧拉角
 */
EulerAngle_t Mahony_GetEuler(Mahony_t *mahony)
{
    if (mahony == NULL) {
        EulerAngle_t zero = {0, 0, 0};
        return zero;
    }
    return mahony->euler;
}

/**
 * @brief 获取当前四元数
 */
Quaternion_t Mahony_GetQuaternion(Mahony_t *mahony)
{
    if (mahony == NULL) {
        Quaternion_t identity = {1, 0, 0, 0};
        return identity;
    }
    return mahony->q;
}

/**
 * @brief 重置滤波器状态
 */
void Mahony_Reset(Mahony_t *mahony)
{
    if (mahony == NULL) return;
    
    mahony->q.q0 = 1.0f;
    mahony->q.q1 = 0.0f;
    mahony->q.q2 = 0.0f;
    mahony->q.q3 = 0.0f;
    
    mahony->euler.roll = 0.0f;
    mahony->euler.pitch = 0.0f;
    mahony->euler.yaw = 0.0f;
    
    mahony->integralFBx = 0.0f;
    mahony->integralFBy = 0.0f;
    mahony->integralFBz = 0.0f;
}

/**
 * @brief 设置 PI 增益
 */
void Mahony_SetGains(Mahony_t *mahony, float kp, float ki)
{
    if (mahony == NULL) return;
    mahony->kp = kp;
    mahony->ki = ki;
}

/**
 * @brief 四元数归一化
 */
void Quaternion_Normalize(Quaternion_t *q)
{
    if (q == NULL) return;
    
    float recipNorm = InvSqrt(q->q0 * q->q0 + q->q1 * q->q1 + 
                               q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 *= recipNorm;
    q->q1 *= recipNorm;
    q->q2 *= recipNorm;
    q->q3 *= recipNorm;
}

/**
 * @brief 四元数转欧拉角
 */
EulerAngle_t Quaternion_ToEuler(Quaternion_t *q)
{
    EulerAngle_t euler;
    
    if (q == NULL) {
        euler.roll = 0;
        euler.pitch = 0;
        euler.yaw = 0;
        return euler;
    }
    
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    euler.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f) {
        euler.pitch = copysignf(90.0f, sinp);  /* 使用 90 度 */
    } else {
        euler.pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    euler.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    
    return euler;
}