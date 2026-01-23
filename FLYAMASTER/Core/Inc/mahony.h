/**
 * @file mahony.h
 * @brief Mahony 互补滤波姿态解算模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 实现 Mahony 互补滤波算法：
 * 1. 融合加速度计和陀螺仪数据
 * 2. 输出欧拉角 (Roll, Pitch, Yaw)
 * 3. 支持 500Hz 更新率
 * 
 * 算法原理：
 * - 使用加速度计修正陀螺仪积分漂移
 * - PI 控制器调节误差
 * - 四元数表示姿态，避免万向锁
 */

#ifndef __MAHONY_H
#define __MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Exported defines ----------------------------------------------------------*/
#define MAHONY_DEFAULT_KP       2.0f    /**< 比例增益 */
#define MAHONY_DEFAULT_KI       0.005f  /**< 积分增益 */
#define MAHONY_SAMPLE_FREQ      500.0f  /**< 采样频率 (Hz) */

/* 角度转换 */
#define DEG_TO_RAD              0.017453292519943295f   /**< π/180 */
#define RAD_TO_DEG              57.29577951308232f      /**< 180/π */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 欧拉角结构体
 */
typedef struct {
    float roll;     /**< 横滚角 (deg) */
    float pitch;    /**< 俯仰角 (deg) */
    float yaw;      /**< 偏航角 (deg) */
} EulerAngle_t;

/**
 * @brief 四元数结构体
 */
typedef struct {
    float q0;   /**< w */
    float q1;   /**< x */
    float q2;   /**< y */
    float q3;   /**< z */
} Quaternion_t;

/**
 * @brief Mahony 滤波器状态
 */
typedef struct {
    Quaternion_t q;         /**< 当前四元数 */
    EulerAngle_t euler;     /**< 当前欧拉角 */
    float integralFBx;      /**< 积分误差 X */
    float integralFBy;      /**< 积分误差 Y */
    float integralFBz;      /**< 积分误差 Z */
    float kp;               /**< 比例增益 */
    float ki;               /**< 积分增益 */
    float sampleFreq;       /**< 采样频率 */
    bool initialized;       /**< 初始化标志 */
} Mahony_t;

/* Exported variables --------------------------------------------------------*/
extern Mahony_t g_mahony;           /**< 全局 Mahony 滤波器 */
extern EulerAngle_t g_euler_angle;  /**< 全局欧拉角 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 Mahony 滤波器
 * @param mahony 滤波器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param sample_freq 采样频率 (Hz)
 */
void Mahony_Init(Mahony_t *mahony, float kp, float ki, float sample_freq);

/**
 * @brief 使用默认参数初始化
 */
void Mahony_InitDefault(void);

/**
 * @brief 更新姿态 (仅使用 IMU 数据)
 * @param mahony 滤波器指针
 * @param gx 陀螺仪 X (rad/s)
 * @param gy 陀螺仪 Y (rad/s)
 * @param gz 陀螺仪 Z (rad/s)
 * @param ax 加速度 X (g)
 * @param ay 加速度 Y (g)
 * @param az 加速度 Z (g)
 */
void Mahony_UpdateIMU(Mahony_t *mahony, 
                       float gx, float gy, float gz,
                       float ax, float ay, float az);

/**
 * @brief 更新姿态 (使用 IMU + 磁力计)
 * @param mahony 滤波器指针
 * @param gx 陀螺仪 X (rad/s)
 * @param gy 陀螺仪 Y (rad/s)
 * @param gz 陀螺仪 Z (rad/s)
 * @param ax 加速度 X (g)
 * @param ay 加速度 Y (g)
 * @param az 加速度 Z (g)
 * @param mx 磁力计 X
 * @param my 磁力计 Y
 * @param mz 磁力计 Z
 */
void Mahony_UpdateAHRS(Mahony_t *mahony,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz);

/**
 * @brief 使用全局 MPU6050 数据更新姿态
 * @note 自动从 g_mpu6050_data 读取数据
 */
void Mahony_Update(void);

/**
 * @brief 获取当前欧拉角
 * @param mahony 滤波器指针
 * @return EulerAngle_t 欧拉角
 */
EulerAngle_t Mahony_GetEuler(Mahony_t *mahony);

/**
 * @brief 获取当前四元数
 * @param mahony 滤波器指针
 * @return Quaternion_t 四元数
 */
Quaternion_t Mahony_GetQuaternion(Mahony_t *mahony);

/**
 * @brief 重置滤波器状态
 * @param mahony 滤波器指针
 */
void Mahony_Reset(Mahony_t *mahony);

/**
 * @brief 设置 PI 增益
 * @param mahony 滤波器指针
 * @param kp 比例增益
 * @param ki 积分增益
 */
void Mahony_SetGains(Mahony_t *mahony, float kp, float ki);

/**
 * @brief 四元数归一化
 * @param q 四元数指针
 */
void Quaternion_Normalize(Quaternion_t *q);

/**
 * @brief 四元数转欧拉角
 * @param q 四元数
 * @return EulerAngle_t 欧拉角 (deg)
 */
EulerAngle_t Quaternion_ToEuler(Quaternion_t *q);

/**
 * @brief 快速平方根倒数 (Quake III 算法)
 * @param x 输入值
 * @return float 1/sqrt(x)
 */
float InvSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /* __MAHONY_H */