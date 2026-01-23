/**
 * @file pid.h
 * @brief PID 控制器模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 实现串级 PID 控制：
 * 1. 外环：角度环 (目标角度 -> 目标角速度)
 * 2. 内环：角速度环 (目标角速度 -> 电机输出)
 * 
 * 特性：
 * - 积分限幅 (Anti-windup)
 * - 微分滤波 (D-term filtering)
 * - 输出限幅
 */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* PID 参数默认值 (需要根据实际调试调整) */
/* 角度环 (外环) */
#define PID_ROLL_ANGLE_KP       4.0f
#define PID_ROLL_ANGLE_KI       0.02f
#define PID_ROLL_ANGLE_KD       0.0f

#define PID_PITCH_ANGLE_KP      4.0f
#define PID_PITCH_ANGLE_KI      0.02f
#define PID_PITCH_ANGLE_KD      0.0f

/* 角速度环 (内环) */
#define PID_ROLL_RATE_KP        0.7f
#define PID_ROLL_RATE_KI        0.3f
#define PID_ROLL_RATE_KD        0.02f

#define PID_PITCH_RATE_KP       0.7f
#define PID_PITCH_RATE_KI       0.3f
#define PID_PITCH_RATE_KD       0.02f

#define PID_YAW_RATE_KP         2.0f
#define PID_YAW_RATE_KI         0.5f
#define PID_YAW_RATE_KD         0.0f

/* 限幅参数 */
#define PID_ANGLE_OUTPUT_LIMIT  500.0f      /**< 角度环输出限幅 (deg/s) */
#define PID_RATE_OUTPUT_LIMIT   500.0f      /**< 角速度环输出限幅 */
#define PID_INTEGRAL_LIMIT      200.0f      /**< 积分限幅 */

/* 微分滤波系数 */
#define PID_D_FILTER_COEFF      0.5f        /**< 微分低通滤波系数 (0-1) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID 控制器结构体
 */
typedef struct {
    /* 参数 */
    float kp;               /**< 比例系数 */
    float ki;               /**< 积分系数 */
    float kd;               /**< 微分系数 */
    
    /* 限幅 */
    float output_limit;     /**< 输出限幅 */
    float integral_limit;   /**< 积分限幅 */
    
    /* 状态 */
    float integral;         /**< 积分累积 */
    float prev_error;       /**< 上次误差 */
    float prev_derivative;  /**< 上次微分 (用于滤波) */
    float output;           /**< 当前输出 */
    
    /* 配置 */
    float d_filter_coeff;   /**< 微分滤波系数 */
    bool enabled;           /**< 使能标志 */
} PID_Controller_t;

/**
 * @brief 飞行控制 PID 组
 */
typedef struct {
    /* 角度环 (外环) */
    PID_Controller_t roll_angle;
    PID_Controller_t pitch_angle;
    
    /* 角速度环 (内环) */
    PID_Controller_t roll_rate;
    PID_Controller_t pitch_rate;
    PID_Controller_t yaw_rate;
} FlightPID_t;

/**
 * @brief PID 输出结构体
 */
typedef struct {
    float roll;     /**< 横滚输出 */
    float pitch;    /**< 俯仰输出 */
    float yaw;      /**< 偏航输出 */
} PID_Output_t;

/* Exported variables --------------------------------------------------------*/
extern FlightPID_t g_flight_pid;    /**< 全局飞行 PID 控制器 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化单个 PID 控制器
 * @param pid PID 控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_limit 输出限幅
 * @param integral_limit 积分限幅
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit);

/**
 * @brief 初始化所有飞行 PID 控制器
 */
void FlightPID_Init(void);

/**
 * @brief 重置 PID 控制器状态
 * @param pid PID 控制器指针
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief 重置所有飞行 PID 控制器
 */
void FlightPID_Reset(void);

/**
 * @brief 计算 PID 输出
 * @param pid PID 控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param dt 时间间隔 (秒)
 * @return float PID 输出
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 计算串级 PID (角度环 + 角速度环)
 * @param target_roll 目标横滚角 (度)
 * @param target_pitch 目标俯仰角 (度)
 * @param target_yaw_rate 目标偏航角速度 (度/秒)
 * @param current_roll 当前横滚角 (度)
 * @param current_pitch 当前俯仰角 (度)
 * @param gyro_x 陀螺仪 X 轴 (度/秒)
 * @param gyro_y 陀螺仪 Y 轴 (度/秒)
 * @param gyro_z 陀螺仪 Z 轴 (度/秒)
 * @param dt 时间间隔 (秒)
 * @param output 输出结构体指针
 */
void FlightPID_Compute(float target_roll, float target_pitch, float target_yaw_rate,
                       float current_roll, float current_pitch,
                       float gyro_x, float gyro_y, float gyro_z,
                       float dt, PID_Output_t *output);

/**
 * @brief 设置 PID 参数
 * @param pid PID 控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置横滚角度环参数
 */
void FlightPID_SetRollAngleGains(float kp, float ki, float kd);

/**
 * @brief 设置俯仰角度环参数
 */
void FlightPID_SetPitchAngleGains(float kp, float ki, float kd);

/**
 * @brief 设置横滚角速度环参数
 */
void FlightPID_SetRollRateGains(float kp, float ki, float kd);

/**
 * @brief 设置俯仰角速度环参数
 */
void FlightPID_SetPitchRateGains(float kp, float ki, float kd);

/**
 * @brief 设置偏航角速度环参数
 */
void FlightPID_SetYawRateGains(float kp, float ki, float kd);

/**
 * @brief 使能/禁用 PID 控制器
 * @param pid PID 控制器指针
 * @param enable true=使能, false=禁用
 */
void PID_Enable(PID_Controller_t *pid, bool enable);

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param limit 限幅值 (正负对称)
 * @return float 限幅后的值
 */
float PID_Constrain(float value, float limit);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */