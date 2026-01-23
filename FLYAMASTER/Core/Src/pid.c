/**
 * @file pid.c
 * @brief PID 控制器实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* Exported variables --------------------------------------------------------*/
FlightPID_t g_flight_pid;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 限幅函数
 */
float PID_Constrain(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化单个 PID 控制器
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->d_filter_coeff = PID_D_FILTER_COEFF;
    
    /* 重置状态 */
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
    pid->enabled = true;
}

/**
 * @brief 初始化所有飞行 PID 控制器
 */
void FlightPID_Init(void)
{
    /* 角度环 (外环) */
    PID_Init(&g_flight_pid.roll_angle,
             PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI, PID_ROLL_ANGLE_KD,
             PID_ANGLE_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT);
    
    PID_Init(&g_flight_pid.pitch_angle,
             PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD,
             PID_ANGLE_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT);
    
    /* 角速度环 (内环) */
    PID_Init(&g_flight_pid.roll_rate,
             PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
             PID_RATE_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT);
    
    PID_Init(&g_flight_pid.pitch_rate,
             PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
             PID_RATE_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT);
    
    PID_Init(&g_flight_pid.yaw_rate,
             PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
             PID_RATE_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT);
}

/**
 * @brief 重置 PID 控制器状态
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 重置所有飞行 PID 控制器
 */
void FlightPID_Reset(void)
{
    PID_Reset(&g_flight_pid.roll_angle);
    PID_Reset(&g_flight_pid.pitch_angle);
    PID_Reset(&g_flight_pid.roll_rate);
    PID_Reset(&g_flight_pid.pitch_rate);
    PID_Reset(&g_flight_pid.yaw_rate);
}

/**
 * @brief 计算 PID 输出
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt)
{
    if (!pid->enabled || dt <= 0.0f) {
        return 0.0f;
    }
    
    /* 计算误差 */
    float error = setpoint - measurement;
    
    /* P 项 */
    float p_term = pid->kp * error;
    
    /* I 项 (带积分限幅) */
    pid->integral += error * dt;
    /* 防止除零：只有 ki > 0 时才进行积分限幅 */
    if (pid->ki > 0.0001f) {
        pid->integral = PID_Constrain(pid->integral, pid->integral_limit / pid->ki);
    }
    float i_term = pid->ki * pid->integral;
    
    /* D 项 (带低通滤波) */
    float derivative = (error - pid->prev_error) / dt;
    /* 一阶低通滤波: y = alpha * x + (1-alpha) * y_prev */
    derivative = pid->d_filter_coeff * derivative + 
                 (1.0f - pid->d_filter_coeff) * pid->prev_derivative;
    float d_term = pid->kd * derivative;
    
    /* 保存状态 */
    pid->prev_error = error;
    pid->prev_derivative = derivative;
    
    /* 计算总输出 */
    pid->output = p_term + i_term + d_term;
    
    /* 输出限幅 */
    pid->output = PID_Constrain(pid->output, pid->output_limit);
    
    return pid->output;
}

/**
 * @brief 计算串级 PID
 */
void FlightPID_Compute(float target_roll, float target_pitch, float target_yaw_rate,
                       float current_roll, float current_pitch,
                       float gyro_x, float gyro_y, float gyro_z,
                       float dt, PID_Output_t *output)
{
    /* 外环：角度环 -> 目标角速度 */
    float target_roll_rate = PID_Compute(&g_flight_pid.roll_angle, 
                                          target_roll, current_roll, dt);
    float target_pitch_rate = PID_Compute(&g_flight_pid.pitch_angle, 
                                           target_pitch, current_pitch, dt);
    
    /* 内环：角速度环 -> 电机输出 */
    output->roll = PID_Compute(&g_flight_pid.roll_rate, 
                               target_roll_rate, gyro_x, dt);
    output->pitch = PID_Compute(&g_flight_pid.pitch_rate, 
                                target_pitch_rate, gyro_y, dt);
    output->yaw = PID_Compute(&g_flight_pid.yaw_rate, 
                              target_yaw_rate, gyro_z, dt);
}

/**
 * @brief 设置 PID 参数
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置横滚角度环参数
 */
void FlightPID_SetRollAngleGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_flight_pid.roll_angle, kp, ki, kd);
}

/**
 * @brief 设置俯仰角度环参数
 */
void FlightPID_SetPitchAngleGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_flight_pid.pitch_angle, kp, ki, kd);
}

/**
 * @brief 设置横滚角速度环参数
 */
void FlightPID_SetRollRateGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_flight_pid.roll_rate, kp, ki, kd);
}

/**
 * @brief 设置俯仰角速度环参数
 */
void FlightPID_SetPitchRateGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_flight_pid.pitch_rate, kp, ki, kd);
}

/**
 * @brief 设置偏航角速度环参数
 */
void FlightPID_SetYawRateGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_flight_pid.yaw_rate, kp, ki, kd);
}

/**
 * @brief 使能/禁用 PID 控制器
 */
void PID_Enable(PID_Controller_t *pid, bool enable)
{
    pid->enabled = enable;
    if (!enable) {
        PID_Reset(pid);
    }
}