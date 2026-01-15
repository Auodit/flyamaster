#include "pid.h"
#include <stddef.h>

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i) {
    // Risk #059 修复：添加指针空检查
    if (pid == NULL) return;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_i = max_i;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = 0.0f;
}

/**
 * @brief 重置 PID 状态
 */
void PID_Reset(PID_TypeDef *pid) {
    // Risk #060 修复：添加指针空检查
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = 0.0f;
}

/**
 * @brief 计算 PID 输出
 * @param target 期望值
 * @param measured 测量值
 * @param dt 采样时间 (秒)
 */
float PID_Calculate(PID_TypeDef *pid, float target, float measured, float dt) {
    // Risk #061 修复：添加指针空检查
    if (pid == NULL) return 0.0f;
    
    // 防止除零错误（任务调度异常时 dt 可能为0）
    if (dt < 0.0001f) dt = 0.0001f;
    
    float error = target - measured;
    float p_out, i_out, d_out, output;

    // P 项
    p_out = pid->Kp * error;

    // I 项 (带积分限幅)
    pid->integral += error * dt;
    if (pid->integral > pid->max_i) pid->integral = pid->max_i;
    else if (pid->integral < -pid->max_i) pid->integral = -pid->max_i;
    i_out = pid->Ki * pid->integral;

    // D 项 (使用测量值的微分，即微分先行，避免设定值突变引起的冲击)
    // d_out = pid->Kd * (error - pid->prev_error) / dt; // 经典微分
    d_out = -pid->Kd * (measured - pid->prev_measure) / dt; // 微分先行

    output = p_out + i_out + d_out;

    // 输出限幅
    if (output > pid->max_out) output = pid->max_out;
    else if (output < -pid->max_out) output = -pid->max_out;

    // 更新状态
    pid->prev_error = error;
    pid->prev_measure = measured;

    return output;
}
