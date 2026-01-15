#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float max_out;      // 输出限幅
    float max_i;        // 积分限幅
    
    float integral;     // 积分累加值
    float prev_error;   // 上一次误差
    float prev_measure; // 上一次测量值 (用于微分先行)
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i);
void PID_Reset(PID_TypeDef *pid);
float PID_Calculate(PID_TypeDef *pid, float target, float measured, float dt);

#endif
