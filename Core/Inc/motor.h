#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// 电机 PWM 范围 (1000~2000us)
#define MOTOR_MIN_PWM 1000
#define MOTOR_MAX_PWM 2000
#define MOTOR_IDLE_PWM 1100 // 怠速

// 电机布局定义 (标准 X 型)
// M1: 右前 CCW (TIM3_CH1 - PC6)
// M2: 左前 CW  (TIM3_CH2 - PC7)  <-- 修正为 CW
// M3: 左后 CCW (TIM4_CH3 - PD14) <-- 修正为 CCW
// M4: 右后 CW  (TIM4_CH4 - PD15)

/**
 * @brief 初始化电机 PWM
 * @param htim3 TIM3 句柄 (控制 M1, M2)
 * @param htim4 TIM4 句柄 (控制 M3, M4)
 */
void Motor_Init(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);
void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void Motor_Stop(void);

/**
 * @brief 电机混控函数 (X 型布局)
 * @param throttle 油门输出 (0.0 ~ 1.0)
 * @param roll     横滚控制量 (-1.0 ~ 1.0)
 * @param pitch    俯仰控制量 (-1.0 ~ 1.0)
 * @param yaw      偏航控制量 (-1.0 ~ 1.0)
 */
void Motor_Mix(float throttle, float roll, float pitch, float yaw);

/**
 * @brief 设置 AirMode 怠速保护状态
 * @param enable 1=启用（解锁后保持怠速），0=禁用（完全停机）
 * @note AirMode 启用时，即使油门为0，电机也保持 MOTOR_IDLE_PWM 运转
 *       防止空中因油门过低导致姿态失控
 */
void Motor_SetAirMode(uint8_t enable);

// 全局变量：当前电机PWM值 (用于上位机监控)
extern uint16_t g_motor_pwm[4];

#endif
