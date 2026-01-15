#include "motor.h"

static TIM_HandleTypeDef *motor_htim3;  // TIM3: M1(CH1), M2(CH2)
static TIM_HandleTypeDef *motor_htim4;  // TIM4: M3(CH3), M4(CH4)
static uint8_t g_airmode_enabled = 0;   // AirMode 状态标志

// 全局变量：当前电机PWM值 (用于上位机监控)
uint16_t g_motor_pwm[4] = {MOTOR_MIN_PWM, MOTOR_MIN_PWM, MOTOR_MIN_PWM, MOTOR_MIN_PWM};

// 辅助宏: 浮点数限幅
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 初始化电机 PWM
 * @param htim3 TIM3 句柄 (控制 M1, M2 - PC6, PC7)
 * @param htim4 TIM4 句柄 (控制 M3, M4 - PD14, PD15)
 */
void Motor_Init(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4) {
    // Risk #057 修复：添加指针空检查
    if (htim3 == NULL || htim4 == NULL) return;
    
    motor_htim3 = htim3;
    motor_htim4 = htim4;
    
    // Risk #057 修复：检查 HAL_TIM_PWM_Start 返回值
    // TIM3: M1(CH1), M2(CH2)
    if (HAL_TIM_PWM_Start(motor_htim3, TIM_CHANNEL_1) != HAL_OK) return;
    if (HAL_TIM_PWM_Start(motor_htim3, TIM_CHANNEL_2) != HAL_OK) return;
    
    // TIM4: M3(CH3), M4(CH4)
    if (HAL_TIM_PWM_Start(motor_htim4, TIM_CHANNEL_3) != HAL_OK) return;
    if (HAL_TIM_PWM_Start(motor_htim4, TIM_CHANNEL_4) != HAL_OK) return;
    
    Motor_Stop();
}

/**
 * @brief 设置 4 个电机的 PWM 值
 * @param m1 电机1 PWM (右前, TIM3_CH1)
 * @param m2 电机2 PWM (左前, TIM3_CH2)
 * @param m3 电机3 PWM (左后, TIM4_CH3)
 * @param m4 电机4 PWM (右后, TIM4_CH4)
 */
void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    // Risk #058 修复：添加句柄空检查
    if (motor_htim3 == NULL || motor_htim4 == NULL) return;
    
    // 限幅保护
    if (m1 > MOTOR_MAX_PWM) m1 = MOTOR_MAX_PWM; else if (m1 < MOTOR_MIN_PWM) m1 = MOTOR_MIN_PWM;
    if (m2 > MOTOR_MAX_PWM) m2 = MOTOR_MAX_PWM; else if (m2 < MOTOR_MIN_PWM) m2 = MOTOR_MIN_PWM;
    if (m3 > MOTOR_MAX_PWM) m3 = MOTOR_MAX_PWM; else if (m3 < MOTOR_MIN_PWM) m3 = MOTOR_MIN_PWM;
    if (m4 > MOTOR_MAX_PWM) m4 = MOTOR_MAX_PWM; else if (m4 < MOTOR_MIN_PWM) m4 = MOTOR_MIN_PWM;

    // 保存到全局数组 (用于上位机监控)
    g_motor_pwm[0] = m1;
    g_motor_pwm[1] = m2;
    g_motor_pwm[2] = m3;
    g_motor_pwm[3] = m4;

    // TIM3: M1(CH1), M2(CH2)
    __HAL_TIM_SET_COMPARE(motor_htim3, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(motor_htim3, TIM_CHANNEL_2, m2);
    
    // TIM4: M3(CH3), M4(CH4)
    __HAL_TIM_SET_COMPARE(motor_htim4, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(motor_htim4, TIM_CHANNEL_4, m4);
}

/**
 * @brief 停止电机 (输出最小 PWM)
 */
void Motor_Stop(void) {
    Motor_SetPWM(MOTOR_MIN_PWM, MOTOR_MIN_PWM, MOTOR_MIN_PWM, MOTOR_MIN_PWM);
}

/**
 * @brief 设置 AirMode 怠速保护状态
 * @param enable 1=启用（解锁后保持怠速），0=禁用（完全停机）
 */
void Motor_SetAirMode(uint8_t enable) {
    g_airmode_enabled = enable;
}

/**
 * @brief 电机混控函数 (X 型布局)
 *
 * 电机布局:
 *       前
 *    M1     M2
 *      \   /
 *       \ /
 *        X
 *       / \
 *      /   \
 *    M3     M4
 *       后
 *
 * M1: 右前 CCW (逆时针) - 产生向左的反扭矩 (CW torque on body)
 * M2: 左前 CW  (顺时针) - 产生向右的反扭矩 (CCW torque on body)
 * M3: 左后 CCW (逆时针) - 产生向左的反扭矩 (CW torque on body)
 * M4: 右后 CW  (顺时针) - 产生向右的反扭矩 (CCW torque on body)
 *
 * ⚠️ 注意：代码实现的是标准 X 型布局 (Betaflight/Cleanflight 标准)
 * - M1 (FR) & M3 (RL) 是 CCW
 * - M2 (FL) & M4 (RR) 是 CW
 * - 原注释称 M2=CCW, M3=CW 与代码逻辑冲突，已修正为标准布局描述。
 * - 请务必检查实际电机转向是否与此匹配！否则 Yaw 轴会反转/失控。
 *
 * 混控矩阵:
 * M1 (FR, CCW) = throttle - roll + pitch + yaw  (Yaw helps CW turn)
 * M2 (FL, CW)  = throttle + roll + pitch - yaw  (Yaw opposes CW turn)
 * M3 (RL, CCW) = throttle + roll - pitch + yaw  (Yaw helps CW turn)
 * M4 (RR, CW)  = throttle - roll - pitch - yaw  (Yaw opposes CW turn)
 *
 * @param throttle 油门输出 (0.0 ~ 1.0)
 * @param roll     横滚控制量 (-1.0 ~ 1.0), 正值向右滚
 * @param pitch    俯仰控制量 (-1.0 ~ 1.0), 正值抬头
 * @param yaw      偏航控制量 (-1.0 ~ 1.0), 正值顺时针转
 */
void Motor_Mix(float throttle, float roll, float pitch, float yaw) {
    float m1, m2, m3, m4;
    
    // 缩放控制量 (控制量范围通常比油门小，这里用 0.3 作为最大混控比例)
    float mix_scale = 0.3f;
    roll *= mix_scale;
    pitch *= mix_scale;
    yaw *= mix_scale;
    
    // X 型混控计算
    m1 = throttle - roll + pitch + yaw;
    m2 = throttle + roll + pitch - yaw;
    m3 = throttle + roll - pitch + yaw;
    m4 = throttle - roll - pitch - yaw;
    
    // 浮点数限幅保护（防止负值或超限导致整数溢出）
    m1 = CONSTRAIN(m1, 0.0f, 1.0f);
    m2 = CONSTRAIN(m2, 0.0f, 1.0f);
    m3 = CONSTRAIN(m3, 0.0f, 1.0f);
    m4 = CONSTRAIN(m4, 0.0f, 1.0f);
    
    // 转换为 PWM 值
    // AirMode 启用时：使用 MOTOR_IDLE_PWM (1100) 作为最小值，防止空中失控
    // AirMode 禁用时：使用 MOTOR_MIN_PWM (1000)，允许完全停机
    uint16_t min_pwm = g_airmode_enabled ? MOTOR_IDLE_PWM : MOTOR_MIN_PWM;
    uint16_t pwm_range = MOTOR_MAX_PWM - min_pwm;
    
    uint16_t pwm1 = min_pwm + (uint16_t)(m1 * pwm_range);
    uint16_t pwm2 = min_pwm + (uint16_t)(m2 * pwm_range);
    uint16_t pwm3 = min_pwm + (uint16_t)(m3 * pwm_range);
    uint16_t pwm4 = min_pwm + (uint16_t)(m4 * pwm_range);
    
    Motor_SetPWM(pwm1, pwm2, pwm3, pwm4);
}
