/**
 * @file motor.c
 * @brief 电机控制实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "tim.h"

/* Exported variables --------------------------------------------------------*/
Motor_Output_t g_motor_output = {
    .pwm = {MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN},
    .ccr = {MOTOR_CCR_MIN, MOTOR_CCR_MIN, MOTOR_CCR_MIN, MOTOR_CCR_MIN},
    .throttle = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .state = MOTOR_STATE_DISARMED,
    .airmode_enabled = false
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 限幅函数
 */
uint16_t Motor_Constrain(int32_t value, uint16_t min, uint16_t max)
{
    if (value < (int32_t)min) return min;
    if (value > (int32_t)max) return max;
    return (uint16_t)value;
}

/**
 * @brief 将 PWM 值转换为 CCR 值
 */
uint16_t Motor_PWMtoCCR(uint16_t pwm)
{
    /* PWM 1000-2000us -> CCR 2000-4000 */
    /* CCR = PWM * 2 */
    return Motor_Constrain((int32_t)pwm * 2, MOTOR_CCR_MIN, MOTOR_CCR_MAX);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电机控制
 */
void Motor_Init(void)
{
    /* 设置初始 PWM 为最小值 */
    Motor_StopAll();
    
    /* 启动 TIM8 PWM 输出 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    
    g_motor_output.state = MOTOR_STATE_DISARMED;
}

/**
 * @brief 解锁电机
 */
void Motor_Arm(void)
{
    if (g_motor_output.state == MOTOR_STATE_DISARMED) {
        g_motor_output.state = MOTOR_STATE_ARMED;
        
        /* 设置怠速 PWM */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            g_motor_output.pwm[i] = MOTOR_PWM_IDLE;
            g_motor_output.ccr[i] = Motor_PWMtoCCR(MOTOR_PWM_IDLE);
        }
        
        Motor_ApplyOutput();
    }
}

/**
 * @brief 锁定电机
 */
void Motor_Disarm(void)
{
    g_motor_output.state = MOTOR_STATE_DISARMED;
    Motor_StopAll();
}

/**
 * @brief 检查电机是否解锁
 */
bool Motor_IsArmed(void)
{
    return (g_motor_output.state != MOTOR_STATE_DISARMED);
}

/**
 * @brief 设置单个电机 PWM
 */
void Motor_SetPWM(Motor_Index_t motor, uint16_t pwm)
{
    if (motor >= MOTOR_COUNT) return;
    
    pwm = Motor_Constrain(pwm, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    g_motor_output.pwm[motor] = pwm;
    g_motor_output.ccr[motor] = Motor_PWMtoCCR(pwm);
}

/**
 * @brief 设置所有电机 PWM
 */
void Motor_SetAllPWM(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4)
{
    Motor_SetPWM(MOTOR_1, pwm1);
    Motor_SetPWM(MOTOR_2, pwm2);
    Motor_SetPWM(MOTOR_3, pwm3);
    Motor_SetPWM(MOTOR_4, pwm4);
}

/**
 * @brief 电机混控计算 (X 型机架)
 * 
 * 混控公式:
 * M1 (右前, CCW): Throttle - Roll - Pitch - Yaw
 * M2 (左后, CCW): Throttle + Roll + Pitch - Yaw
 * M3 (左前, CW):  Throttle + Roll - Pitch + Yaw
 * M4 (右后, CW):  Throttle - Roll + Pitch + Yaw
 */
void Motor_Mix(float throttle, float roll, float pitch, float yaw)
{
    if (g_motor_output.state == MOTOR_STATE_DISARMED) {
        Motor_StopAll();
        return;
    }
    
    g_motor_output.state = MOTOR_STATE_RUNNING;
    g_motor_output.throttle = throttle;
    g_motor_output.roll = roll;
    g_motor_output.pitch = pitch;
    g_motor_output.yaw = yaw;
    
    /* 油门映射: 0.0-1.0 -> 1000-2000 */
    float throttle_pwm = MOTOR_PWM_MIN + throttle * (MOTOR_PWM_MAX - MOTOR_PWM_MIN);
    
    /* 混控计算 */
    float motor_out[MOTOR_COUNT];
    motor_out[MOTOR_1] = throttle_pwm - roll - pitch - yaw;  /* 右前 CCW */
    motor_out[MOTOR_2] = throttle_pwm + roll + pitch - yaw;  /* 左后 CCW */
    motor_out[MOTOR_3] = throttle_pwm + roll - pitch + yaw;  /* 左前 CW */
    motor_out[MOTOR_4] = throttle_pwm - roll + pitch + yaw;  /* 右后 CW */
    
    /* Airmode 处理 */
    if (g_motor_output.airmode_enabled) {
        /* 找出最大和最小值 */
        float max_out = motor_out[0];
        float min_out = motor_out[0];
        for (int i = 1; i < MOTOR_COUNT; i++) {
            if (motor_out[i] > max_out) max_out = motor_out[i];
            if (motor_out[i] < min_out) min_out = motor_out[i];
        }
        
        /* 计算需要的偏移量 */
        float offset = 0.0f;
        if (max_out > MOTOR_PWM_MAX) {
            offset = MOTOR_PWM_MAX - max_out;
        } else if (min_out < MOTOR_PWM_IDLE) {
            offset = MOTOR_PWM_IDLE - min_out;
        }
        
        /* 应用偏移 */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor_out[i] += offset;
        }
    }
    
    /* 限幅并设置 PWM */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint16_t pwm;
        if (throttle < 0.05f && !g_motor_output.airmode_enabled) {
            /* 低油门时使用怠速 */
            pwm = MOTOR_PWM_IDLE;
        } else {
            pwm = Motor_Constrain((int32_t)motor_out[i], MOTOR_PWM_IDLE, MOTOR_PWM_MAX);
        }
        g_motor_output.pwm[i] = pwm;
        g_motor_output.ccr[i] = Motor_PWMtoCCR(pwm);
    }
}

/**
 * @brief 应用电机输出
 */
void Motor_ApplyOutput(void)
{
    if (g_motor_output.state == MOTOR_STATE_DISARMED) {
        /* 锁定状态，输出最小值 */
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_CCR_MIN);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_CCR_MIN);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, MOTOR_CCR_MIN);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MOTOR_CCR_MIN);
    } else {
        /* 输出混控结果 */
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, g_motor_output.ccr[MOTOR_1]);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, g_motor_output.ccr[MOTOR_2]);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, g_motor_output.ccr[MOTOR_3]);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, g_motor_output.ccr[MOTOR_4]);
    }
}

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motor_output.pwm[i] = MOTOR_PWM_MIN;
        g_motor_output.ccr[i] = MOTOR_CCR_MIN;
    }
    
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_CCR_MIN);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_CCR_MIN);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, MOTOR_CCR_MIN);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MOTOR_CCR_MIN);
}

/**
 * @brief 使能/禁用 Airmode
 */
void Motor_SetAirmode(bool enable)
{
    g_motor_output.airmode_enabled = enable;
}

/**
 * @brief 获取电机 PWM 值
 */
uint16_t Motor_GetPWM(Motor_Index_t motor)
{
    if (motor >= MOTOR_COUNT) return MOTOR_PWM_MIN;
    return g_motor_output.pwm[motor];
}

/* 电机测试状态机变量 */
static struct {
    bool active;                /**< 测试是否激活 */
    Motor_Index_t motor;        /**< 当前测试的电机 */
    uint32_t start_time;        /**< 测试开始时间 */
    uint32_t duration_ms;       /**< 测试持续时间 */
    Motor_State_t prev_state;   /**< 测试前的电机状态 */
} s_motor_test = {0};

/**
 * @brief 启动电机测试 (非阻塞)
 * @param motor 电机索引
 * @param duration_ms 持续时间 (ms)
 * @return bool true=启动成功, false=已有测试在进行
 */
bool Motor_TestStart(Motor_Index_t motor, uint32_t duration_ms)
{
    if (motor >= MOTOR_COUNT) return false;
    if (s_motor_test.active) return false;  /* 已有测试在进行 */
    
    /* 保存当前状态 */
    s_motor_test.prev_state = g_motor_output.state;
    s_motor_test.motor = motor;
    s_motor_test.duration_ms = duration_ms;
    s_motor_test.start_time = HAL_GetTick();
    s_motor_test.active = true;
    
    /* 临时解锁 */
    g_motor_output.state = MOTOR_STATE_ARMED;
    
    /* 设置测试电机 */
    Motor_StopAll();
    Motor_SetPWM(motor, MOTOR_PWM_IDLE + 100);
    Motor_ApplyOutput();
    
    return true;
}

/**
 * @brief 更新电机测试状态机 (非阻塞)
 * @note 需要在主循环或任务中周期性调用
 * @return bool true=测试仍在进行, false=测试已完成或未激活
 */
bool Motor_TestUpdate(void)
{
    if (!s_motor_test.active) return false;
    
    uint32_t elapsed = HAL_GetTick() - s_motor_test.start_time;
    
    if (elapsed >= s_motor_test.duration_ms) {
        /* 测试完成 */
        Motor_StopAll();
        Motor_ApplyOutput();
        
        /* 恢复状态 */
        g_motor_output.state = s_motor_test.prev_state;
        s_motor_test.active = false;
        
        return false;
    }
    
    return true;
}

/**
 * @brief 检查电机测试是否正在进行
 * @return bool true=测试中, false=空闲
 */
bool Motor_IsTestActive(void)
{
    return s_motor_test.active;
}

/**
 * @brief 停止电机测试
 */
void Motor_TestStop(void)
{
    if (!s_motor_test.active) return;
    
    Motor_StopAll();
    Motor_ApplyOutput();
    g_motor_output.state = s_motor_test.prev_state;
    s_motor_test.active = false;
}

/**
 * @brief 电机测试 (阻塞方式，仅用于非 RTOS 环境)
 * @deprecated 在 FreeRTOS 环境中请使用 Motor_TestStart() + Motor_TestUpdate()
 */
void Motor_Test(Motor_Index_t motor, uint32_t duration_ms)
{
    if (motor >= MOTOR_COUNT) return;
    
    /* 保存当前状态 */
    Motor_State_t prev_state = g_motor_output.state;
    
    /* 临时解锁 */
    g_motor_output.state = MOTOR_STATE_ARMED;
    
    /* 设置测试电机 */
    Motor_StopAll();
    Motor_SetPWM(motor, MOTOR_PWM_IDLE + 100);
    Motor_ApplyOutput();
    
    /* 等待 - 注意：在 FreeRTOS 环境中应使用 osDelay */
    #ifdef USE_FREERTOS
    osDelay(duration_ms);
    #else
    HAL_Delay(duration_ms);
    #endif
    
    /* 停止 */
    Motor_StopAll();
    Motor_ApplyOutput();
    
    /* 恢复状态 */
    g_motor_output.state = prev_state;
}