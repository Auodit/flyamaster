/**
 * @file motor.h
 * @brief 电机控制模块 (PWM 输出 + 混控)
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 实现四轴飞行器电机控制：
 * 1. TIM8 CH1-CH4 输出 400Hz PWM
 * 2. X 型机架混控算法
 * 3. Airmode 支持
 * 
 * 电机布局 (X 型机架，从上方看):
 *        前
 *    M3     M1
 *      \   /
 *       \ /
 *       / \
 *      /   \
 *    M2     M4
 *        后
 * 
 * M1 (右前): CCW (逆时针)
 * M2 (左后): CCW (逆时针)
 * M3 (左前): CW  (顺时针)
 * M4 (右后): CW  (顺时针)
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* 电机数量 */
#define MOTOR_COUNT             4

/* PWM 参数 */
#define MOTOR_PWM_MIN           1000    /**< 最小 PWM 值 (电机停转) */
#define MOTOR_PWM_MAX           2000    /**< 最大 PWM 值 (全油门) */
#define MOTOR_PWM_IDLE          1100    /**< 怠速 PWM 值 */
#define MOTOR_PWM_ARM           1050    /**< 解锁时的最小 PWM */

/* TIM8 ARR = 4999, 对应 2MHz 计数时钟 */
/* PWM 占空比 = CCR / (ARR + 1) */
/* 1000us = 1000 * 2 = 2000 计数 */
/* 2000us = 2000 * 2 = 4000 计数 */
#define MOTOR_CCR_MIN           2000    /**< 最小 CCR 值 */
#define MOTOR_CCR_MAX           4000    /**< 最大 CCR 值 */
#define MOTOR_CCR_IDLE          2200    /**< 怠速 CCR 值 */

/* 混控限制 */
#define MOTOR_MIX_LIMIT         500.0f  /**< 混控输出限幅 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机索引枚举
 */
typedef enum {
    MOTOR_1 = 0,    /**< 右前 (CCW) - PC6 TIM8_CH1 */
    MOTOR_2,        /**< 左后 (CCW) - PC7 TIM8_CH2 */
    MOTOR_3,        /**< 左前 (CW)  - PC8 TIM8_CH3 */
    MOTOR_4         /**< 右后 (CW)  - PC9 TIM8_CH4 */
} Motor_Index_t;

/**
 * @brief 电机状态枚举
 */
typedef enum {
    MOTOR_STATE_DISARMED = 0,   /**< 锁定状态 */
    MOTOR_STATE_ARMED,          /**< 解锁状态 */
    MOTOR_STATE_RUNNING         /**< 运行状态 */
} Motor_State_t;

/**
 * @brief 电机输出结构体
 */
typedef struct {
    uint16_t pwm[MOTOR_COUNT];      /**< PWM 值 (1000-2000) */
    uint16_t ccr[MOTOR_COUNT];      /**< CCR 值 (直接写入寄存器) */
    float throttle;                 /**< 油门输入 (0.0-1.0) */
    float roll;                     /**< 横滚输入 */
    float pitch;                    /**< 俯仰输入 */
    float yaw;                      /**< 偏航输入 */
    Motor_State_t state;            /**< 电机状态 */
    bool airmode_enabled;           /**< Airmode 使能 */
} Motor_Output_t;

/* Exported variables --------------------------------------------------------*/
extern Motor_Output_t g_motor_output;   /**< 全局电机输出 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电机控制
 * @note 启动 TIM8 PWM 输出
 */
void Motor_Init(void);

/**
 * @brief 解锁电机
 * @note 电机开始输出怠速 PWM
 */
void Motor_Arm(void);

/**
 * @brief 锁定电机
 * @note 电机停止输出
 */
void Motor_Disarm(void);

/**
 * @brief 检查电机是否解锁
 * @return bool true=已解锁, false=已锁定
 */
bool Motor_IsArmed(void);

/**
 * @brief 设置单个电机 PWM
 * @param motor 电机索引
 * @param pwm PWM 值 (1000-2000)
 */
void Motor_SetPWM(Motor_Index_t motor, uint16_t pwm);

/**
 * @brief 设置所有电机 PWM
 * @param pwm1 电机 1 PWM
 * @param pwm2 电机 2 PWM
 * @param pwm3 电机 3 PWM
 * @param pwm4 电机 4 PWM
 */
void Motor_SetAllPWM(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);

/**
 * @brief 电机混控计算 (X 型机架)
 * @param throttle 油门 (0.0 - 1.0)
 * @param roll 横滚 PID 输出
 * @param pitch 俯仰 PID 输出
 * @param yaw 偏航 PID 输出
 */
void Motor_Mix(float throttle, float roll, float pitch, float yaw);

/**
 * @brief 应用电机输出
 * @note 将混控结果写入 PWM 寄存器
 */
void Motor_ApplyOutput(void);

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void);

/**
 * @brief 使能/禁用 Airmode
 * @param enable true=使能, false=禁用
 */
void Motor_SetAirmode(bool enable);

/**
 * @brief 获取电机 PWM 值
 * @param motor 电机索引
 * @return uint16_t PWM 值
 */
uint16_t Motor_GetPWM(Motor_Index_t motor);

/**
 * @brief 启动电机测试 (非阻塞)
 * @param motor 电机索引
 * @param duration_ms 持续时间 (ms)
 * @return bool true=启动成功, false=已有测试在进行
 */
bool Motor_TestStart(Motor_Index_t motor, uint32_t duration_ms);

/**
 * @brief 更新电机测试状态机 (非阻塞)
 * @note 需要在主循环或任务中周期性调用
 * @return bool true=测试仍在进行, false=测试已完成或未激活
 */
bool Motor_TestUpdate(void);

/**
 * @brief 检查电机测试是否正在进行
 * @return bool true=测试中, false=空闲
 */
bool Motor_IsTestActive(void);

/**
 * @brief 停止电机测试
 */
void Motor_TestStop(void);

/**
 * @brief 电机测试 (阻塞方式，仅用于非 RTOS 环境)
 * @param motor 电机索引
 * @param duration_ms 持续时间 (ms)
 * @deprecated 在 FreeRTOS 环境中请使用 Motor_TestStart() + Motor_TestUpdate()
 */
void Motor_Test(Motor_Index_t motor, uint32_t duration_ms);

/**
 * @brief 将 PWM 值转换为 CCR 值
 * @param pwm PWM 值 (1000-2000)
 * @return uint16_t CCR 值
 */
uint16_t Motor_PWMtoCCR(uint16_t pwm);

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return uint16_t 限幅后的值
 */
uint16_t Motor_Constrain(int32_t value, uint16_t min, uint16_t max);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
