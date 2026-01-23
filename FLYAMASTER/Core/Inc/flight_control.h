/**
 * @file flight_control.h
 * @brief 飞行控制核心模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 整合所有飞行控制逻辑：
 * 1. 飞行模式管理
 * 2. 解锁/锁定逻辑
 * 3. 失控保护
 * 4. 控制回路调度
 */

#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* 控制参数 */
#define FC_MAX_ANGLE            45.0f   /**< 最大倾斜角度 (度) */
#define FC_MAX_YAW_RATE         200.0f  /**< 最大偏航角速度 (度/秒) */
#define FC_THROTTLE_DEADBAND    0.05f   /**< 油门死区 */
#define FC_STICK_DEADBAND       0.02f   /**< 摇杆死区 */

/* 解锁条件 */
#define FC_ARM_THROTTLE_MAX     0.05f   /**< 解锁时油门最大值 */
#define FC_ARM_STICK_MAX        0.1f    /**< 解锁时摇杆最大偏移 */

/* 失控保护 */
#define FC_FAILSAFE_THROTTLE    0.3f    /**< 失控保护油门 */
#define FC_FAILSAFE_TIMEOUT_MS  500     /**< 失控保护超时 (ms) */

/* 自动降落 */
#define FC_LANDING_MIN_THROTTLE 0.15f   /**< 降落最小油门 (防止失速) */
#define FC_LANDING_RATE         0.0002f /**< 降落油门递减率 (每控制周期) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 飞行模式枚举
 */
typedef enum {
    FC_MODE_STABILIZE = 0,  /**< 自稳模式 (角度控制) */
    FC_MODE_ACRO,           /**< 特技模式 (角速度控制) */
    FC_MODE_ALTITUDE,       /**< 定高模式 */
    FC_MODE_POSITION,       /**< 定点模式 */
    FC_MODE_RTH,            /**< 返航模式 */
    FC_MODE_LAND            /**< 降落模式 */
} FC_Mode_t;

/**
 * @brief 飞行状态枚举
 */
typedef enum {
    FC_STATE_INIT = 0,      /**< 初始化中 */
    FC_STATE_DISARMED,      /**< 已锁定 */
    FC_STATE_ARMING,        /**< 解锁中 */
    FC_STATE_ARMED,         /**< 已解锁 */
    FC_STATE_FLYING,        /**< 飞行中 */
    FC_STATE_FAILSAFE,      /**< 失控保护 */
    FC_STATE_LANDING,       /**< 降落中 */
    FC_STATE_ERROR          /**< 错误状态 */
} FC_State_t;

/**
 * @brief 解锁检查结果
 */
typedef enum {
    FC_ARM_OK = 0,              /**< 可以解锁 */
    FC_ARM_THROTTLE_HIGH,       /**< 油门过高 */
    FC_ARM_STICK_NOT_CENTER,    /**< 摇杆未归中 */
    FC_ARM_SENSOR_ERROR,        /**< 传感器错误 */
    FC_ARM_RC_LOST,             /**< 遥控器信号丢失 */
    FC_ARM_CALIBRATING,         /**< 正在校准 */
    FC_ARM_ANGLE_TOO_LARGE      /**< 倾斜角度过大 */
} FC_ArmCheck_t;

/**
 * @brief 飞行控制数据结构
 */
typedef struct {
    /* 状态 */
    FC_State_t state;           /**< 当前状态 */
    FC_State_t prev_state;      /**< 进入失控保护前的状态 (用于恢复) */
    FC_Mode_t mode;             /**< 当前模式 */
    
    /* 目标值 */
    float target_roll;          /**< 目标横滚角 (度) */
    float target_pitch;         /**< 目标俯仰角 (度) */
    float target_yaw_rate;      /**< 目标偏航角速度 (度/秒) */
    float target_throttle;      /**< 目标油门 (0.0-1.0) */
    
    /* 当前姿态 */
    float current_roll;         /**< 当前横滚角 (度) */
    float current_pitch;        /**< 当前俯仰角 (度) */
    float current_yaw;          /**< 当前偏航角 (度) */
    
    /* 角速度 */
    float gyro_x;               /**< 陀螺仪 X (度/秒) */
    float gyro_y;               /**< 陀螺仪 Y (度/秒) */
    float gyro_z;               /**< 陀螺仪 Z (度/秒) */
    
    /* 控制输出 */
    float output_roll;          /**< 横滚输出 */
    float output_pitch;         /**< 俯仰输出 */
    float output_yaw;           /**< 偏航输出 */
    
    /* 时间 */
    uint32_t last_update_tick;  /**< 上次更新时间 */
    float dt;                   /**< 控制周期 (秒) */
    
    /* 标志 */
    bool sensors_ready;         /**< 传感器就绪 */
    bool rc_connected;          /**< 遥控器连接 */
    bool failsafe_active;       /**< 失控保护激活 */
    bool calibrating;           /**< 正在校准 */
} FC_Data_t;

/* Exported variables --------------------------------------------------------*/
extern FC_Data_t g_fc_data;     /**< 全局飞行控制数据 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化飞行控制
 */
void FC_Init(void);

/**
 * @brief 飞行控制主循环
 * @note 在 ctl_task 中调用，250Hz
 */
void FC_Update(void);

/**
 * @brief 更新传感器数据
 * @note 从 IMU 和 Mahony 滤波器获取数据
 */
void FC_UpdateSensors(void);

/**
 * @brief 更新遥控器输入
 * @note 从 SBUS 获取数据
 */
void FC_UpdateRC(void);

/**
 * @brief 计算控制输出
 */
void FC_ComputeControl(void);

/**
 * @brief 应用控制输出到电机
 */
void FC_ApplyOutput(void);

/**
 * @brief 尝试解锁
 * @return FC_ArmCheck_t 解锁检查结果
 */
FC_ArmCheck_t FC_TryArm(void);

/**
 * @brief 锁定
 */
void FC_Disarm(void);

/**
 * @brief 检查是否可以解锁
 * @return FC_ArmCheck_t 检查结果
 */
FC_ArmCheck_t FC_CheckArmConditions(void);

/**
 * @brief 设置飞行模式
 * @param mode 目标模式
 */
void FC_SetMode(FC_Mode_t mode);

/**
 * @brief 获取当前飞行模式
 * @return FC_Mode_t 当前模式
 */
FC_Mode_t FC_GetMode(void);

/**
 * @brief 获取当前飞行状态
 * @return FC_State_t 当前状态
 */
FC_State_t FC_GetState(void);

/**
 * @brief 检查是否已解锁
 * @return bool true=已解锁
 */
bool FC_IsArmed(void);

/**
 * @brief 检查是否在飞行中
 * @return bool true=飞行中
 */
bool FC_IsFlying(void);

/**
 * @brief 处理失控保护
 */
void FC_HandleFailsafe(void);

/**
 * @brief 应用摇杆死区
 * @param value 输入值
 * @param deadband 死区大小
 * @return float 处理后的值
 */
float FC_ApplyDeadband(float value, float deadband);

/**
 * @brief 将摇杆值映射到角度
 * @param stick 摇杆值 (-1.0 - 1.0)
 * @param max_angle 最大角度
 * @return float 目标角度
 */
float FC_StickToAngle(float stick, float max_angle);

/**
 * @brief 将摇杆值映射到角速度
 * @param stick 摇杆值 (-1.0 - 1.0)
 * @param max_rate 最大角速度
 * @return float 目标角速度
 */
float FC_StickToRate(float stick, float max_rate);

/**
 * @brief 获取飞行时间 (秒)
 * @return uint32_t 飞行时间
 */
uint32_t FC_GetFlightTime(void);

/**
 * @brief 重置飞行控制
 */
void FC_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_CONTROL_H */