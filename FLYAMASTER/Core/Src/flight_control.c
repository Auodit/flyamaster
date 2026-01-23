/**
 * @file flight_control.c
 * @brief 飞行控制核心实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "flight_control.h"
#include "main.h"
#include "mpu6050.h"
#include "mahony.h"
#include "sbus.h"
#include "pid.h"
#include "motor.h"
#include <math.h>

/* Exported variables --------------------------------------------------------*/
FC_Data_t g_fc_data = {
    .state = FC_STATE_INIT,
    .prev_state = FC_STATE_DISARMED,
    .mode = FC_MODE_STABILIZE,
    .target_roll = 0.0f,
    .target_pitch = 0.0f,
    .target_yaw_rate = 0.0f,
    .target_throttle = 0.0f,
    .current_roll = 0.0f,
    .current_pitch = 0.0f,
    .current_yaw = 0.0f,
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .output_roll = 0.0f,
    .output_pitch = 0.0f,
    .output_yaw = 0.0f,
    .last_update_tick = 0,
    .dt = 0.004f,  /* 250Hz = 4ms */
    .sensors_ready = false,
    .rc_connected = false,
    .failsafe_active = false,
    .calibrating = false
};

/* Private variables ---------------------------------------------------------*/
static uint32_t arm_start_tick = 0;
static uint32_t flight_start_tick = 0;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化飞行控制
 */
void FC_Init(void)
{
    /* 初始化 PID 控制器 */
    FlightPID_Init();
    
    /* 初始化电机 */
    Motor_Init();
    
    /* 设置初始状态 */
    g_fc_data.state = FC_STATE_DISARMED;
    g_fc_data.mode = FC_MODE_STABILIZE;
    g_fc_data.sensors_ready = false;
    g_fc_data.rc_connected = false;
    g_fc_data.failsafe_active = false;
    
    g_fc_data.last_update_tick = HAL_GetTick();
}

/**
 * @brief 飞行控制主循环
 */
void FC_Update(void)
{
    uint32_t current_tick = HAL_GetTick();
    
    /* 计算时间间隔 */
    g_fc_data.dt = (float)(current_tick - g_fc_data.last_update_tick) / 1000.0f;
    if (g_fc_data.dt < 0.001f) g_fc_data.dt = 0.001f;
    if (g_fc_data.dt > 0.1f) g_fc_data.dt = 0.1f;
    g_fc_data.last_update_tick = current_tick;
    
    /* 更新传感器数据 */
    FC_UpdateSensors();
    
    /* 更新遥控器输入 */
    FC_UpdateRC();
    
    /* 处理失控保护 */
    FC_HandleFailsafe();
    
    /* 根据状态执行控制 */
    switch (g_fc_data.state) {
        case FC_STATE_DISARMED:
            /* 锁定状态，电机停止 */
            Motor_Disarm();
            break;
            
        case FC_STATE_ARMING:
            /* 解锁中，等待确认 */
            if ((current_tick - arm_start_tick) > 500) {
                g_fc_data.state = FC_STATE_ARMED;
                Motor_Arm();
            }
            break;
            
        case FC_STATE_ARMED:
        case FC_STATE_FLYING:
            /* 安全检查：传感器必须有效 */
            if (!g_fc_data.sensors_ready) {
                /* 传感器故障，进入失控保护 */
                g_fc_data.failsafe_active = true;
                g_fc_data.state = FC_STATE_FAILSAFE;
                break;
            }
            
            /* 计算控制输出 */
            FC_ComputeControl();
            /* 应用到电机 */
            FC_ApplyOutput();
            
            /* 检测是否在飞行 */
            if (g_fc_data.target_throttle > 0.1f) {
                if (g_fc_data.state == FC_STATE_ARMED) {
                    g_fc_data.state = FC_STATE_FLYING;
                    flight_start_tick = current_tick;
                }
            }
            break;
            
        case FC_STATE_FAILSAFE:
            /* 失控保护，缓慢降落 */
            g_fc_data.target_throttle = FC_FAILSAFE_THROTTLE;
            g_fc_data.target_roll = 0.0f;
            g_fc_data.target_pitch = 0.0f;
            g_fc_data.target_yaw_rate = 0.0f;
            FC_ComputeControl();
            FC_ApplyOutput();
            break;
            
        case FC_STATE_LANDING:
            /* 自动降落 - 缓慢递减油门 */
            /* 0.0002f @ 250Hz = 0.05/s，从满油门到 0 需要 20 秒 */
            g_fc_data.target_throttle -= 0.0002f;
            
            /* 保持最小油门以防止失速 */
            if (g_fc_data.target_throttle < FC_LANDING_MIN_THROTTLE) {
                g_fc_data.target_throttle = FC_LANDING_MIN_THROTTLE;
                /* 检测是否已着陆 (可通过加速度计检测) */
                /* TODO: 添加着陆检测逻辑 */
            }
            
            /* 完全停止条件：油门极低且持续一段时间 */
            if (g_fc_data.target_throttle <= 0.05f) {
                static uint32_t landing_timer = 0;
                if (landing_timer == 0) {
                    landing_timer = HAL_GetTick();
                } else if ((HAL_GetTick() - landing_timer) > 3000) {
                    /* 3 秒后锁定 */
                    g_fc_data.target_throttle = 0.0f;
                    FC_Disarm();
                    landing_timer = 0;
                }
            }
            
            FC_ComputeControl();
            FC_ApplyOutput();
            break;
            
        default:
            Motor_Disarm();
            break;
    }
}

/**
 * @brief 更新传感器数据
 */
void FC_UpdateSensors(void)
{
    /* 从 Mahony 滤波器获取姿态 */
    g_fc_data.current_roll = g_mahony.euler.roll;
    g_fc_data.current_pitch = g_mahony.euler.pitch;
    g_fc_data.current_yaw = g_mahony.euler.yaw;
    
    /* 从 MPU6050 获取角速度 (°/s) */
    g_fc_data.gyro_x = g_mpu6050_data.gyro_x;
    g_fc_data.gyro_y = g_mpu6050_data.gyro_y;
    g_fc_data.gyro_z = g_mpu6050_data.gyro_z;
    
    /* 检查传感器状态 (使用 data_ready 标志) */
    g_fc_data.sensors_ready = g_mpu6050_data_ready && !g_fc_data.calibrating;
}

/**
 * @brief 更新遥控器输入
 */
void FC_UpdateRC(void)
{
    /* 检查遥控器连接 */
    g_fc_data.rc_connected = SBUS_IsConnected();
    
    if (!g_fc_data.rc_connected) {
        return;
    }
    
    /* 获取油门 (已归一化 0.0-1.0) */
    g_fc_data.target_throttle = g_rc_data.throttle;
    
    /* 应用死区并映射到角度 */
    float roll_stick = FC_ApplyDeadband(g_rc_data.roll, FC_STICK_DEADBAND);
    float pitch_stick = FC_ApplyDeadband(g_rc_data.pitch, FC_STICK_DEADBAND);
    float yaw_stick = FC_ApplyDeadband(g_rc_data.yaw, FC_STICK_DEADBAND);
    
    /* 根据模式计算目标值 */
    switch (g_fc_data.mode) {
        case FC_MODE_STABILIZE:
            /* 自稳模式：摇杆控制角度 */
            g_fc_data.target_roll = FC_StickToAngle(roll_stick, FC_MAX_ANGLE);
            g_fc_data.target_pitch = FC_StickToAngle(pitch_stick, FC_MAX_ANGLE);
            g_fc_data.target_yaw_rate = FC_StickToRate(yaw_stick, FC_MAX_YAW_RATE);
            break;
            
        case FC_MODE_ACRO:
            /* 特技模式：摇杆控制角速度 */
            g_fc_data.target_roll = FC_StickToRate(roll_stick, FC_MAX_YAW_RATE);
            g_fc_data.target_pitch = FC_StickToRate(pitch_stick, FC_MAX_YAW_RATE);
            g_fc_data.target_yaw_rate = FC_StickToRate(yaw_stick, FC_MAX_YAW_RATE);
            break;
            
        default:
            g_fc_data.target_roll = FC_StickToAngle(roll_stick, FC_MAX_ANGLE);
            g_fc_data.target_pitch = FC_StickToAngle(pitch_stick, FC_MAX_ANGLE);
            g_fc_data.target_yaw_rate = FC_StickToRate(yaw_stick, FC_MAX_YAW_RATE);
            break;
    }
    
    /* 处理解锁/锁定开关 */
    if (g_rc_data.arm_switch == 1 && g_fc_data.state == FC_STATE_DISARMED) {
        FC_TryArm();
    } else if (g_rc_data.arm_switch == 0 && g_fc_data.state != FC_STATE_DISARMED) {
        FC_Disarm();
    }
    
    /* 处理模式开关 */
    switch (g_rc_data.mode_switch) {
        case 0:
            FC_SetMode(FC_MODE_STABILIZE);
            break;
        case 1:
            FC_SetMode(FC_MODE_STABILIZE);  /* 半自稳暂时用自稳 */
            break;
        case 2:
            FC_SetMode(FC_MODE_ACRO);
            break;
    }
}

/**
 * @brief 计算控制输出
 */
void FC_ComputeControl(void)
{
    PID_Output_t pid_output;
    
    if (g_fc_data.mode == FC_MODE_ACRO) {
        /* 特技模式：只用角速度环 */
        pid_output.roll = PID_Compute(&g_flight_pid.roll_rate,
                                       g_fc_data.target_roll,
                                       g_fc_data.gyro_x,
                                       g_fc_data.dt);
        pid_output.pitch = PID_Compute(&g_flight_pid.pitch_rate,
                                        g_fc_data.target_pitch,
                                        g_fc_data.gyro_y,
                                        g_fc_data.dt);
        pid_output.yaw = PID_Compute(&g_flight_pid.yaw_rate,
                                      g_fc_data.target_yaw_rate,
                                      g_fc_data.gyro_z,
                                      g_fc_data.dt);
    } else {
        /* 自稳模式：串级 PID */
        FlightPID_Compute(g_fc_data.target_roll,
                          g_fc_data.target_pitch,
                          g_fc_data.target_yaw_rate,
                          g_fc_data.current_roll,
                          g_fc_data.current_pitch,
                          g_fc_data.gyro_x,
                          g_fc_data.gyro_y,
                          g_fc_data.gyro_z,
                          g_fc_data.dt,
                          &pid_output);
    }
    
    g_fc_data.output_roll = pid_output.roll;
    g_fc_data.output_pitch = pid_output.pitch;
    g_fc_data.output_yaw = pid_output.yaw;
}

/**
 * @brief 应用控制输出到电机
 */
void FC_ApplyOutput(void)
{
    /* 电机混控 */
    Motor_Mix(g_fc_data.target_throttle,
              g_fc_data.output_roll,
              g_fc_data.output_pitch,
              g_fc_data.output_yaw);
    
    /* 应用输出 */
    Motor_ApplyOutput();
}

/**
 * @brief 尝试解锁
 */
FC_ArmCheck_t FC_TryArm(void)
{
    FC_ArmCheck_t check = FC_CheckArmConditions();
    
    if (check == FC_ARM_OK) {
        g_fc_data.state = FC_STATE_ARMING;
        arm_start_tick = HAL_GetTick();
        FlightPID_Reset();
    }
    
    return check;
}

/**
 * @brief 锁定
 */
void FC_Disarm(void)
{
    g_fc_data.state = FC_STATE_DISARMED;
    Motor_Disarm();
    FlightPID_Reset();
}

/**
 * @brief 检查解锁条件
 */
FC_ArmCheck_t FC_CheckArmConditions(void)
{
    /* 检查传感器 */
    if (!g_fc_data.sensors_ready) {
        return FC_ARM_SENSOR_ERROR;
    }
    
    /* 检查遥控器 */
    if (!g_fc_data.rc_connected) {
        return FC_ARM_RC_LOST;
    }
    
    /* 检查校准状态 */
    if (g_fc_data.calibrating) {
        return FC_ARM_CALIBRATING;
    }
    
    /* 检查油门 */
    if (g_fc_data.target_throttle > FC_ARM_THROTTLE_MAX) {
        return FC_ARM_THROTTLE_HIGH;
    }
    
    /* 检查摇杆 */
    if (fabsf(g_rc_data.roll) > FC_ARM_STICK_MAX ||
        fabsf(g_rc_data.pitch) > FC_ARM_STICK_MAX) {
        return FC_ARM_STICK_NOT_CENTER;
    }
    
    /* 检查倾斜角度 */
    if (fabsf(g_fc_data.current_roll) > 10.0f ||
        fabsf(g_fc_data.current_pitch) > 10.0f) {
        return FC_ARM_ANGLE_TOO_LARGE;
    }
    
    return FC_ARM_OK;
}

/**
 * @brief 设置飞行模式
 */
void FC_SetMode(FC_Mode_t mode)
{
    if (g_fc_data.mode != mode) {
        g_fc_data.mode = mode;
        /* 切换模式时重置 PID */
        FlightPID_Reset();
    }
}

/**
 * @brief 获取当前飞行模式
 */
FC_Mode_t FC_GetMode(void)
{
    return g_fc_data.mode;
}

/**
 * @brief 获取当前飞行状态
 */
FC_State_t FC_GetState(void)
{
    return g_fc_data.state;
}

/**
 * @brief 检查是否已解锁
 */
bool FC_IsArmed(void)
{
    return (g_fc_data.state != FC_STATE_DISARMED && 
            g_fc_data.state != FC_STATE_INIT);
}

/**
 * @brief 检查是否在飞行中
 */
bool FC_IsFlying(void)
{
    return (g_fc_data.state == FC_STATE_FLYING);
}

/**
 * @brief 处理失控保护
 * @note 进入失控保护时保存当前状态，恢复时还原到之前的状态
 *       避免从 FLYING 状态直接跳变到 ARMED 状态
 */
void FC_HandleFailsafe(void)
{
    if (!g_fc_data.rc_connected && FC_IsArmed()) {
        /* 进入失控保护：保存当前状态 */
        if (!g_fc_data.failsafe_active) {
            g_fc_data.prev_state = g_fc_data.state;
        }
        g_fc_data.failsafe_active = true;
        g_fc_data.state = FC_STATE_FAILSAFE;
    } else if (g_fc_data.rc_connected && g_fc_data.failsafe_active) {
        /* 恢复控制：还原到之前的状态 */
        g_fc_data.failsafe_active = false;
        if (g_fc_data.state == FC_STATE_FAILSAFE) {
            /* 恢复到进入失控保护前的状态 */
            g_fc_data.state = g_fc_data.prev_state;
        }
    }
}

/**
 * @brief 应用摇杆死区
 */
float FC_ApplyDeadband(float value, float deadband)
{
    if (fabsf(value) < deadband) {
        return 0.0f;
    }
    
    /* 重新映射，使死区外的值从 0 开始 */
    if (value > 0) {
        return (value - deadband) / (1.0f - deadband);
    } else {
        return (value + deadband) / (1.0f - deadband);
    }
}

/**
 * @brief 将摇杆值映射到角度
 */
float FC_StickToAngle(float stick, float max_angle)
{
    return stick * max_angle;
}

/**
 * @brief 将摇杆值映射到角速度
 */
float FC_StickToRate(float stick, float max_rate)
{
    return stick * max_rate;
}

/**
 * @brief 获取飞行时间
 */
uint32_t FC_GetFlightTime(void)
{
    if (g_fc_data.state == FC_STATE_FLYING) {
        return (HAL_GetTick() - flight_start_tick) / 1000;
    }
    return 0;
}

/**
 * @brief 重置飞行控制
 */
void FC_Reset(void)
{
    FC_Disarm();
    FlightPID_Reset();
    g_fc_data.state = FC_STATE_DISARMED;
    g_fc_data.mode = FC_MODE_STABILIZE;
    g_fc_data.failsafe_active = false;
}