#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"

// SBUS 协议参数
#define SBUS_FRAME_SIZE 25
#define SBUS_CHANNEL_NUMBER 16

// SBUS 通道值范围
#define SBUS_MIN_VALUE   172
#define SBUS_MID_VALUE   992
#define SBUS_MAX_VALUE   1811

// 通道映射定义 (可根据遥控器配置调整)
#define CH_ROLL     0   // 横滚 (Aileron)
#define CH_PITCH    1   // 俯仰 (Elevator)
#define CH_THROTTLE 2   // 油门 (Throttle)
#define CH_YAW      3   // 偏航 (Rudder)
#define CH_ARM      4   // 解锁开关 (通常是三段开关)
#define CH_MODE     5   // 飞行模式开关

// 解锁/上锁阈值
#define ARM_THRESHOLD_LOW   300   // 油门低位阈值
#define ARM_THRESHOLD_HIGH  1700  // 开关高位阈值

// SBUS 数据结构体
typedef struct {
    uint16_t channels[SBUS_CHANNEL_NUMBER]; // 16个通道值 (范围 172~1811)
    uint8_t failsafe; // 失控保护标志 (0:正常, 1:丢帧, 2:失控)
    uint8_t frame_lost;
} SBUS_Data_t;

// 全局变量声明
extern SBUS_Data_t SBUS_Data;
extern uint8_t SBUS_RxBuffer[SBUS_FRAME_SIZE];

// 函数声明
void SBUS_Init(UART_HandleTypeDef *huart);
void SBUS_Parse(uint8_t *buffer);
uint8_t SBUS_IsConnected(void);

/**
 * @brief 检查解锁手势 (油门最低 + 方向舵最右)
 * @return 1: 解锁手势有效, 0: 无效
 */
uint8_t SBUS_CheckArmGesture(void);

/**
 * @brief 检查上锁手势 (油门最低 + 方向舵最左)
 * @return 1: 上锁手势有效, 0: 无效
 */
uint8_t SBUS_CheckDisarmGesture(void);

/**
 * @brief 将通道值转换为归一化值
 * @param channel 通道索引
 * @param min_out 输出最小值
 * @param max_out 输出最大值
 * @return 归一化后的值
 */
float SBUS_GetNormalized(uint8_t channel, float min_out, float max_out);

/**
 * @brief 获取油门归一化值 (0.0 ~ 1.0)
 */
float SBUS_GetThrottle(void);

/**
 * @brief 获取横滚归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetRoll(void);

/**
 * @brief 获取俯仰归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetPitch(void);

/**
 * @brief 获取偏航归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetYaw(void);

#endif
