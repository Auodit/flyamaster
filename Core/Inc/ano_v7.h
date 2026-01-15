/**
 * @file    ano_v7.h
 * @brief   匿名上位机 V7 通信协议头文件
 * @author  Q版 Doro (飞控霸主)
 * @date    2026-01-14
 * @version 1.0.0
 * 
 * @note    本协议用于与匿名地面站 V7 进行数据通信
 *          支持姿态/传感器/电机/状态数据上传，以及 PID 参数读写
 */

#ifndef __ANO_V7_H
#define __ANO_V7_H

#include "main.h"
#include <stdint.h>

/* ======================== 协议帧格式常量 ======================== */

#define ANO_V7_FRAME_HEADER1    0xAA    // 帧头1
#define ANO_V7_FRAME_HEADER2    0xAF    // 帧头2

/* 功能字定义 */
#define ANO_V7_FUNC_ATTITUDE    0x01    // 姿态数据帧 (FC→PC)
#define ANO_V7_FUNC_SENSOR      0x02    // 传感器数据帧 (FC→PC)
#define ANO_V7_FUNC_MOTOR       0x04    // 电机PWM帧 (FC→PC)
#define ANO_V7_FUNC_STATUS      0x05    // 状态数据帧 (FC→PC)
#define ANO_V7_FUNC_PID_READ    0x10    // 读取PID (PC→FC)
#define ANO_V7_FUNC_PID_WRITE   0x11    // 写入PID (PC→FC)
#define ANO_V7_FUNC_PID_DATA    0x12    // PID数据帧 (FC→PC)

/* 缓冲区大小 */
#define ANO_V7_TX_BUF_SIZE      256     // 发送缓冲区大小
#define ANO_V7_RX_BUF_SIZE      256     // 接收缓冲区大小

/* ======================== 数据结构定义 ======================== */

/**
 * @brief 姿态数据帧 (0x01) - 18字节
 * @note  发送频率: 20Hz
 */
typedef struct {
    int16_t roll;      // 横滚角 ×100 (度)
    int16_t pitch;     // 俯仰角 ×100 (度)
    int16_t yaw;       // 航向角 ×100 (度)
    int32_t alt;       // 高度 ×100 (cm)
    int16_t vx;        // X速度 ×100 (m/s)
    int16_t vy;        // Y速度 ×100 (m/s)
    int16_t vz;        // Z速度 ×100 (m/s)
} __attribute__((packed)) AnoV7_Attitude_t;

/**
 * @brief 传感器数据帧 (0x02) - 18字节
 * @note  发送频率: 10Hz
 */
typedef struct {
    int16_t acc_x;     // 加速度 X ×100 (g)
    int16_t acc_y;     // 加速度 Y ×100 (g)
    int16_t acc_z;     // 加速度 Z ×100 (g)
    int16_t gyro_x;    // 角速度 X ×100 (deg/s)
    int16_t gyro_y;    // 角速度 Y ×100 (deg/s)
    int16_t gyro_z;    // 角速度 Z ×100 (deg/s)
    int16_t mag_x;     // 磁力计 X (未使用)
    int16_t mag_y;     // 磁力计 Y (未使用)
    int16_t mag_z;     // 磁力计 Z (未使用)
} __attribute__((packed)) AnoV7_Sensor_t;

/**
 * @brief 电机PWM帧 (0x04) - 16字节
 * @note  发送频率: 10Hz
 */
typedef struct {
    uint16_t motor[8]; // 8路电机PWM (1000~2000)
} __attribute__((packed)) AnoV7_Motor_t;

/**
 * @brief 状态数据帧 (0x05) - 6字节
 * @note  发送频率: 5Hz
 */
typedef struct {
    uint16_t voltage;  // 电池电压 ×100 (V)
    uint8_t  locked;   // 锁定状态 (0=上锁 1=解锁)
    uint8_t  mode;     // 飞行模式 (0=手动 1=定高 2=定点)
    uint16_t error;    // 错误码
} __attribute__((packed)) AnoV7_Status_t;

/**
 * @brief PID参数帧 (0x10/0x11/0x12) - 13字节
 */
typedef struct {
    uint8_t pid_id;    // PID组ID (0~9)
    float   kp;        // P参数
    float   ki;        // I参数
    float   kd;        // D参数
} __attribute__((packed)) AnoV7_PID_t;

/* ======================== 回调函数类型定义 ======================== */

/**
 * @brief PID参数写入回调函数类型
 * @param pid_data PID参数结构体指针
 */
typedef void (*AnoV7_PID_Callback)(AnoV7_PID_t *pid_data);

/* ======================== 函数接口声明 ======================== */

/**
 * @brief 初始化匿名上位机V7协议
 * @param huart UART句柄指针 (使用 &huart5 - UART5, PC12/PD2, 115200 baud)
 */
void AnoV7_Init(UART_HandleTypeDef *huart);

/**
 * @brief 发送姿态数据帧
 * @note  建议调用频率: 20Hz (50ms)
 */
void AnoV7_SendAttitude(void);

/**
 * @brief 发送传感器数据帧
 * @note  建议调用频率: 10Hz (100ms)
 */
void AnoV7_SendSensor(void);

/**
 * @brief 发送电机PWM帧
 * @note  建议调用频率: 10Hz (100ms)
 */
void AnoV7_SendMotor(void);

/**
 * @brief 发送状态数据帧
 * @note  建议调用频率: 5Hz (200ms)
 */
void AnoV7_SendStatus(void);

/**
 * @brief 发送PID数据帧
 * @param pid_id PID组ID (0~9)
 */
void AnoV7_SendPID(uint8_t pid_id);

/**
 * @brief 解析接收到的数据
 * @param buf 接收缓冲区指针
 * @param len 数据长度
 */
void AnoV7_Parse(uint8_t *buf, uint16_t len);

/**
 * @brief 注册PID参数写入回调函数
 * @param callback 回调函数指针
 */
void AnoV7_RegisterPIDCallback(AnoV7_PID_Callback callback);

#endif /* __ANO_V7_H */
