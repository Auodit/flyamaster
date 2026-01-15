#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

// 简单的帧头定义
#define FRAME_HEADER 0xAA
#define FRAME_TAIL   0x55

// 功能字
#define CMD_STATUS       0x01  // 发送状态 (姿态, 电压)
#define CMD_CONTROL      0x02  // 接收控制 (摇杆)
#define CMD_PID_SET      0x10  // 设置 PID 参数
#define CMD_PID_GET      0x11  // 获取 PID 参数
#define CMD_MOTOR_TEST   0x20  // 电机测试
#define CMD_CALIBRATE    0x30  // 校准指令
#define CMD_ARM          0x40  // 解锁指令
#define CMD_DISARM       0x41  // 上锁指令

// 发送状态包
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float voltage;
} Status_Packet_t;

// PID 参数包
typedef struct {
    uint8_t pid_id;  // 0=roll_angle, 1=roll_rate, 2=pitch_angle, 3=pitch_rate, 4=yaw_rate
    float kp;
    float ki;
    float kd;
} PID_Packet_t;

// 回调函数指针类型
typedef void (*Protocol_PID_Callback)(PID_Packet_t *pid);
typedef void (*Protocol_Arm_Callback)(uint8_t arm);

// 回调注册
extern Protocol_PID_Callback g_pid_callback;
extern Protocol_Arm_Callback g_arm_callback;

void Protocol_SendStatus(UART_HandleTypeDef *huart, Status_Packet_t *status);
void Protocol_Parse(uint8_t *buf, uint16_t len);
void Protocol_RegisterPIDCallback(Protocol_PID_Callback cb);
void Protocol_RegisterArmCallback(Protocol_Arm_Callback cb);

#endif
