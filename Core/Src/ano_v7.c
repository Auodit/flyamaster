/**
 * @file    ano_v7.c
 * @brief   匿名上位机 V7 通信协议实现文件
 * @author  Q版 Doro (飞控霸主)
 * @date    2026-01-14
 * @version 1.0.0
 */

#include "ano_v7.h"
#include "mpu6050.h"
#include "altitude.h"
#include "flash_params.h"
#include "string.h"

/* ======================== 私有变量 ======================== */

static UART_HandleTypeDef *g_ano_uart = NULL;  // UART句柄
static uint8_t tx_buffer[ANO_V7_TX_BUF_SIZE];  // 发送缓冲区
static uint8_t rx_buffer[ANO_V7_RX_BUF_SIZE];  // 接收缓冲区

static AnoV7_PID_Callback g_ano_pid_callback = NULL;  // 匿名协议 PID 回调函数

/* ======================== 外部变量引用 ======================== */

// 从 freertos.c 引用全局变量
extern volatile float g_roll, g_pitch, g_yaw;
extern volatile float g_gyro_x, g_gyro_y, g_gyro_z;
extern volatile float g_battery_voltage;
extern volatile uint8_t g_flight_state;   // FlightState_t
extern volatile uint8_t g_flight_mode;    // FlightMode_t

// 从 altitude.c 引用高度数据结构体
extern Altitude_TypeDef Alt_Data;

// 从 optical_flow.c 引用光流速度
extern volatile float g_flow_vx, g_flow_vy;

// 从 mpu6050.c 引用加速度和陀螺仪数据 (结构体定义已在 mpu6050.h 中)
extern volatile MPU6050_t MPU6050_Data;

// 从 motor.c 引用电机PWM (需要在 motor.h 中添加外部声明)
extern uint16_t g_motor_pwm[4];

/* ======================== 私有函数声明 ======================== */

static uint8_t AnoV7_CalcSum8(uint8_t *buf, uint16_t len);
static void AnoV7_SendFrame(uint8_t func, uint8_t *data, uint8_t len);

/* ======================== 公共函数实现 ======================== */

/**
 * @brief 初始化匿名上位机V7协议
 * @param huart UART句柄指针
 */
void AnoV7_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;
    
    g_ano_uart = huart;
    
    // 启动 UART DMA 循环接收
    HAL_UART_Receive_DMA(g_ano_uart, rx_buffer, ANO_V7_RX_BUF_SIZE);
}

/**
 * @brief 发送姿态数据帧 (0x01)
 * @note  建议调用频率: 20Hz (50ms)
 */
void AnoV7_SendAttitude(void)
{
    if (g_ano_uart == NULL) return;
    
    AnoV7_Attitude_t att;
    
    // 填充姿态数据 (角度单位: 度, ×100)
    att.roll  = (int16_t)(g_roll * 100.0f);
    att.pitch = (int16_t)(g_pitch * 100.0f);
    att.yaw   = (int16_t)(g_yaw * 100.0f);
    
    // 填充高度数据 (单位: cm, ×100)
    att.alt = (int32_t)(Alt_Data.height * 10000.0f);  // m → cm ×100
    
    // 填充速度数据 (单位: m/s, ×100)
    att.vx = (int16_t)(g_flow_vx * 100.0f);
    att.vy = (int16_t)(g_flow_vy * 100.0f);
    att.vz = (int16_t)(Alt_Data.velocity * 100.0f);
    
    // 发送数据帧
    AnoV7_SendFrame(ANO_V7_FUNC_ATTITUDE, (uint8_t*)&att, sizeof(att));
}

/**
 * @brief 发送传感器数据帧 (0x02)
 * @note  建议调用频率: 10Hz (100ms)
 */
void AnoV7_SendSensor(void)
{
    if (g_ano_uart == NULL) return;
    
    AnoV7_Sensor_t sensor;
    
    // 填充加速度数据 (单位: g, ×100)
    sensor.acc_x = (int16_t)(MPU6050_Data.Ax * 100.0f);
    sensor.acc_y = (int16_t)(MPU6050_Data.Ay * 100.0f);
    sensor.acc_z = (int16_t)(MPU6050_Data.Az * 100.0f);
    
    // 填充角速度数据 (单位: rad/s → deg/s, 然后 ×100)
    // Info #051 修复：注释更准确描述两步转换过程
    sensor.gyro_x = (int16_t)(MPU6050_Data.Gx * 57.2958f * 100.0f);
    sensor.gyro_y = (int16_t)(MPU6050_Data.Gy * 57.2958f * 100.0f);
    sensor.gyro_z = (int16_t)(MPU6050_Data.Gz * 57.2958f * 100.0f);
    
    // 磁力计数据 (暂不支持)
    sensor.mag_x = 0;
    sensor.mag_y = 0;
    sensor.mag_z = 0;
    
    // 发送数据帧
    AnoV7_SendFrame(ANO_V7_FUNC_SENSOR, (uint8_t*)&sensor, sizeof(sensor));
}

/**
 * @brief 发送电机PWM帧 (0x04)
 * @note  建议调用频率: 10Hz (100ms)
 */
void AnoV7_SendMotor(void)
{
    if (g_ano_uart == NULL) return;
    
    AnoV7_Motor_t motor;
    
    // 填充电机PWM数据 (4个电机 + 4个备用)
    motor.motor[0] = g_motor_pwm[0];  // M1
    motor.motor[1] = g_motor_pwm[1];  // M2
    motor.motor[2] = g_motor_pwm[2];  // M3
    motor.motor[3] = g_motor_pwm[3];  // M4
    motor.motor[4] = 0;  // 备用
    motor.motor[5] = 0;
    motor.motor[6] = 0;
    motor.motor[7] = 0;
    
    // 发送数据帧
    AnoV7_SendFrame(ANO_V7_FUNC_MOTOR, (uint8_t*)&motor, sizeof(motor));
}

/**
 * @brief 发送状态数据帧 (0x05)
 * @note  建议调用频率: 5Hz (200ms)
 */
void AnoV7_SendStatus(void)
{
    if (g_ano_uart == NULL) return;
    
    AnoV7_Status_t status;
    
    // 填充状态数据
    status.voltage = (uint16_t)(g_battery_voltage * 100.0f);  // 电压 ×100
    status.locked  = (g_flight_state == 1) ? 1 : 0;  // 1=解锁 0=上锁
    status.mode    = (uint8_t)g_flight_mode;         // 飞行模式
    status.error   = 0;  // 错误码 (暂未实现)
    
    // 发送数据帧
    AnoV7_SendFrame(ANO_V7_FUNC_STATUS, (uint8_t*)&status, sizeof(status));
}

/**
 * @brief 发送PID数据帧 (0x12)
 * @param pid_id PID组ID (0~9)
 */
void AnoV7_SendPID(uint8_t pid_id)
{
    if (g_ano_uart == NULL) return;
    if (pid_id > 9) return;
    
    AnoV7_PID_t pid;
    
    // 从 Flash 参数中读取 PID 值
    PID_Params_t *p_params = FlashParams_GetPID((PID_Index_t)pid_id);
    
    pid.pid_id = pid_id;
    
    if (p_params != NULL) {
        pid.kp = p_params->kp;
        pid.ki = p_params->ki;
        pid.kd = p_params->kd;
    } else {
        pid.kp = 0.0f;
        pid.ki = 0.0f;
        pid.kd = 0.0f;
    }
    
    // 发送数据帧
    AnoV7_SendFrame(ANO_V7_FUNC_PID_DATA, (uint8_t*)&pid, sizeof(pid));
}

/**
 * @brief 解析接收到的数据
 * @param buf 接收缓冲区指针
 * @param len 数据长度
 */
void AnoV7_Parse(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < 6) return;  // 最小帧长度: 2字节头 + 1功能字 + 1长度 + 0数据 + 1校验 + 1尾
    
    // 查找帧头
    for (uint16_t i = 0; i < len - 5; i++)
    {
        if (buf[i] == ANO_V7_FRAME_HEADER1 && buf[i+1] == ANO_V7_FRAME_HEADER2)
        {
            uint8_t func = buf[i+2];
            uint8_t data_len = buf[i+3];
            
            // 检查长度是否合法
            if (i + 4 + data_len + 1 > len) break;
            
            // 校验和验证
            uint8_t sum = AnoV7_CalcSum8(&buf[i+2], data_len + 2);
            if (sum != buf[i + 4 + data_len]) continue;
            
            // 根据功能字处理
            switch (func)
            {
                case ANO_V7_FUNC_PID_READ:
                {
                    // 读取PID请求
                    if (data_len >= 1)
                    {
                        uint8_t pid_id = buf[i+4];
                        AnoV7_SendPID(pid_id);
                    }
                    break;
                }
                
                case ANO_V7_FUNC_PID_WRITE:
                {
                    // 写入PID请求
                    if (data_len >= sizeof(AnoV7_PID_t) && g_ano_pid_callback != NULL)
                    {
                        AnoV7_PID_t *pid = (AnoV7_PID_t*)&buf[i+4];
                        g_ano_pid_callback(pid);
                    }
                    break;
                }
                
                default:
                    break;
            }
        }
    }
}

/**
 * @brief 注册PID参数写入回调函数
 * @param callback 回调函数指针
 */
void AnoV7_RegisterPIDCallback(AnoV7_PID_Callback callback)
{
    g_ano_pid_callback = callback;
}

/* ======================== 私有函数实现 ======================== */

/**
 * @brief 计算 SUM8 校验和
 * @param buf 数据缓冲区
 * @param len 数据长度
 * @return 校验和 (低8位)
 */
static uint8_t AnoV7_CalcSum8(uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return sum;
}

/**
 * @brief 发送数据帧
 * @param func 功能字
 * @param data 数据指针
 * @param len 数据长度
 */
static void AnoV7_SendFrame(uint8_t func, uint8_t *data, uint8_t len)
{
    if (g_ano_uart == NULL) return;
    
    // 检查 UART 是否空闲（防止 DMA 发送冲突）
    if (g_ano_uart->gState != HAL_UART_STATE_READY)
    {
        // UART 正在发送，直接返回避免数据丢失
        // 注意：这会导致当前帧被丢弃，但不会阻塞任务
        return;
    }
    
    uint16_t idx = 0;
    
    // 帧头
    tx_buffer[idx++] = ANO_V7_FRAME_HEADER1;
    tx_buffer[idx++] = ANO_V7_FRAME_HEADER2;
    
    // 功能字
    tx_buffer[idx++] = func;
    
    // 数据长度
    tx_buffer[idx++] = len;
    
    // 数据
    if (data != NULL && len > 0)
    {
        memcpy(&tx_buffer[idx], data, len);
        idx += len;
    }
    
    // 校验和 (功能字 + 长度 + 数据)
    tx_buffer[idx] = AnoV7_CalcSum8(&tx_buffer[2], len + 2);
    idx++;
    
    // DMA 非阻塞发送（带返回值检查）
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(g_ano_uart, tx_buffer, idx);
    
    // 如果发送失败，静默处理（避免任务阻塞）
    // 可选：在这里添加错误计数或日志记录
    (void)status;  // 防止编译器警告未使用变量
}
