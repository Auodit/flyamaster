/**
 * @file ano_v7.c
 * @brief 匿名上位机 V7 协议实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "ano_v7.h"
#include "usart.h"
#include "ring_buffer.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define ANO_TX_BUFFER_SIZE      128

/* Private variables ---------------------------------------------------------*/
static uint8_t ano_tx_buffer[ANO_TX_BUFFER_SIZE];
static ANO_Frame_t ano_rx_frame;
static uint8_t ano_rx_state = 0;
static uint8_t ano_rx_index = 0;
static uint8_t ano_rx_len = 0;

/* 全局错误码 */
uint16_t g_ano_error_code = ANO_ERROR_NONE;

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart5;

/* Private function prototypes -----------------------------------------------*/
static void ANO_TransmitData(const uint8_t *data, uint16_t len);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 ANO 协议
 */
void ANO_Init(void)
{
    g_ano_error_code = ANO_ERROR_NONE;
    ano_rx_state = 0;
    ano_rx_index = 0;
}

/**
 * @brief 发送帧
 */
void ANO_SendFrame(uint8_t func, const uint8_t *data, uint8_t len)
{
    if (len > ANO_MAX_FRAME_LEN) {
        return;
    }
    
    uint8_t idx = 0;
    uint8_t sum = 0;
    
    /* 帧头 */
    ano_tx_buffer[idx++] = ANO_FRAME_HEAD;
    ano_tx_buffer[idx++] = ANO_FRAME_ADDR;
    ano_tx_buffer[idx++] = func;
    ano_tx_buffer[idx++] = len;
    
    /* 数据 */
    if (data != NULL && len > 0) {
        memcpy(&ano_tx_buffer[idx], data, len);
        idx += len;
    }
    
    /* 计算校验和 (从 ADDR 开始) */
    for (uint8_t i = 1; i < idx; i++) {
        sum += ano_tx_buffer[i];
    }
    ano_tx_buffer[idx++] = sum;
    
    /* 发送 */
    ANO_TransmitData(ano_tx_buffer, idx);
}

/**
 * @brief 发送状态数据 (0x01)
 */
void ANO_SendStatus(void)
{
    /* 需要外部提供数据，这里只是框架 */
    uint8_t data[10];
    
    /* 占位数据 */
    memset(data, 0, sizeof(data));
    
    ANO_SendFrame(ANO_FC_STATUS, data, 10);
}

/**
 * @brief 发送传感器数据 (0x02)
 */
void ANO_SendSensor(void)
{
    uint8_t data[18];
    
    /* 占位数据 */
    memset(data, 0, sizeof(data));
    
    ANO_SendFrame(ANO_FC_SENSOR, data, 18);
}

/**
 * @brief 发送遥控器数据 (0x03)
 */
void ANO_SendRC(void)
{
    uint8_t data[20];
    
    /* 占位数据 */
    memset(data, 0, sizeof(data));
    
    ANO_SendFrame(ANO_FC_RC, data, 20);
}

/**
 * @brief 发送 GPS 数据 (0x04)
 */
void ANO_SendGPS(void)
{
    uint8_t data[16];
    
    /* 占位数据 */
    memset(data, 0, sizeof(data));
    
    ANO_SendFrame(ANO_FC_GPS, data, 16);
}

/**
 * @brief 发送电源数据 (0x05)
 */
void ANO_SendPower(void)
{
    uint8_t data[7];
    
    /* 占位数据 */
    memset(data, 0, sizeof(data));
    
    ANO_SendFrame(ANO_FC_POWER, data, 7);
}

/**
 * @brief 发送电机数据 (0x06)
 */
void ANO_SendMotor(const uint16_t *motor)
{
    uint8_t data[8];
    
    if (motor != NULL) {
        data[0] = (motor[0] >> 8) & 0xFF;
        data[1] = motor[0] & 0xFF;
        data[2] = (motor[1] >> 8) & 0xFF;
        data[3] = motor[1] & 0xFF;
        data[4] = (motor[2] >> 8) & 0xFF;
        data[5] = motor[2] & 0xFF;
        data[6] = (motor[3] >> 8) & 0xFF;
        data[7] = motor[3] & 0xFF;
    } else {
        memset(data, 0, sizeof(data));
    }
    
    ANO_SendFrame(ANO_FC_MOTOR, data, 8);
}

/**
 * @brief 发送姿态数据 (0x0A)
 */
void ANO_SendAttitude(float roll, float pitch, float yaw)
{
    uint8_t data[6];
    
    int16_t roll_i = (int16_t)(roll * 100.0f);
    int16_t pitch_i = (int16_t)(pitch * 100.0f);
    int16_t yaw_i = (int16_t)(yaw * 100.0f);
    
    data[0] = (roll_i >> 8) & 0xFF;
    data[1] = roll_i & 0xFF;
    data[2] = (pitch_i >> 8) & 0xFF;
    data[3] = pitch_i & 0xFF;
    data[4] = (yaw_i >> 8) & 0xFF;
    data[5] = yaw_i & 0xFF;
    
    ANO_SendFrame(ANO_FC_ATTITUDE, data, 6);
}

/**
 * @brief 发送高度数据 (0x0B)
 */
void ANO_SendHeight(float altitude, float velocity)
{
    uint8_t data[8];
    
    int32_t alt_cm = (int32_t)(altitude * 100.0f);
    int16_t vel_cms = (int16_t)(velocity * 100.0f);
    
    data[0] = (alt_cm >> 24) & 0xFF;
    data[1] = (alt_cm >> 16) & 0xFF;
    data[2] = (alt_cm >> 8) & 0xFF;
    data[3] = alt_cm & 0xFF;
    data[4] = (vel_cms >> 8) & 0xFF;
    data[5] = vel_cms & 0xFF;
    data[6] = 0;  /* 预留 */
    data[7] = 0;
    
    ANO_SendFrame(ANO_FC_HEIGHT, data, 8);
}

/**
 * @brief 发送用户自定义数据 (0xF0)
 */
void ANO_SendUserData(const uint8_t *data, uint8_t len)
{
    ANO_SendFrame(ANO_FC_USER, data, len);
}

/**
 * @brief 解析接收字节
 */
void ANO_ParseByte(uint8_t byte)
{
    switch (ano_rx_state) {
        case 0:  /* 等待帧头 */
            if (byte == ANO_FRAME_HEAD) {
                ano_rx_frame.head = byte;
                ano_rx_state = 1;
            }
            break;
            
        case 1:  /* 地址 */
            ano_rx_frame.addr = byte;
            ano_rx_state = 2;
            break;
            
        case 2:  /* 功能码 */
            ano_rx_frame.func = byte;
            ano_rx_state = 3;
            break;
            
        case 3:  /* 数据长度 */
            ano_rx_frame.len = byte;
            ano_rx_len = byte;
            ano_rx_index = 0;
            if (ano_rx_len > 0 && ano_rx_len <= ANO_MAX_FRAME_LEN) {
                ano_rx_state = 4;
            } else if (ano_rx_len == 0) {
                ano_rx_state = 5;
            } else {
                ano_rx_state = 0;  /* 长度错误 */
            }
            break;
            
        case 4:  /* 数据 */
            ano_rx_frame.data[ano_rx_index++] = byte;
            if (ano_rx_index >= ano_rx_len) {
                ano_rx_state = 5;
            }
            break;
            
        case 5:  /* 校验和 */
        {
            ano_rx_frame.sum = byte;
            
            /* 验证校验和 */
            uint8_t sum = ano_rx_frame.addr + ano_rx_frame.func + ano_rx_frame.len;
            uint8_t i;
            for (i = 0; i < ano_rx_len; i++) {
                sum += ano_rx_frame.data[i];
            }
            
            if (sum == ano_rx_frame.sum) {
                ANO_ProcessFrame(&ano_rx_frame);
            }
            
            ano_rx_state = 0;
            break;
        }
            
        default:
            ano_rx_state = 0;
            break;
    }
}

/**
 * @brief 处理接收到的帧
 */
void ANO_ProcessFrame(const ANO_Frame_t *frame)
{
    switch (frame->func) {
        case ANO_CMD_PARAM_READ:
            /* 参数读取请求 */
            break;
            
        case ANO_CMD_PARAM_WRITE:
            /* 参数写入请求 */
            break;
            
        case ANO_CMD_CONTROL:
            /* 控制指令 */
            break;
            
        case ANO_CMD_CALIB:
            /* 校准指令 */
            break;
            
        case ANO_CMD_FLIGHT:
            /* 飞行指令 */
            break;
            
        default:
            break;
    }
}

/**
 * @brief 计算校验和
 */
uint8_t ANO_CalculateSum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 设置错误码
 */
void ANO_SetError(uint16_t error)
{
    g_ano_error_code |= error;
}

/**
 * @brief 清除错误码
 */
void ANO_ClearError(uint16_t error)
{
    g_ano_error_code &= ~error;
}

/**
 * @brief 获取错误码
 */
uint16_t ANO_GetError(void)
{
    return g_ano_error_code;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 发送数据
 */
static void ANO_TransmitData(const uint8_t *data, uint16_t len)
{
    /* 使用 UART5 发送 */
    HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 100);
}
