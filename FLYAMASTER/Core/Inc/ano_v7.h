/**
 * @file ano_v7.h
 * @brief 匿名上位机 V7 协议模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 匿名科创地面站 V7 协议实现，用于飞行数据可视化和参数调试。
 * 
 * 通信接口:
 * - UART5 (PC12/PD2) - 115200 bps
 * - 或 USB CDC 虚拟串口
 */

#ifndef __ANO_V7_H
#define __ANO_V7_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* 帧头帧尾 */
#define ANO_FRAME_HEAD          0xAA    /**< 帧头 */
#define ANO_FRAME_ADDR          0xFF    /**< 目标地址 (广播) */

/* 功能码定义 (上行 - 飞控发送) */
#define ANO_FC_STATUS           0x01    /**< 飞控状态 */
#define ANO_FC_SENSOR           0x02    /**< 传感器数据 */
#define ANO_FC_RC               0x03    /**< 遥控器数据 */
#define ANO_FC_GPS              0x04    /**< GPS 数据 */
#define ANO_FC_POWER            0x05    /**< 电源数据 */
#define ANO_FC_MOTOR            0x06    /**< 电机数据 */
#define ANO_FC_SENSOR2          0x07    /**< 传感器数据 2 */
#define ANO_FC_LINK             0x08    /**< 链路状态 */
#define ANO_FC_ATTITUDE         0x0A    /**< 姿态数据 */
#define ANO_FC_HEIGHT           0x0B    /**< 高度数据 */
#define ANO_FC_SPEED            0x0C    /**< 速度数据 */
#define ANO_FC_POSITION         0x0D    /**< 位置数据 */
#define ANO_FC_TARGET           0x0E    /**< 目标数据 */
#define ANO_FC_USER             0xF0    /**< 用户自定义 */

/* 功能码定义 (下行 - 上位机发送) */
#define ANO_CMD_PARAM_READ      0xE0    /**< 参数读取 */
#define ANO_CMD_PARAM_WRITE     0xE1    /**< 参数写入 */
#define ANO_CMD_CONTROL         0xE2    /**< 控制指令 */
#define ANO_CMD_CALIB           0xE3    /**< 校准指令 */
#define ANO_CMD_FLIGHT          0xE4    /**< 飞行指令 */

/* 错误码定义 */
#define ANO_ERROR_NONE          0x0000  /**< 无错误 */
#define ANO_ERROR_IMU_FAIL      0x0001  /**< IMU 失败 */
#define ANO_ERROR_BARO_FAIL     0x0002  /**< 气压计失败 */
#define ANO_ERROR_MAG_FAIL      0x0004  /**< 磁力计失败 */
#define ANO_ERROR_GPS_TIMEOUT   0x0008  /**< GPS 超时 */
#define ANO_ERROR_RC_LOST       0x0010  /**< 遥控器丢失 */
#define ANO_ERROR_LOW_BATTERY   0x0020  /**< 低电压 */
#define ANO_ERROR_MOTOR_LOCK    0x0040  /**< 电机锁定 */
#define ANO_ERROR_FLASH_FAIL    0x0080  /**< Flash 失败 */
#define ANO_ERROR_SENSOR_CALIB  0x0100  /**< 需要校准 */
#define ANO_ERROR_ATTITUDE_ERR  0x0200  /**< 姿态异常 */
#define ANO_ERROR_FAILSAFE      0x0400  /**< Failsafe 触发 */
#define ANO_ERROR_ARMING_BLOCK  0x0800  /**< 解锁阻止 */

/* 飞行模式定义 */
#define ANO_MODE_STABILIZE      0x01    /**< 自稳模式 */
#define ANO_MODE_ALTITUDE       0x02    /**< 定高模式 */
#define ANO_MODE_POSITION       0x03    /**< 定点模式 */
#define ANO_MODE_ACRO           0x04    /**< 特技模式 */
#define ANO_MODE_RTH            0x05    /**< 返航模式 */
#define ANO_MODE_LAND           0x06    /**< 降落模式 */
#define ANO_MODE_MISSION        0x07    /**< 航线模式 */

/* 解锁状态 */
#define ANO_LOCK_STATUS         0x00    /**< 锁定 */
#define ANO_UNLOCK_STATUS       0x01    /**< 解锁 */

/* 最大帧长度 */
#define ANO_MAX_FRAME_LEN       64

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ANO 帧结构
 */
typedef struct {
    uint8_t head;           /**< 帧头 0xAA */
    uint8_t addr;           /**< 目标地址 */
    uint8_t func;           /**< 功能码 */
    uint8_t len;            /**< 数据长度 */
    uint8_t data[ANO_MAX_FRAME_LEN]; /**< 数据 */
    uint8_t sum;            /**< 校验和 */
} ANO_Frame_t;

/**
 * @brief ANO 状态数据
 */
typedef struct {
    int16_t roll;           /**< 横滚角 x100 */
    int16_t pitch;          /**< 俯仰角 x100 */
    int16_t yaw;            /**< 航向角 x100 */
    int32_t altitude;       /**< 高度 cm */
    uint8_t mode;           /**< 飞行模式 */
    uint8_t armed;          /**< 解锁状态 */
} ANO_Status_t;

/**
 * @brief ANO 传感器数据
 */
typedef struct {
    int16_t acc_x;          /**< 加速度 X */
    int16_t acc_y;          /**< 加速度 Y */
    int16_t acc_z;          /**< 加速度 Z */
    int16_t gyro_x;         /**< 陀螺仪 X */
    int16_t gyro_y;         /**< 陀螺仪 Y */
    int16_t gyro_z;         /**< 陀螺仪 Z */
    int16_t mag_x;          /**< 磁力计 X */
    int16_t mag_y;          /**< 磁力计 Y */
    int16_t mag_z;          /**< 磁力计 Z */
} ANO_Sensor_t;

/**
 * @brief ANO 遥控器数据
 */
typedef struct {
    int16_t ch[16];         /**< 16 通道数据 */
} ANO_RC_t;

/**
 * @brief ANO GPS 数据
 */
typedef struct {
    uint8_t fix_type;       /**< 定位类型 */
    uint8_t satellites;     /**< 卫星数 */
    int32_t latitude;       /**< 纬度 x10^7 */
    int32_t longitude;      /**< 经度 x10^7 */
    int32_t altitude;       /**< 高度 cm */
    int16_t speed;          /**< 速度 cm/s */
    int16_t course;         /**< 航向 x100 */
} ANO_GPS_t;

/**
 * @brief ANO 电源数据
 */
typedef struct {
    uint16_t voltage;       /**< 电压 mV */
    uint16_t current;       /**< 电流 mA */
    uint16_t capacity;      /**< 已用容量 mAh */
    uint8_t percent;        /**< 电量百分比 */
} ANO_Power_t;

/* Exported variables --------------------------------------------------------*/
extern uint16_t g_ano_error_code;   /**< 全局错误码 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 ANO 协议
 */
void ANO_Init(void);

/**
 * @brief 发送状态数据
 */
void ANO_SendStatus(void);

/**
 * @brief 发送传感器数据
 */
void ANO_SendSensor(void);

/**
 * @brief 发送遥控器数据
 */
void ANO_SendRC(void);

/**
 * @brief 发送 GPS 数据
 */
void ANO_SendGPS(void);

/**
 * @brief 发送电源数据
 */
void ANO_SendPower(void);

/**
 * @brief 发送电机数据
 * @param motor 4 个电机 PWM 值
 */
void ANO_SendMotor(const uint16_t *motor);

/**
 * @brief 发送姿态数据
 * @param roll 横滚角 (度)
 * @param pitch 俯仰角 (度)
 * @param yaw 航向角 (度)
 */
void ANO_SendAttitude(float roll, float pitch, float yaw);

/**
 * @brief 发送高度数据
 * @param altitude 高度 (m)
 * @param velocity 垂直速度 (m/s)
 */
void ANO_SendHeight(float altitude, float velocity);

/**
 * @brief 发送用户自定义数据
 * @param data 数据
 * @param len 长度
 */
void ANO_SendUserData(const uint8_t *data, uint8_t len);

/**
 * @brief 处理接收到的帧
 * @param frame 帧数据
 */
void ANO_ProcessFrame(const ANO_Frame_t *frame);

/**
 * @brief 解析接收字节
 * @param byte 接收到的字节
 */
void ANO_ParseByte(uint8_t byte);

/**
 * @brief 发送帧
 * @param func 功能码
 * @param data 数据
 * @param len 长度
 */
void ANO_SendFrame(uint8_t func, const uint8_t *data, uint8_t len);

/**
 * @brief 计算校验和
 * @param data 数据
 * @param len 长度
 * @return uint8_t 校验和
 */
uint8_t ANO_CalculateSum(const uint8_t *data, uint8_t len);

/**
 * @brief 设置错误码
 * @param error 错误码
 */
void ANO_SetError(uint16_t error);

/**
 * @brief 清除错误码
 * @param error 错误码
 */
void ANO_ClearError(uint16_t error);

/**
 * @brief 获取错误码
 * @return uint16_t 当前错误码
 */
uint16_t ANO_GetError(void);

#ifdef __cplusplus
}
#endif

#endif /* __ANO_V7_H */