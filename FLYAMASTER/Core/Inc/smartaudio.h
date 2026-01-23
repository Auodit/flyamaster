/**
 * @file smartaudio.h
 * @brief VTX SmartAudio 协议驱动模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * SmartAudio 是 TBS 开发的图传控制协议，用于远程调节 VTX 参数。
 * 
 * 硬件连接:
 * - UART4_TX (PC10) - 半双工模式
 * - 波特率: 4800 bps
 * - 协议: SmartAudio V2.1
 */

#ifndef __SMARTAUDIO_H
#define __SMARTAUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* SmartAudio 协议常量 */
#define SA_SYNC_BYTE            0xAA    /**< 同步字节 */
#define SA_HEADER_BYTE          0x55    /**< 头字节 */

/* SmartAudio 命令 */
#define SA_CMD_GET_SETTINGS     0x01    /**< 获取设置 */
#define SA_CMD_SET_POWER        0x02    /**< 设置功率 */
#define SA_CMD_SET_CHANNEL      0x03    /**< 设置频道 */
#define SA_CMD_SET_FREQ         0x04    /**< 设置频率 */
#define SA_CMD_SET_MODE         0x05    /**< 设置模式 */

/* SmartAudio 响应 */
#define SA_RESP_GET_SETTINGS    0x09    /**< 设置响应 */
#define SA_RESP_SET_POWER       0x0A    /**< 功率响应 */
#define SA_RESP_SET_CHANNEL     0x0B    /**< 频道响应 */
#define SA_RESP_SET_FREQ        0x0C    /**< 频率响应 */
#define SA_RESP_SET_MODE        0x0D    /**< 模式响应 */

/* 功率等级 (SmartAudio V2) */
#define SA_POWER_25MW           0       /**< 25mW */
#define SA_POWER_200MW          1       /**< 200mW */
#define SA_POWER_500MW          2       /**< 500mW */
#define SA_POWER_800MW          3       /**< 800mW */

/* 频段定义 */
#define SA_BAND_A               0       /**< Band A */
#define SA_BAND_B               1       /**< Band B */
#define SA_BAND_E               2       /**< Band E */
#define SA_BAND_F               3       /**< Fatshark */
#define SA_BAND_R               4       /**< Raceband */

/* 模式标志 */
#define SA_MODE_UNLOCKED        0x00    /**< 解锁模式 */
#define SA_MODE_PIT             0x01    /**< PIT 模式 */
#define SA_MODE_IN_RANGE_PIT    0x02    /**< 范围内 PIT */
#define SA_MODE_OUT_RANGE_PIT   0x04    /**< 范围外 PIT */

/* 超时设置 */
#define SA_TIMEOUT_MS           100     /**< 响应超时 (ms) */
#define SA_RETRY_COUNT          3       /**< 重试次数 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VTX 状态枚举
 */
typedef enum {
    SA_STATUS_UNKNOWN = 0,      /**< 未知状态 */
    SA_STATUS_READY,            /**< 就绪 */
    SA_STATUS_BUSY,             /**< 忙 */
    SA_STATUS_ERROR,            /**< 错误 */
    SA_STATUS_TIMEOUT           /**< 超时 */
} SA_Status_t;

/**
 * @brief VTX 设置结构体
 */
typedef struct {
    uint8_t channel;            /**< 当前频道 (0-39) */
    uint8_t power;              /**< 功率等级 (0-3) */
    uint8_t mode;               /**< 模式标志 */
    uint16_t frequency;         /**< 当前频率 (MHz) */
    uint8_t band;               /**< 频段 */
    uint8_t version;            /**< SmartAudio 版本 */
    bool pit_mode;              /**< PIT 模式状态 */
    bool unlocked;              /**< 解锁状态 */
} SA_Settings_t;

/**
 * @brief SmartAudio 数据结构体
 */
typedef struct {
    SA_Settings_t settings;     /**< VTX 设置 */
    SA_Status_t status;         /**< 当前状态 */
    bool initialized;           /**< 初始化标志 */
    bool connected;             /**< 连接状态 */
    uint32_t last_update;       /**< 上次更新时间 */
    uint8_t retry_count;        /**< 重试计数 */
    
    /* 接收缓冲区 */
    uint8_t rx_buffer[32];      /**< 接收缓冲区 */
    uint8_t rx_index;           /**< 接收索引 */
    bool rx_complete;           /**< 接收完成标志 */
} SA_Data_t;

/* Exported variables --------------------------------------------------------*/
extern SA_Data_t g_smartaudio_data;     /**< 全局 SmartAudio 数据 */

/* 频率表 (5 频段 x 8 频道) */
extern const uint16_t sa_frequency_table[5][8];

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SmartAudio
 * @return bool true=成功
 */
bool SA_Init(void);

/**
 * @brief 获取 VTX 设置
 * @return SA_Status_t 状态
 */
SA_Status_t SA_GetSettings(void);

/**
 * @brief 设置功率等级
 * @param power 功率等级 (0-3)
 * @return SA_Status_t 状态
 */
SA_Status_t SA_SetPower(uint8_t power);

/**
 * @brief 设置频道
 * @param channel 频道 (0-39)
 * @return SA_Status_t 状态
 */
SA_Status_t SA_SetChannel(uint8_t channel);

/**
 * @brief 设置频率
 * @param frequency 频率 (MHz)
 * @return SA_Status_t 状态
 */
SA_Status_t SA_SetFrequency(uint16_t frequency);

/**
 * @brief 设置 PIT 模式
 * @param enable true=启用 PIT
 * @return SA_Status_t 状态
 */
SA_Status_t SA_SetPitMode(bool enable);

/**
 * @brief 设置频段和频道
 * @param band 频段 (0-4)
 * @param channel 频道 (0-7)
 * @return SA_Status_t 状态
 */
SA_Status_t SA_SetBandChannel(uint8_t band, uint8_t channel);

/**
 * @brief 处理接收数据
 * @note 在 UART 接收中断中调用
 * @param data 接收到的字节
 */
void SA_ProcessRxByte(uint8_t data);

/**
 * @brief 处理接收完成
 * @note 在主循环或任务中调用
 */
void SA_ProcessResponse(void);

/**
 * @brief 周期性更新
 * @note 在 comm_task 中调用
 */
void SA_Update(void);

/**
 * @brief 检查连接状态
 * @return bool true=已连接
 */
bool SA_IsConnected(void);

/**
 * @brief 获取当前频率
 * @return uint16_t 频率 (MHz)
 */
uint16_t SA_GetFrequency(void);

/**
 * @brief 获取当前功率
 * @return uint8_t 功率等级
 */
uint8_t SA_GetPower(void);

/**
 * @brief 频道转频率
 * @param band 频段
 * @param channel 频道
 * @return uint16_t 频率 (MHz)
 */
uint16_t SA_ChannelToFrequency(uint8_t band, uint8_t channel);

/**
 * @brief 频率转频道
 * @param frequency 频率
 * @param band 输出频段
 * @param channel 输出频道
 * @return bool true=找到匹配
 */
bool SA_FrequencyToChannel(uint16_t frequency, uint8_t *band, uint8_t *channel);

/**
 * @brief 计算 CRC8
 * @param data 数据
 * @param len 长度
 * @return uint8_t CRC 值
 */
uint8_t SA_CalculateCRC(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SMARTAUDIO_H */