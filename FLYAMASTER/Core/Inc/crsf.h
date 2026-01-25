/**
 * @file crsf.h
 * @brief CRSF (Crossfire) 协议驱动 - ELRS 接收机
 * @version 1.0.0
 * @date 2026-01-24
 *
 * @details
 * 实现 CRSF 协议解析 (用于 ELRS 接收机):
 * 1. 使用 USART3 DMA 循环接收 (PB10/PB11)
 * 2. 解析 16 通道数据
 * 3. 检测失控保护 (Failsafe)
 * 4. 读取 RSSI/LQ/SNR 遥测数据
 *
 * CRSF 协议规格：
 * - 波特率: 420000 bps
 * - 数据位: 8
 * - 校验: None
 * - 停止位: 1
 * - 无需信号反相
 *
 * 硬件连接:
 * - USART3_TX: PB10 (ELRS_TX)
 * - USART3_RX: PB11 (ELRS_RX)
 * - DMA: DMA1_Stream3 (TX) / DMA1_Stream1 (RX)
 */

#ifndef __CRSF_H
#define __CRSF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define CRSF_BAUDRATE           420000  /**< CRSF 波特率 */
#define CRSF_MAX_FRAME_SIZE     64      /**< 最大帧长度 */
#define CRSF_CHANNELS           16      /**< 通道数 */

/* CRSF 帧类型 */
#define CRSF_SYNC_BYTE          0xC8    /**< 同步字节 (来自接收机) */
#define CRSF_SYNC_BYTE_FC       0xEE    /**< 同步字节 (来自飞控) */

/* CRSF 帧类型定义 */
#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21

/* 通道值范围 (CRSF 使用 11 位, 范围 172-1811) */
#define CRSF_CHANNEL_MIN        172     /**< 最小值 */
#define CRSF_CHANNEL_MID        992     /**< 中间值 */
#define CRSF_CHANNEL_MAX        1811    /**< 最大值 */

/* 超时检测 */
#define CRSF_TIMEOUT_MS         100     /**< 信号超时时间 (ms) */

/* Link Quality 阈值 */
#define CRSF_LQ_CRITICAL        20      /**< 临界信号质量 (%) */
#define CRSF_LQ_WARNING         50      /**< 警告信号质量 (%) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CRSF 链路统计数据
 */
typedef struct {
    uint8_t uplink_rssi_1;      /**< 上行 RSSI 天线 1 (dBm, 负值) */
    uint8_t uplink_rssi_2;      /**< 上行 RSSI 天线 2 (dBm, 负值) */
    uint8_t uplink_lq;          /**< 上行链路质量 (0-100%) */
    int8_t uplink_snr;          /**< 上行信噪比 (dB) */
    uint8_t active_antenna;     /**< 活动天线 (0 或 1) */
    uint8_t rf_mode;            /**< RF 模式 */
    uint8_t uplink_tx_power;    /**< 上行发射功率 (mW 编码) */
    uint8_t downlink_rssi;      /**< 下行 RSSI (dBm, 负值) */
    uint8_t downlink_lq;        /**< 下行链路质量 (0-100%) */
    int8_t downlink_snr;        /**< 下行信噪比 (dB) */
} CRSF_LinkStats_t;

/**
 * @brief CRSF 原始数据结构
 */
typedef struct {
    uint16_t channels[CRSF_CHANNELS];   /**< 16 通道原始值 (172-1811) */
    bool failsafe;                      /**< 失控保护标志 */
} CRSF_RawData_t;

/**
 * @brief RC 遥控器数据结构 (归一化)
 */
typedef struct {
    float throttle;     /**< 油门 (0.0 - 1.0) */
    float roll;         /**< 横滚 (-1.0 - 1.0) */
    float pitch;        /**< 俯仰 (-1.0 - 1.0) */
    float yaw;          /**< 偏航 (-1.0 - 1.0) */
    float aux1;         /**< 辅助通道 1 (-1.0 - 1.0) */
    float aux2;         /**< 辅助通道 2 (-1.0 - 1.0) */
    float aux3;         /**< 辅助通道 3 (-1.0 - 1.0) */
    float aux4;         /**< 辅助通道 4 (-1.0 - 1.0) */
    uint8_t arm_switch;     /**< 解锁开关 (0=锁定, 1=解锁) */
    uint8_t mode_switch;    /**< 模式开关 (0=自稳, 1=半自稳, 2=手动) */
    bool connected;         /**< 连接状态 */
    bool failsafe;          /**< 失控保护 */
    
    /* RSSI/Link Quality (来自 CRSF 遥测) */
    int8_t rssi_dbm;        /**< RSSI (dBm, 负值) */
    uint8_t rssi_percent;   /**< RSSI 百分比 (0-100%) */
    uint8_t link_quality;   /**< 链路质量 (0-100%) */
    int8_t snr;             /**< 信噪比 (dB) */
    bool rssi_warning;      /**< RSSI 警告 */
    bool rssi_critical;     /**< RSSI 临界 */
} CRSF_RCData_t;

/**
 * @brief CRSF 状态枚举
 */
typedef enum {
    CRSF_OK = 0,
    CRSF_ERROR,
    CRSF_TIMEOUT,
    CRSF_CRC_ERROR,
    CRSF_FAILSAFE
} CRSF_Status_t;

/* Exported variables --------------------------------------------------------*/
extern CRSF_RawData_t g_crsf_raw;       /**< 全局 CRSF 原始数据 */
extern CRSF_RCData_t g_rc_data;         /**< 全局遥控器数据 */
extern CRSF_LinkStats_t g_crsf_link;    /**< 全局链路统计 */
extern volatile bool g_crsf_new_frame;  /**< 新帧标志 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 CRSF 接收
 * @return CRSF_Status_t 初始化状态
 * @note 启动 USART3 DMA 循环接收
 */
CRSF_Status_t CRSF_Init(void);

/**
 * @brief 处理接收到的 CRSF 数据
 * @note 在主循环或任务中周期性调用
 */
void CRSF_Process(void);

/**
 * @brief 解析 CRSF 帧
 * @param buffer 接收缓冲区
 * @param len 数据长度
 * @return bool true=解析成功, false=帧无效
 */
bool CRSF_ParseFrame(const uint8_t *buffer, uint16_t len);

/**
 * @brief 解析 RC 通道数据
 * @param payload 帧载荷
 * @param raw 输出原始数据
 * @return bool true=解析成功
 */
bool CRSF_ParseChannels(const uint8_t *payload, CRSF_RawData_t *raw);

/**
 * @brief 解析链路统计数据
 * @param payload 帧载荷
 * @param stats 输出链路统计
 * @return bool true=解析成功
 */
bool CRSF_ParseLinkStats(const uint8_t *payload, CRSF_LinkStats_t *stats);

/**
 * @brief 将原始值转换为归一化值
 * @param raw 原始数据
 * @param rc 输出遥控器数据
 */
void CRSF_Normalize(const CRSF_RawData_t *raw, CRSF_RCData_t *rc);

/**
 * @brief 检查 CRSF 连接状态
 * @return bool true=已连接, false=断开
 */
bool CRSF_IsConnected(void);

/**
 * @brief 检查失控保护状态
 * @return bool true=失控保护激活, false=正常
 */
bool CRSF_IsFailsafe(void);

/**
 * @brief 获取指定通道的原始值
 * @param channel 通道号 (0-15)
 * @return uint16_t 通道值 (172-1811)
 */
uint16_t CRSF_GetChannel(uint8_t channel);

/**
 * @brief 获取指定通道的归一化值
 * @param channel 通道号 (0-15)
 * @return float 归一化值 (-1.0 - 1.0)
 */
float CRSF_GetChannelNormalized(uint8_t channel);

/**
 * @brief 将 CRSF 值映射到指定范围
 * @param value CRSF 原始值
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @return float 映射后的值
 */
float CRSF_Map(uint16_t value, float out_min, float out_max);

/**
 * @brief UART 空闲中断回调 (用于帧检测)
 * @note 在 stm32f4xx_it.c 中调用
 */
void CRSF_UART_IdleCallback(void);

/**
 * @brief DMA 接收完成回调
 * @note 在 HAL_UART_RxCpltCallback 中调用
 */
void CRSF_DMA_RxCpltCallback(void);

/**
 * @brief 重置 CRSF 接收状态
 */
void CRSF_Reset(void);

/**
 * @brief 获取 RSSI 值 (dBm)
 * @return int8_t RSSI (dBm, 负值)
 */
int8_t CRSF_GetRSSI_dBm(void);

/**
 * @brief 获取 RSSI 百分比
 * @return uint8_t RSSI 百分比 (0-100)
 */
uint8_t CRSF_GetRSSI_Percent(void);

/**
 * @brief 获取链路质量
 * @return uint8_t 链路质量百分比 (0-100)
 */
uint8_t CRSF_GetLinkQuality(void);

/**
 * @brief 获取信噪比
 * @return int8_t SNR (dB)
 */
int8_t CRSF_GetSNR(void);

/**
 * @brief 检查 RSSI 是否处于警告状态
 * @return bool true=警告
 */
bool CRSF_IsRSSIWarning(void);

/**
 * @brief 检查 RSSI 是否处于临界状态
 * @return bool true=临界
 */
bool CRSF_IsRSSICritical(void);

/**
 * @brief 计算 CRSF CRC8
 * @param data 数据指针
 * @param len 数据长度
 * @return uint8_t CRC8 值
 */
uint8_t CRSF_CRC8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H */