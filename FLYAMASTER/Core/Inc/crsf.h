/**
 * @file crsf.h
 * @brief CRSF (Crossfire Serial Protocol) 接收机驱动模块
 * @version 1.0.0
 * @date 2026-01-23
 *
 * @details
 * 实现 CRSF 协议解析 (ELRS/TBS Crossfire 原生协议):
 * 1. 使用 USART3 DMA 循环接收 (PB10/PB11)
 * 2. 解析 16 通道数据 (11 位/通道)
 * 3. 支持双向通信 (遥测回传)
 * 4. 内置 Link Quality 和 RSSI
 *
 * CRSF 协议规格：
 * - 波特率: 420000 bps (ELRS 标准)
 * - 数据位: 8
 * - 校验: None
 * - 停止位: 1
 * - 无需硬件反相器
 *
 * 帧格式:
 * [Sync] [Len] [Type] [Payload...] [CRC8]
 *   0xC8   N    0x16   RC Channels   CRC
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

/* CRSF 帧定义 */
#define CRSF_SYNC_BYTE          0xC8    /**< 同步字节 (来自接收机) */
#define CRSF_SYNC_BYTE_FC       0xEE    /**< 同步字节 (来自飞控) */
#define CRSF_MAX_FRAME_SIZE     64      /**< 最大帧长度 */
#define CRSF_MAX_PAYLOAD_SIZE   60      /**< 最大载荷长度 */

/* CRSF 帧类型 */
#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_VARIO            0x07
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_BARO_ALTITUDE    0x09
#define CRSF_FRAMETYPE_HEARTBEAT        0x0B
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16    /**< RC 通道数据 */
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21
#define CRSF_FRAMETYPE_DEVICE_PING      0x28
#define CRSF_FRAMETYPE_DEVICE_INFO      0x29
#define CRSF_FRAMETYPE_MSP_REQ          0x7A
#define CRSF_FRAMETYPE_MSP_RESP         0x7B
#define CRSF_FRAMETYPE_MSP_WRITE        0x7C

/* 通道配置 */
#define CRSF_CHANNELS           16      /**< 通道数 */
#define CRSF_CHANNEL_MIN        172     /**< 最小值 (对应 -100%) */
#define CRSF_CHANNEL_MID        992     /**< 中间值 (对应 0%) */
#define CRSF_CHANNEL_MAX        1811    /**< 最大值 (对应 +100%) */

/* RC 通道帧载荷长度 */
#define CRSF_RC_CHANNELS_PAYLOAD_SIZE   22  /**< 16 通道 × 11 位 = 176 位 = 22 字节 */

/* 超时检测 */
#define CRSF_TIMEOUT_MS         100     /**< 信号超时时间 (ms) */
#define CRSF_FAILSAFE_TIMEOUT   500     /**< 失控保护超时 (ms) */

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
    uint8_t uplink_tx_power;    /**< 上行发射功率 (mW) */
    uint8_t downlink_rssi;      /**< 下行 RSSI (dBm, 负值) */
    uint8_t downlink_lq;        /**< 下行链路质量 (0-100%) */
    int8_t downlink_snr;        /**< 下行信噪比 (dB) */
} CRSF_LinkStats_t;

/**
 * @brief CRSF 原始通道数据
 */
typedef struct {
    uint16_t channels[CRSF_CHANNELS];   /**< 16 通道原始值 (172-1811) */
    bool failsafe;                      /**< 失控保护标志 */
    uint32_t last_update_tick;          /**< 最后更新时间 */
} CRSF_RawData_t;

/**
 * @brief RC 遥控器归一化数据 (与 SBUS 兼容)
 */
typedef struct {
    float throttle;         /**< 油门 (0.0 - 1.0) */
    float roll;             /**< 横滚 (-1.0 - 1.0) */
    float pitch;            /**< 俯仰 (-1.0 - 1.0) */
    float yaw;              /**< 偏航 (-1.0 - 1.0) */
    float aux1;             /**< 辅助通道 1 (-1.0 - 1.0) */
    float aux2;             /**< 辅助通道 2 (-1.0 - 1.0) */
    float aux3;             /**< 辅助通道 3 (-1.0 - 1.0) */
    float aux4;             /**< 辅助通道 4 (-1.0 - 1.0) */
    uint8_t arm_switch;     /**< 解锁开关 (0=锁定, 1=解锁) */
    uint8_t mode_switch;    /**< 模式开关 (0=自稳, 1=半自稳, 2=手动) */
    bool connected;         /**< 连接状态 */
    bool failsafe;          /**< 失控保护 */
    
    /* RSSI/Link Quality (CRSF 原生支持) */
    uint8_t rssi;           /**< 信号强度 (0-100%, 从 LQ 映射) */
    uint8_t link_quality;   /**< 链路质量 (0-100%) */
    int8_t rssi_dbm;        /**< RSSI (dBm, 负值) */
    bool rssi_warning;      /**< RSSI 警告 */
    bool rssi_critical;     /**< RSSI 临界 */
} RC_Data_t;

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

/**
 * @brief CRSF 帧解析状态
 */
typedef enum {
    CRSF_STATE_SYNC,        /**< 等待同步字节 */
    CRSF_STATE_LEN,         /**< 等待长度字节 */
    CRSF_STATE_TYPE,        /**< 等待类型字节 */
    CRSF_STATE_PAYLOAD,     /**< 接收载荷 */
    CRSF_STATE_CRC          /**< 验证 CRC */
} CRSF_ParseState_t;

/* Exported variables --------------------------------------------------------*/
extern CRSF_RawData_t g_crsf_raw;       /**< 全局 CRSF 原始数据 */
extern CRSF_LinkStats_t g_crsf_stats;   /**< 全局链路统计 */
extern RC_Data_t g_rc_data;             /**< 全局遥控器数据 (兼容 SBUS) */
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
 * @param len 缓冲区长度
 * @return bool true=解析成功, false=帧无效
 */
bool CRSF_ParseFrame(const uint8_t *buffer, uint16_t len);

/**
 * @brief 解析 RC 通道数据
 * @param payload 载荷数据
 * @param raw 输出原始数据
 * @return bool true=解析成功
 */
bool CRSF_ParseRCChannels(const uint8_t *payload, CRSF_RawData_t *raw);

/**
 * @brief 解析链路统计数据
 * @param payload 载荷数据
 * @param stats 输出统计数据
 * @return bool true=解析成功
 */
bool CRSF_ParseLinkStats(const uint8_t *payload, CRSF_LinkStats_t *stats);

/**
 * @brief 将原始值转换为归一化值
 * @param raw 原始数据
 * @param rc 输出遥控器数据
 */
void CRSF_Normalize(const CRSF_RawData_t *raw, RC_Data_t *rc);

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
 * @brief DMA 半传输完成回调
 * @note 在 HAL_UART_RxHalfCpltCallback 中调用
 */
void CRSF_DMA_RxHalfCpltCallback(void);

/**
 * @brief 重置 CRSF 接收状态
 */
void CRSF_Reset(void);

/**
 * @brief 获取链路质量
 * @return uint8_t 链路质量百分比 (0-100)
 */
uint8_t CRSF_GetLinkQuality(void);

/**
 * @brief 获取 RSSI (dBm)
 * @return int8_t RSSI 值 (负数)
 */
int8_t CRSF_GetRSSI_dBm(void);

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
 * @brief 发送遥测数据 (电池信息)
 * @param voltage 电压 (0.1V 单位)
 * @param current 电流 (0.1A 单位)
 * @param capacity 已用容量 (mAh)
 * @param remaining 剩余百分比 (0-100)
 * @return CRSF_Status_t 发送状态
 */
CRSF_Status_t CRSF_SendTelemetryBattery(uint16_t voltage, uint16_t current, 
                                         uint32_t capacity, uint8_t remaining);

/**
 * @brief 发送遥测数据 (GPS 信息)
 * @param lat 纬度 (度 × 10^7)
 * @param lon 经度 (度 × 10^7)
 * @param speed 地速 (cm/s)
 * @param heading 航向 (度 × 100)
 * @param altitude 海拔 (m + 1000m 偏移)
 * @param satellites 卫星数
 * @return CRSF_Status_t 发送状态
 */
CRSF_Status_t CRSF_SendTelemetryGPS(int32_t lat, int32_t lon, uint16_t speed,
                                     uint16_t heading, uint16_t altitude, uint8_t satellites);

/**
 * @brief 发送遥测数据 (姿态信息)
 * @param pitch 俯仰角 (弧度 × 10000)
 * @param roll 横滚角 (弧度 × 10000)
 * @param yaw 偏航角 (弧度 × 10000)
 * @return CRSF_Status_t 发送状态
 */
CRSF_Status_t CRSF_SendTelemetryAttitude(int16_t pitch, int16_t roll, int16_t yaw);

/**
 * @brief 发送遥测数据 (飞行模式)
 * @param mode 飞行模式字符串 (最多 14 字符)
 * @return CRSF_Status_t 发送状态
 */
CRSF_Status_t CRSF_SendTelemetryFlightMode(const char *mode);

/**
 * @brief 计算 CRC8 (DVB-S2 多项式)
 * @param data 数据指针
 * @param len 数据长度
 * @return uint8_t CRC8 值
 */
uint8_t CRSF_CRC8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H */