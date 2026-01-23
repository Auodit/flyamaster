/**
 * @file sbus.h
 * @brief SBUS 接收机驱动模块 (DMA 循环接收)
 * @version 1.1.0
 * @date 2026-01-18
 *
 * @details
 * 实现 SBUS 协议解析：
 * 1. 使用 UART4 DMA 循环接收
 * 2. 解析 16 通道数据
 * 3. 检测失控保护 (Failsafe)
 * 4. 支持通过 CH16 读取 RSSI (ELRS/Crossfire/FrSky)
 *
 * SBUS 协议规格：
 * - 波特率: 100000 bps
 * - 数据位: 8 (实际 9 位含校验)
 * - 校验: Even
 * - 停止位: 2
 * - 信号反相 (需要硬件反相器)
 *
 * RSSI 通道说明：
 * - 现代接收机 (ELRS, Crossfire, 新版 FrSky) 将 RSSI 嵌入 SBUS 通道
 * - 默认使用 CH16 传输 RSSI 值
 * - 不需要额外的模拟 RSSI 线 (PC5)
 * - 在接收机端配置 RSSI Channel = 16
 */

#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define SBUS_FRAME_SIZE         25      /**< SBUS 帧长度 */
#define SBUS_HEADER             0x0F    /**< 帧头 */
#define SBUS_FOOTER             0x00    /**< 帧尾 */
#define SBUS_CHANNELS           16      /**< 通道数 */

/* 通道值范围 */
#define SBUS_MIN_VALUE          172     /**< 最小值 (对应 -100%) */
#define SBUS_MID_VALUE          992     /**< 中间值 (对应 0%) */
#define SBUS_MAX_VALUE          1811    /**< 最大值 (对应 +100%) */

/* 标志位 */
#define SBUS_FLAG_CH17          0x01    /**< 通道 17 (数字) */
#define SBUS_FLAG_CH18          0x02    /**< 通道 18 (数字) */
#define SBUS_FLAG_FRAME_LOST    0x04    /**< 帧丢失 */
#define SBUS_FLAG_FAILSAFE      0x08    /**< 失控保护激活 */

/* 超时检测 */
#define SBUS_TIMEOUT_MS         100     /**< 信号超时时间 (ms) */

/* RSSI 通道配置 */
#define SBUS_RSSI_CHANNEL       15      /**< RSSI 通道索引 (CH16 = index 15) */
#define SBUS_RSSI_MIN           172     /**< RSSI 最小值 (对应 0%) */
#define SBUS_RSSI_MAX           1811    /**< RSSI 最大值 (对应 100%) */

/* Link Quality 阈值 */
#define SBUS_LQ_CRITICAL        20      /**< 临界信号质量 (%) */
#define SBUS_LQ_WARNING         50      /**< 警告信号质量 (%) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SBUS 原始数据结构
 */
typedef struct {
    uint16_t channels[SBUS_CHANNELS];   /**< 16 通道原始值 (172-1811) */
    bool ch17;                          /**< 数字通道 17 */
    bool ch18;                          /**< 数字通道 18 */
    bool frame_lost;                    /**< 帧丢失标志 */
    bool failsafe;                      /**< 失控保护标志 */
} SBUS_RawData_t;

/**
 * @brief SBUS 归一化数据结构
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
    
    /* RSSI/Link Quality (通过 CH16 获取) */
    uint8_t rssi;           /**< 信号强度 (0-100%) */
    uint8_t link_quality;   /**< 链路质量 (0-100%) */
    bool rssi_warning;      /**< RSSI 警告 */
    bool rssi_critical;     /**< RSSI 临界 */
} RC_Data_t;

/**
 * @brief SBUS 状态枚举
 */
typedef enum {
    SBUS_OK = 0,
    SBUS_ERROR,
    SBUS_TIMEOUT,
    SBUS_FRAME_LOST,
    SBUS_FAILSAFE
} SBUS_Status_t;

/* Exported variables --------------------------------------------------------*/
extern SBUS_RawData_t g_sbus_raw;   /**< 全局 SBUS 原始数据 */
extern RC_Data_t g_rc_data;         /**< 全局遥控器数据 */
extern volatile bool g_sbus_new_frame;  /**< 新帧标志 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SBUS 接收
 * @return SBUS_Status_t 初始化状态
 * @note 启动 UART4 DMA 循环接收
 */
SBUS_Status_t SBUS_Init(void);

/**
 * @brief 处理接收到的 SBUS 数据
 * @note 在主循环或任务中周期性调用
 */
void SBUS_Process(void);

/**
 * @brief 解析 SBUS 帧
 * @param buffer 接收缓冲区
 * @param raw 输出原始数据
 * @return bool true=解析成功, false=帧无效
 */
bool SBUS_ParseFrame(const uint8_t *buffer, SBUS_RawData_t *raw);

/**
 * @brief 将原始值转换为归一化值
 * @param raw 原始数据
 * @param rc 输出遥控器数据
 */
void SBUS_Normalize(const SBUS_RawData_t *raw, RC_Data_t *rc);

/**
 * @brief 检查 SBUS 连接状态
 * @return bool true=已连接, false=断开
 */
bool SBUS_IsConnected(void);

/**
 * @brief 检查失控保护状态
 * @return bool true=失控保护激活, false=正常
 */
bool SBUS_IsFailsafe(void);

/**
 * @brief 获取指定通道的原始值
 * @param channel 通道号 (0-15)
 * @return uint16_t 通道值 (172-1811)
 */
uint16_t SBUS_GetChannel(uint8_t channel);

/**
 * @brief 获取指定通道的归一化值
 * @param channel 通道号 (0-15)
 * @return float 归一化值 (-1.0 - 1.0)
 */
float SBUS_GetChannelNormalized(uint8_t channel);

/**
 * @brief 将 SBUS 值映射到指定范围
 * @param value SBUS 原始值
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @return float 映射后的值
 */
float SBUS_Map(uint16_t value, float out_min, float out_max);

/**
 * @brief UART 空闲中断回调 (用于帧检测)
 * @note 在 stm32f4xx_it.c 中调用
 */
void SBUS_UART_IdleCallback(void);

/**
 * @brief DMA 接收完成回调
 * @note 在 HAL_UART_RxCpltCallback 中调用
 */
void SBUS_DMA_RxCpltCallback(void);

/**
 * @brief 重置 SBUS 接收状态
 */
void SBUS_Reset(void);

/**
 * @brief 获取 RSSI 值 (通过 CH16)
 * @return uint8_t RSSI 百分比 (0-100)
 * @note 现代接收机 (ELRS/Crossfire/FrSky) 将 RSSI 嵌入 CH16
 */
uint8_t SBUS_GetRSSI(void);

/**
 * @brief 获取链路质量
 * @return uint8_t 链路质量百分比 (0-100)
 */
uint8_t SBUS_GetLinkQuality(void);

/**
 * @brief 检查 RSSI 是否处于警告状态
 * @return bool true=警告
 */
bool SBUS_IsRSSIWarning(void);

/**
 * @brief 检查 RSSI 是否处于临界状态
 * @return bool true=临界
 */
bool SBUS_IsRSSICritical(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H */