/**
 * @file crsf.c
 * @brief CRSF (Crossfire Serial Protocol) 接收机驱动实现
 * @version 1.0.0
 * @date 2026-01-23
 *
 * @note 支持 ELRS/TBS Crossfire 原生协议
 *       使用 USART3 (PB10/PB11) @ 420000 baud
 */

/* Includes ------------------------------------------------------------------*/
#include "crsf.h"
#include "usart.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define CRSF_RX_BUFFER_SIZE     128     /**< DMA 接收缓冲区大小 */
#define CRSF_TX_BUFFER_SIZE     64      /**< 发送缓冲区大小 */

/* CRC8 DVB-S2 多项式 */
#define CRSF_CRC8_POLY          0xD5

/* Private variables ---------------------------------------------------------*/
static uint8_t crsf_rx_buffer[CRSF_RX_BUFFER_SIZE];     /**< DMA 接收缓冲区 */
static uint8_t crsf_tx_buffer[CRSF_TX_BUFFER_SIZE];     /**< 发送缓冲区 */
static uint8_t crsf_frame_buffer[CRSF_MAX_FRAME_SIZE];  /**< 帧解析缓冲区 */
static volatile uint16_t crsf_rx_head = 0;              /**< 接收头指针 */
static volatile uint16_t crsf_rx_tail = 0;              /**< 接收尾指针 */

/* CRC8 查找表 (DVB-S2 多项式 0xD5) */
static const uint8_t crsf_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/* Exported variables --------------------------------------------------------*/
CRSF_RawData_t g_crsf_raw = {0};
CRSF_LinkStats_t g_crsf_stats = {0};
RC_Data_t g_rc_data = {0};
volatile bool g_crsf_new_frame = false;

/* Private function prototypes -----------------------------------------------*/
static void CRSF_ProcessBuffer(void);
static bool CRSF_ValidateFrame(const uint8_t *frame, uint8_t len);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 计算 CRC8 (DVB-S2 多项式)
 */
uint8_t CRSF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crsf_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief 初始化 CRSF 接收
 */
CRSF_Status_t CRSF_Init(void)
{
    /* 清空缓冲区 */
    memset(crsf_rx_buffer, 0, sizeof(crsf_rx_buffer));
    memset(&g_crsf_raw, 0, sizeof(g_crsf_raw));
    memset(&g_crsf_stats, 0, sizeof(g_crsf_stats));
    memset(&g_rc_data, 0, sizeof(g_rc_data));
    
    /* 初始化默认值 */
    for (int i = 0; i < CRSF_CHANNELS; i++) {
        g_crsf_raw.channels[i] = CRSF_CHANNEL_MID;
    }
    g_crsf_raw.channels[2] = CRSF_CHANNEL_MIN;  /* 油门默认最低 (CH3) */
    
    g_rc_data.connected = false;
    g_rc_data.failsafe = true;
    g_rc_data.rssi = 0;
    g_rc_data.link_quality = 0;
    
    /* 重置指针 */
    crsf_rx_head = 0;
    crsf_rx_tail = 0;
    
    /* 启用 USART3 空闲中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    
    /* 启动 USART3 DMA 循环接收 */
    if (HAL_UART_Receive_DMA(&huart3, crsf_rx_buffer, CRSF_RX_BUFFER_SIZE) != HAL_OK) {
        return CRSF_ERROR;
    }
    
    return CRSF_OK;
}

/**
 * @brief 处理接收到的 CRSF 数据
 */
void CRSF_Process(void)
{
    uint32_t current_tick = HAL_GetTick();
    
    /* 检查超时 */
    if ((current_tick - g_crsf_raw.last_update_tick) > CRSF_TIMEOUT_MS) {
        g_rc_data.connected = false;
        
        /* 失控保护超时 */
        if ((current_tick - g_crsf_raw.last_update_tick) > CRSF_FAILSAFE_TIMEOUT) {
            g_rc_data.failsafe = true;
            g_crsf_raw.failsafe = true;
        }
        return;
    }
    
    /* 如果有新帧，处理数据 */
    if (g_crsf_new_frame) {
        g_crsf_new_frame = false;
        
        /* 归一化数据 */
        CRSF_Normalize(&g_crsf_raw, &g_rc_data);
        
        /* 更新连接状态 */
        g_rc_data.connected = !g_crsf_raw.failsafe;
        g_rc_data.failsafe = g_crsf_raw.failsafe;
    }
}

/**
 * @brief 解析 RC 通道数据
 */
bool CRSF_ParseRCChannels(const uint8_t *payload, CRSF_RawData_t *raw)
{
    /* CRSF RC 通道数据格式: 16 通道 × 11 位 = 176 位 = 22 字节
     * 与 SBUS 格式相同，但值范围不同
     */
    raw->channels[0]  = ((payload[0]       | payload[1]  << 8) & 0x07FF);
    raw->channels[1]  = ((payload[1]  >> 3 | payload[2]  << 5) & 0x07FF);
    raw->channels[2]  = ((payload[2]  >> 6 | payload[3]  << 2 | payload[4] << 10) & 0x07FF);
    raw->channels[3]  = ((payload[4]  >> 1 | payload[5]  << 7) & 0x07FF);
    raw->channels[4]  = ((payload[5]  >> 4 | payload[6]  << 4) & 0x07FF);
    raw->channels[5]  = ((payload[6]  >> 7 | payload[7]  << 1 | payload[8] << 9) & 0x07FF);
    raw->channels[6]  = ((payload[8]  >> 2 | payload[9]  << 6) & 0x07FF);
    raw->channels[7]  = ((payload[9]  >> 5 | payload[10] << 3) & 0x07FF);
    raw->channels[8]  = ((payload[11]      | payload[12] << 8) & 0x07FF);
    raw->channels[9]  = ((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
    raw->channels[10] = ((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
    raw->channels[11] = ((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
    raw->channels[12] = ((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
    raw->channels[13] = ((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
    raw->channels[14] = ((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
    raw->channels[15] = ((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
    
    raw->failsafe = false;
    raw->last_update_tick = HAL_GetTick();
    
    return true;
}

/**
 * @brief 解析链路统计数据
 */
bool CRSF_ParseLinkStats(const uint8_t *payload, CRSF_LinkStats_t *stats)
{
    stats->uplink_rssi_1 = payload[0];
    stats->uplink_rssi_2 = payload[1];
    stats->uplink_lq = payload[2];
    stats->uplink_snr = (int8_t)payload[3];
    stats->active_antenna = payload[4];
    stats->rf_mode = payload[5];
    stats->uplink_tx_power = payload[6];
    stats->downlink_rssi = payload[7];
    stats->downlink_lq = payload[8];
    stats->downlink_snr = (int8_t)payload[9];
    
    /* 更新全局 RC 数据中的链路质量 */
    g_rc_data.link_quality = stats->uplink_lq;
    g_rc_data.rssi = stats->uplink_lq;  /* 使用 LQ 作为 RSSI 百分比 */
    g_rc_data.rssi_dbm = -(int8_t)stats->uplink_rssi_1;  /* 转换为负值 */
    
    /* 设置警告标志 */
    g_rc_data.rssi_warning = (stats->uplink_lq < CRSF_LQ_WARNING);
    g_rc_data.rssi_critical = (stats->uplink_lq < CRSF_LQ_CRITICAL);
    
    return true;
}

/**
 * @brief 解析 CRSF 帧
 */
bool CRSF_ParseFrame(const uint8_t *buffer, uint16_t len)
{
    if (len < 4) return false;  /* 最小帧长度: Sync + Len + Type + CRC */
    
    uint8_t sync = buffer[0];
    uint8_t frame_len = buffer[1];
    uint8_t frame_type = buffer[2];
    
    /* 验证同步字节 */
    if (sync != CRSF_SYNC_BYTE && sync != CRSF_SYNC_BYTE_FC) {
        return false;
    }
    
    /* 验证帧长度 */
    if (frame_len < 2 || frame_len > CRSF_MAX_PAYLOAD_SIZE + 2) {
        return false;
    }
    
    /* 验证缓冲区长度 */
    if (len < (uint16_t)(frame_len + 2)) {
        return false;
    }
    
    /* 验证 CRC (从 Type 开始到 CRC 前一个字节) */
    uint8_t crc_calc = CRSF_CRC8(&buffer[2], frame_len - 1);
    uint8_t crc_recv = buffer[frame_len + 1];
    
    if (crc_calc != crc_recv) {
        return false;
    }
    
    /* 根据帧类型处理 */
    const uint8_t *payload = &buffer[3];
    
    switch (frame_type) {
        case CRSF_FRAMETYPE_RC_CHANNELS:
            if (CRSF_ParseRCChannels(payload, &g_crsf_raw)) {
                g_crsf_new_frame = true;
            }
            break;
            
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            CRSF_ParseLinkStats(payload, &g_crsf_stats);
            break;
            
        default:
            /* 其他帧类型暂不处理 */
            break;
    }
    
    return true;
}

/**
 * @brief 将原始值转换为归一化值
 */
void CRSF_Normalize(const CRSF_RawData_t *raw, RC_Data_t *rc)
{
    /* 通道映射 (AETR 顺序):
     * CH1 = Roll (A)
     * CH2 = Pitch (E)
     * CH3 = Throttle (T)
     * CH4 = Yaw (R)
     * CH5 = Aux1 (解锁开关)
     * CH6 = Aux2 (模式开关)
     * CH7 = Aux3
     * CH8 = Aux4
     */
    
    /* 油门: 0.0 - 1.0 */
    rc->throttle = CRSF_Map(raw->channels[2], 0.0f, 1.0f);
    
    /* 摇杆: -1.0 - 1.0 */
    rc->roll  = CRSF_Map(raw->channels[0], -1.0f, 1.0f);
    rc->pitch = CRSF_Map(raw->channels[1], -1.0f, 1.0f);
    rc->yaw   = CRSF_Map(raw->channels[3], -1.0f, 1.0f);
    
    /* 辅助通道 */
    rc->aux1 = CRSF_Map(raw->channels[4], -1.0f, 1.0f);
    rc->aux2 = CRSF_Map(raw->channels[5], -1.0f, 1.0f);
    rc->aux3 = CRSF_Map(raw->channels[6], -1.0f, 1.0f);
    rc->aux4 = CRSF_Map(raw->channels[7], -1.0f, 1.0f);
    
    /* 解锁开关 (CH5): 低位=锁定, 高位=解锁 */
    rc->arm_switch = (raw->channels[4] > CRSF_CHANNEL_MID) ? 1 : 0;
    
    /* 模式开关 (CH6): 三段开关 */
    if (raw->channels[5] < (CRSF_CHANNEL_MIN + 400)) {
        rc->mode_switch = 0;  /* 自稳模式 */
    } else if (raw->channels[5] > (CRSF_CHANNEL_MAX - 400)) {
        rc->mode_switch = 2;  /* 手动模式 */
    } else {
        rc->mode_switch = 1;  /* 半自稳模式 */
    }
}

/**
 * @brief 检查 CRSF 连接状态
 */
bool CRSF_IsConnected(void)
{
    return g_rc_data.connected;
}

/**
 * @brief 检查失控保护状态
 */
bool CRSF_IsFailsafe(void)
{
    return g_rc_data.failsafe;
}

/**
 * @brief 获取指定通道的原始值
 */
uint16_t CRSF_GetChannel(uint8_t channel)
{
    if (channel >= CRSF_CHANNELS) {
        return CRSF_CHANNEL_MID;
    }
    return g_crsf_raw.channels[channel];
}

/**
 * @brief 获取指定通道的归一化值
 */
float CRSF_GetChannelNormalized(uint8_t channel)
{
    if (channel >= CRSF_CHANNELS) {
        return 0.0f;
    }
    return CRSF_Map(g_crsf_raw.channels[channel], -1.0f, 1.0f);
}

/**
 * @brief 将 CRSF 值映射到指定范围
 */
float CRSF_Map(uint16_t value, float out_min, float out_max)
{
    /* 限制输入范围 */
    if (value < CRSF_CHANNEL_MIN) value = CRSF_CHANNEL_MIN;
    if (value > CRSF_CHANNEL_MAX) value = CRSF_CHANNEL_MAX;
    
    /* 线性映射 */
    float normalized = (float)(value - CRSF_CHANNEL_MIN) / (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    return out_min + normalized * (out_max - out_min);
}

/**
 * @brief 处理 DMA 缓冲区数据
 */
static void CRSF_ProcessBuffer(void)
{
    /* 获取当前 DMA 位置 */
    uint16_t dma_pos = CRSF_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    
    /* 处理新接收的数据 */
    while (crsf_rx_tail != dma_pos) {
        uint8_t byte = crsf_rx_buffer[crsf_rx_tail];
        
        /* 查找同步字节 */
        if (byte == CRSF_SYNC_BYTE || byte == CRSF_SYNC_BYTE_FC) {
            /* 尝试解析帧 */
            uint16_t available = (dma_pos >= crsf_rx_tail) ? 
                                 (dma_pos - crsf_rx_tail) : 
                                 (CRSF_RX_BUFFER_SIZE - crsf_rx_tail + dma_pos);
            
            if (available >= 3) {
                /* 读取帧长度 */
                uint16_t len_pos = (crsf_rx_tail + 1) % CRSF_RX_BUFFER_SIZE;
                uint8_t frame_len = crsf_rx_buffer[len_pos];
                
                if (frame_len >= 2 && frame_len <= CRSF_MAX_PAYLOAD_SIZE + 2) {
                    uint16_t total_len = frame_len + 2;  /* Sync + Len + Payload + CRC */
                    
                    if (available >= total_len) {
                        /* 复制帧到解析缓冲区 */
                        for (uint16_t i = 0; i < total_len; i++) {
                            crsf_frame_buffer[i] = crsf_rx_buffer[(crsf_rx_tail + i) % CRSF_RX_BUFFER_SIZE];
                        }
                        
                        /* 解析帧 */
                        if (CRSF_ParseFrame(crsf_frame_buffer, total_len)) {
                            /* 成功解析，跳过整个帧 */
                            crsf_rx_tail = (crsf_rx_tail + total_len) % CRSF_RX_BUFFER_SIZE;
                            continue;
                        }
                    }
                }
            }
        }
        
        /* 移动到下一个字节 */
        crsf_rx_tail = (crsf_rx_tail + 1) % CRSF_RX_BUFFER_SIZE;
    }
}

/**
 * @brief UART 空闲中断回调
 */
void CRSF_UART_IdleCallback(void)
{
    /* 检查是否是 USART3 的空闲中断 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
        /* 清除空闲标志 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        
        /* 处理缓冲区数据 */
        CRSF_ProcessBuffer();
    }
}

/**
 * @brief DMA 接收完成回调
 */
void CRSF_DMA_RxCpltCallback(void)
{
    /* 处理缓冲区数据 */
    CRSF_ProcessBuffer();
}

/**
 * @brief DMA 半传输完成回调
 */
void CRSF_DMA_RxHalfCpltCallback(void)
{
    /* 处理缓冲区数据 */
    CRSF_ProcessBuffer();
}

/**
 * @brief 重置 CRSF 接收状态
 */
void CRSF_Reset(void)
{
    /* 停止 DMA */
    HAL_UART_DMAStop(&huart3);
    
    /* 清空缓冲区 */
    memset(crsf_rx_buffer, 0, sizeof(crsf_rx_buffer));
    memset(crsf_frame_buffer, 0, sizeof(crsf_frame_buffer));
    
    /* 重置指针 */
    crsf_rx_head = 0;
    crsf_rx_tail = 0;
    
    /* 重置状态 */
    g_crsf_new_frame = false;
    g_rc_data.connected = false;
    g_rc_data.failsafe = true;
    g_rc_data.rssi = 0;
    g_rc_data.link_quality = 0;
    g_rc_data.rssi_warning = true;
    g_rc_data.rssi_critical = true;
    
    /* 重新启动 DMA */
    HAL_UART_Receive_DMA(&huart3, crsf_rx_buffer, CRSF_RX_BUFFER_SIZE);
}

/**
 * @brief 获取链路质量
 */
uint8_t CRSF_GetLinkQuality(void)
{
    return g_rc_data.link_quality;
}

/**
 * @brief 获取 RSSI (dBm)
 */
int8_t CRSF_GetRSSI_dBm(void)
{
    return g_rc_data.rssi_dbm;
}

/**
 * @brief 检查 RSSI 是否处于警告状态
 */
bool CRSF_IsRSSIWarning(void)
{
    return g_rc_data.rssi_warning;
}

/**
 * @brief 检查 RSSI 是否处于临界状态
 */
bool CRSF_IsRSSICritical(void)
{
    return g_rc_data.rssi_critical;
}

/**
 * @brief 发送遥测数据 (电池信息)
 */
CRSF_Status_t CRSF_SendTelemetryBattery(uint16_t voltage, uint16_t current, 
                                         uint32_t capacity, uint8_t remaining)
{
    uint8_t payload[8];
    
    /* 电压: 0.1V 单位, 大端序 */
    payload[0] = (voltage >> 8) & 0xFF;
    payload[1] = voltage & 0xFF;
    
    /* 电流: 0.1A 单位, 大端序 */
    payload[2] = (current >> 8) & 0xFF;
    payload[3] = current & 0xFF;
    
    /* 已用容量: mAh, 大端序 (24位) */
    payload[4] = (capacity >> 16) & 0xFF;
    payload[5] = (capacity >> 8) & 0xFF;
    payload[6] = capacity & 0xFF;
    
    /* 剩余百分比 */
    payload[7] = remaining;
    
    /* 构建帧 */
    crsf_tx_buffer[0] = CRSF_SYNC_BYTE_FC;
    crsf_tx_buffer[1] = 10;  /* Len = Type + Payload + CRC */
    crsf_tx_buffer[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
    memcpy(&crsf_tx_buffer[3], payload, 8);
    crsf_tx_buffer[11] = CRSF_CRC8(&crsf_tx_buffer[2], 9);
    
    /* 发送 */
    if (HAL_UART_Transmit_DMA(&huart3, crsf_tx_buffer, 12) != HAL_OK) {
        return CRSF_ERROR;
    }
    
    return CRSF_OK;
}

/**
 * @brief 发送遥测数据 (GPS 信息)
 */
CRSF_Status_t CRSF_SendTelemetryGPS(int32_t lat, int32_t lon, uint16_t speed,
                                     uint16_t heading, uint16_t altitude, uint8_t satellites)
{
    uint8_t payload[15];
    
    /* 纬度: 度 × 10^7, 大端序 */
    payload[0] = (lat >> 24) & 0xFF;
    payload[1] = (lat >> 16) & 0xFF;
    payload[2] = (lat >> 8) & 0xFF;
    payload[3] = lat & 0xFF;
    
    /* 经度: 度 × 10^7, 大端序 */
    payload[4] = (lon >> 24) & 0xFF;
    payload[5] = (lon >> 16) & 0xFF;
    payload[6] = (lon >> 8) & 0xFF;
    payload[7] = lon & 0xFF;
    
    /* 地速: cm/s, 大端序 */
    payload[8] = (speed >> 8) & 0xFF;
    payload[9] = speed & 0xFF;
    
    /* 航向: 度 × 100, 大端序 */
    payload[10] = (heading >> 8) & 0xFF;
    payload[11] = heading & 0xFF;
    
    /* 海拔: m + 1000m 偏移, 大端序 */
    payload[12] = (altitude >> 8) & 0xFF;
    payload[13] = altitude & 0xFF;
    
    /* 卫星数 */
    payload[14] = satellites;
    
    /* 构建帧 */
    crsf_tx_buffer[0] = CRSF_SYNC_BYTE_FC;
    crsf_tx_buffer[1] = 17;  /* Len = Type + Payload + CRC */
    crsf_tx_buffer[2] = CRSF_FRAMETYPE_GPS;
    memcpy(&crsf_tx_buffer[3], payload, 15);
    crsf_tx_buffer[18] = CRSF_CRC8(&crsf_tx_buffer[2], 16);
    
    /* 发送 */
    if (HAL_UART_Transmit_DMA(&huart3, crsf_tx_buffer, 19) != HAL_OK) {
        return CRSF_ERROR;
    }
    
    return CRSF_OK;
}

/**
 * @brief 发送遥测数据 (姿态信息)
 */
CRSF_Status_t CRSF_SendTelemetryAttitude(int16_t pitch, int16_t roll, int16_t yaw)
{
    uint8_t payload[6];
    
    /* 俯仰角: 弧度 × 10000, 大端序 */
    payload[0] = (pitch >> 8) & 0xFF;
    payload[1] = pitch & 0xFF;
    
    /* 横滚角: 弧度 × 10000, 大端序 */
    payload[2] = (roll >> 8) & 0xFF;
    payload[3] = roll & 0xFF;
    
    /* 偏航角: 弧度 × 10000, 大端序 */
    payload[4] = (yaw >> 8) & 0xFF;
    payload[5] = yaw & 0xFF;
    
    /* 构建帧 */
    crsf_tx_buffer[0] = CRSF_SYNC_BYTE_FC;
    crsf_tx_buffer[1] = 8;  /* Len = Type + Payload + CRC */
    crsf_tx_buffer[2] = CRSF_FRAMETYPE_ATTITUDE;
    memcpy(&crsf_tx_buffer[3], payload, 6);
    crsf_tx_buffer[9] = CRSF_CRC8(&crsf_tx_buffer[2], 7);
    
    /* 发送 */
    if (HAL_UART_Transmit_DMA(&huart3, crsf_tx_buffer, 10) != HAL_OK) {
        return CRSF_ERROR;
    }
    
    return CRSF_OK;
}

/**
 * @brief 发送遥测数据 (飞行模式)
 */
CRSF_Status_t CRSF_SendTelemetryFlightMode(const char *mode)
{
    uint8_t len = strlen(mode);
    if (len > 14) len = 14;  /* 最多 14 字符 */
    
    /* 构建帧 */
    crsf_tx_buffer[0] = CRSF_SYNC_BYTE_FC;
    crsf_tx_buffer[1] = len + 3;  /* Len = Type + Payload + CRC */
    crsf_tx_buffer[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&crsf_tx_buffer[3], mode, len);
    crsf_tx_buffer[3 + len] = 0;  /* 字符串结束符 */
    crsf_tx_buffer[4 + len] = CRSF_CRC8(&crsf_tx_buffer[2], len + 2);
    
    /* 发送 */
    if (HAL_UART_Transmit_DMA(&huart3, crsf_tx_buffer, 5 + len) != HAL_OK) {
        return CRSF_ERROR;
    }
    
    return CRSF_OK;
}