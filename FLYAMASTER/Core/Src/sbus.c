/**
 * @file sbus.c
 * @brief SBUS 接收机驱动实现
 * @version 1.1.0
 * @date 2026-01-18
 *
 * @note 支持通过 CH16 读取 RSSI (ELRS/Crossfire/FrSky)
 */

/* Includes ------------------------------------------------------------------*/
#include "sbus.h"
#include "usart.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define SBUS_RX_BUFFER_SIZE     50      /**< 双缓冲区大小 */

/* Private variables ---------------------------------------------------------*/
static uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];     /**< DMA 接收缓冲区 */
static uint8_t sbus_frame_buffer[SBUS_FRAME_SIZE];      /**< 帧解析缓冲区 */
static uint32_t sbus_last_update_tick = 0;              /**< 最后更新时间 */
static volatile uint8_t sbus_rx_index = 0;              /**< 接收索引 */

/* Exported variables --------------------------------------------------------*/
SBUS_RawData_t g_sbus_raw = {0};
RC_Data_t g_rc_data = {0};
volatile bool g_sbus_new_frame = false;

/* Private function prototypes -----------------------------------------------*/
static void SBUS_FindFrame(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 SBUS 接收
 */
SBUS_Status_t SBUS_Init(void)
{
    /* 清空缓冲区 */
    memset(sbus_rx_buffer, 0, sizeof(sbus_rx_buffer));
    memset(&g_sbus_raw, 0, sizeof(g_sbus_raw));
    memset(&g_rc_data, 0, sizeof(g_rc_data));
    
    /* 初始化默认值 */
    for (int i = 0; i < SBUS_CHANNELS; i++) {
        g_sbus_raw.channels[i] = SBUS_MID_VALUE;
    }
    g_sbus_raw.channels[0] = SBUS_MIN_VALUE;  /* 油门默认最低 */
    
    g_rc_data.connected = false;
    g_rc_data.failsafe = true;
    
    /* 启动 UART4 DMA 循环接收 */
    /* 使用空闲中断检测帧边界 */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    
    if (HAL_UART_Receive_DMA(&huart4, sbus_rx_buffer, SBUS_RX_BUFFER_SIZE) != HAL_OK) {
        return SBUS_ERROR;
    }
    
    return SBUS_OK;
}

/**
 * @brief 处理接收到的 SBUS 数据
 */
void SBUS_Process(void)
{
    uint32_t current_tick = HAL_GetTick();
    
    /* 检查超时 */
    if ((current_tick - sbus_last_update_tick) > SBUS_TIMEOUT_MS) {
        g_rc_data.connected = false;
        g_rc_data.failsafe = true;
        return;
    }
    
    /* 如果有新帧，处理数据 */
    if (g_sbus_new_frame) {
        g_sbus_new_frame = false;
        
        /* 解析帧 */
        if (SBUS_ParseFrame(sbus_frame_buffer, &g_sbus_raw)) {
            /* 归一化数据 */
            SBUS_Normalize(&g_sbus_raw, &g_rc_data);
            
            /* 更新连接状态 */
            g_rc_data.connected = !g_sbus_raw.failsafe && !g_sbus_raw.frame_lost;
            g_rc_data.failsafe = g_sbus_raw.failsafe;
            
            sbus_last_update_tick = current_tick;
        }
    }
}

/**
 * @brief 解析 SBUS 帧
 */
bool SBUS_ParseFrame(const uint8_t *buffer, SBUS_RawData_t *raw)
{
    /* 验证帧头帧尾 */
    if (buffer[0] != SBUS_HEADER) {
        return false;
    }
    
    /* 帧尾可以是 0x00, 0x04, 0x14, 0x24, 0x34 等 */
    /* 只检查低 4 位 */
    if ((buffer[24] & 0x0F) != 0x00 && (buffer[24] & 0x0F) != 0x04) {
        /* 某些接收机帧尾不标准，放宽检查 */
    }
    
    /* 解析 16 通道数据 (11 位/通道，紧密打包) */
    raw->channels[0]  = ((buffer[1]       | buffer[2]  << 8) & 0x07FF);
    raw->channels[1]  = ((buffer[2]  >> 3 | buffer[3]  << 5) & 0x07FF);
    raw->channels[2]  = ((buffer[3]  >> 6 | buffer[4]  << 2 | buffer[5] << 10) & 0x07FF);
    raw->channels[3]  = ((buffer[5]  >> 1 | buffer[6]  << 7) & 0x07FF);
    raw->channels[4]  = ((buffer[6]  >> 4 | buffer[7]  << 4) & 0x07FF);
    raw->channels[5]  = ((buffer[7]  >> 7 | buffer[8]  << 1 | buffer[9] << 9) & 0x07FF);
    raw->channels[6]  = ((buffer[9]  >> 2 | buffer[10] << 6) & 0x07FF);
    raw->channels[7]  = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
    raw->channels[8]  = ((buffer[12]      | buffer[13] << 8) & 0x07FF);
    raw->channels[9]  = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
    raw->channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
    raw->channels[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
    raw->channels[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
    raw->channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
    raw->channels[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
    raw->channels[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);
    
    /* 解析标志位 */
    raw->ch17 = (buffer[23] & SBUS_FLAG_CH17) ? true : false;
    raw->ch18 = (buffer[23] & SBUS_FLAG_CH18) ? true : false;
    raw->frame_lost = (buffer[23] & SBUS_FLAG_FRAME_LOST) ? true : false;
    raw->failsafe = (buffer[23] & SBUS_FLAG_FAILSAFE) ? true : false;
    
    return true;
}

/**
 * @brief 将原始值转换为归一化值
 */
void SBUS_Normalize(const SBUS_RawData_t *raw, RC_Data_t *rc)
{
    /* 通道映射 (根据遥控器设置调整):
     * CH1 = Roll
     * CH2 = Pitch
     * CH3 = Throttle
     * CH4 = Yaw
     * CH5 = Aux1 (解锁开关)
     * CH6 = Aux2 (模式开关)
     * CH7 = Aux3
     * CH8 = Aux4
     * CH16 = RSSI (现代接收机: ELRS/Crossfire/FrSky)
     */
    
    /* 油门: 0.0 - 1.0 */
    rc->throttle = SBUS_Map(raw->channels[2], 0.0f, 1.0f);
    
    /* 摇杆: -1.0 - 1.0 */
    rc->roll  = SBUS_Map(raw->channels[0], -1.0f, 1.0f);
    rc->pitch = SBUS_Map(raw->channels[1], -1.0f, 1.0f);
    rc->yaw   = SBUS_Map(raw->channels[3], -1.0f, 1.0f);
    
    /* 辅助通道 */
    rc->aux1 = SBUS_Map(raw->channels[4], -1.0f, 1.0f);
    rc->aux2 = SBUS_Map(raw->channels[5], -1.0f, 1.0f);
    rc->aux3 = SBUS_Map(raw->channels[6], -1.0f, 1.0f);
    rc->aux4 = SBUS_Map(raw->channels[7], -1.0f, 1.0f);
    
    /* 解锁开关 (CH5): 低位=锁定, 高位=解锁 */
    rc->arm_switch = (raw->channels[4] > SBUS_MID_VALUE) ? 1 : 0;
    
    /* 模式开关 (CH6): 三段开关 */
    if (raw->channels[5] < (SBUS_MIN_VALUE + 400)) {
        rc->mode_switch = 0;  /* 自稳模式 */
    } else if (raw->channels[5] > (SBUS_MAX_VALUE - 400)) {
        rc->mode_switch = 2;  /* 手动模式 */
    } else {
        rc->mode_switch = 1;  /* 半自稳模式 */
    }
    
    /* RSSI/Link Quality (通过 CH16 获取)
     * 现代接收机 (ELRS, Crossfire, 新版 FrSky) 将 RSSI 嵌入 SBUS 通道
     * 不需要额外的模拟 RSSI 线 (PC5)
     * 在接收机端配置: RSSI Channel = 16
     */
    uint16_t rssi_raw = raw->channels[SBUS_RSSI_CHANNEL];
    
    /* 将 SBUS 值 (172-1811) 映射到百分比 (0-100) */
    if (rssi_raw < SBUS_RSSI_MIN) rssi_raw = SBUS_RSSI_MIN;
    if (rssi_raw > SBUS_RSSI_MAX) rssi_raw = SBUS_RSSI_MAX;
    
    rc->rssi = (uint8_t)(((uint32_t)(rssi_raw - SBUS_RSSI_MIN) * 100) /
                         (SBUS_RSSI_MAX - SBUS_RSSI_MIN));
    rc->link_quality = rc->rssi;  /* 对于大多数接收机，RSSI = LQ */
    
    /* 设置警告标志 */
    rc->rssi_warning = (rc->rssi < SBUS_LQ_WARNING);
    rc->rssi_critical = (rc->rssi < SBUS_LQ_CRITICAL);
}

/**
 * @brief 检查 SBUS 连接状态
 */
bool SBUS_IsConnected(void)
{
    return g_rc_data.connected;
}

/**
 * @brief 检查失控保护状态
 */
bool SBUS_IsFailsafe(void)
{
    return g_rc_data.failsafe;
}

/**
 * @brief 获取指定通道的原始值
 */
uint16_t SBUS_GetChannel(uint8_t channel)
{
    if (channel >= SBUS_CHANNELS) {
        return SBUS_MID_VALUE;
    }
    return g_sbus_raw.channels[channel];
}

/**
 * @brief 获取指定通道的归一化值
 */
float SBUS_GetChannelNormalized(uint8_t channel)
{
    if (channel >= SBUS_CHANNELS) {
        return 0.0f;
    }
    return SBUS_Map(g_sbus_raw.channels[channel], -1.0f, 1.0f);
}

/**
 * @brief 将 SBUS 值映射到指定范围
 */
float SBUS_Map(uint16_t value, float out_min, float out_max)
{
    /* 限制输入范围 */
    if (value < SBUS_MIN_VALUE) value = SBUS_MIN_VALUE;
    if (value > SBUS_MAX_VALUE) value = SBUS_MAX_VALUE;
    
    /* 线性映射 */
    float normalized = (float)(value - SBUS_MIN_VALUE) / (float)(SBUS_MAX_VALUE - SBUS_MIN_VALUE);
    return out_min + normalized * (out_max - out_min);
}

/**
 * @brief UART 空闲中断回调
 */
void SBUS_UART_IdleCallback(void)
{
    /* 检查是否是 UART4 的空闲中断 */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE)) {
        /* 清除空闲标志 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        
        /* 停止 DMA */
        HAL_UART_DMAStop(&huart4);
        
        /* 计算接收到的字节数 */
        uint32_t rx_len = SBUS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
        
        /* 查找并提取 SBUS 帧 */
        if (rx_len >= SBUS_FRAME_SIZE) {
            /* 查找帧头 */
            for (uint32_t i = 0; i <= rx_len - SBUS_FRAME_SIZE; i++) {
                if (sbus_rx_buffer[i] == SBUS_HEADER) {
                    /* 复制帧数据 */
                    memcpy(sbus_frame_buffer, &sbus_rx_buffer[i], SBUS_FRAME_SIZE);
                    g_sbus_new_frame = true;
                    break;
                }
            }
        }
        
        /* 重新启动 DMA 接收 */
        HAL_UART_Receive_DMA(&huart4, sbus_rx_buffer, SBUS_RX_BUFFER_SIZE);
    }
}

/**
 * @brief DMA 接收完成回调
 */
void SBUS_DMA_RxCpltCallback(void)
{
    /* 缓冲区满，查找帧 */
    SBUS_FindFrame();
}

/**
 * @brief 在缓冲区中查找 SBUS 帧
 */
static void SBUS_FindFrame(void)
{
    /* 查找帧头 */
    for (uint32_t i = 0; i <= SBUS_RX_BUFFER_SIZE - SBUS_FRAME_SIZE; i++) {
        if (sbus_rx_buffer[i] == SBUS_HEADER) {
            /* 复制帧数据 */
            memcpy(sbus_frame_buffer, &sbus_rx_buffer[i], SBUS_FRAME_SIZE);
            g_sbus_new_frame = true;
            break;
        }
    }
}

/**
 * @brief 重置 SBUS 接收状态
 */
void SBUS_Reset(void)
{
    /* 停止 DMA */
    HAL_UART_DMAStop(&huart4);
    
    /* 清空缓冲区 */
    memset(sbus_rx_buffer, 0, sizeof(sbus_rx_buffer));
    memset(sbus_frame_buffer, 0, sizeof(sbus_frame_buffer));
    
    /* 重置状态 */
    g_sbus_new_frame = false;
    g_rc_data.connected = false;
    g_rc_data.failsafe = true;
    g_rc_data.rssi = 0;
    g_rc_data.link_quality = 0;
    g_rc_data.rssi_warning = true;
    g_rc_data.rssi_critical = true;
    
    /* 重新启动 DMA */
    HAL_UART_Receive_DMA(&huart4, sbus_rx_buffer, SBUS_RX_BUFFER_SIZE);
}

/**
 * @brief 获取 RSSI 值 (通过 CH16)
 */
uint8_t SBUS_GetRSSI(void)
{
    return g_rc_data.rssi;
}

/**
 * @brief 获取链路质量
 */
uint8_t SBUS_GetLinkQuality(void)
{
    return g_rc_data.link_quality;
}

/**
 * @brief 检查 RSSI 是否处于警告状态
 */
bool SBUS_IsRSSIWarning(void)
{
    return g_rc_data.rssi_warning;
}

/**
 * @brief 检查 RSSI 是否处于临界状态
 */
bool SBUS_IsRSSICritical(void)
{
    return g_rc_data.rssi_critical;
}