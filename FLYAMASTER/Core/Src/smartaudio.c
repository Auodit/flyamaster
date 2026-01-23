/**
 * @file smartaudio.c
 * @brief VTX SmartAudio 协议驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "smartaudio.h"
#include "usart.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define SA_BAUDRATE             4800    /**< SmartAudio 波特率 */
#define SA_TX_BUFFER_SIZE       16      /**< 发送缓冲区大小 */
#define SA_RX_BUFFER_SIZE       32      /**< 接收缓冲区大小 */

/* Private variables ---------------------------------------------------------*/
SA_Data_t g_smartaudio_data = {0};

/* 发送缓冲区 */
static uint8_t sa_tx_buffer[SA_TX_BUFFER_SIZE];

/* 频率表 (5 频段 x 8 频道) - 单位 MHz */
const uint16_t sa_frequency_table[5][8] = {
    /* Band A */
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725},
    /* Band B */
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},
    /* Band E */
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945},
    /* Fatshark */
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880},
    /* Raceband */
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}
};

/* CRC8 查找表 (多项式 0xD5) */
static const uint8_t crc8_table[256] = {
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

/* Private function prototypes -----------------------------------------------*/
static void SA_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len);
static bool SA_WaitResponse(uint32_t timeout);
static void SA_ParseResponse(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 计算 CRC8
 */
uint8_t SA_CalculateCRC(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    
    return crc;
}

/**
 * @brief 初始化 SmartAudio
 */
bool SA_Init(void)
{
    /* 清空数据结构 */
    memset(&g_smartaudio_data, 0, sizeof(g_smartaudio_data));
    
    /* 默认设置 */
    g_smartaudio_data.settings.power = SA_POWER_25MW;
    g_smartaudio_data.settings.band = SA_BAND_R;
    g_smartaudio_data.settings.channel = 0;
    g_smartaudio_data.settings.frequency = sa_frequency_table[SA_BAND_R][0];
    
    g_smartaudio_data.status = SA_STATUS_UNKNOWN;
    g_smartaudio_data.initialized = true;
    
    /* 尝试获取 VTX 设置 */
    SA_GetSettings();
    
    return true;
}

/**
 * @brief 发送 SmartAudio 命令
 */
static void SA_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t idx = 0;
    
    /* 构建帧 */
    sa_tx_buffer[idx++] = 0x00;         /* 前导零 (半双工需要) */
    sa_tx_buffer[idx++] = SA_SYNC_BYTE; /* 同步字节 */
    sa_tx_buffer[idx++] = SA_HEADER_BYTE; /* 头字节 */
    sa_tx_buffer[idx++] = cmd;          /* 命令 */
    sa_tx_buffer[idx++] = len;          /* 数据长度 */
    
    /* 复制数据 */
    if (data != NULL && len > 0) {
        memcpy(&sa_tx_buffer[idx], data, len);
        idx += len;
    }
    
    /* 计算 CRC (从 cmd 开始) */
    sa_tx_buffer[idx++] = SA_CalculateCRC(&sa_tx_buffer[3], 2 + len);
    
    /* 发送数据 */
    HAL_UART_Transmit(&huart4, sa_tx_buffer, idx, 100);
    
    /* 清空接收缓冲区 */
    g_smartaudio_data.rx_index = 0;
    g_smartaudio_data.rx_complete = false;
    
    g_smartaudio_data.status = SA_STATUS_BUSY;
}

/**
 * @brief 等待响应
 */
static bool SA_WaitResponse(uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    
    while (!g_smartaudio_data.rx_complete) {
        if ((HAL_GetTick() - start) > timeout) {
            g_smartaudio_data.status = SA_STATUS_TIMEOUT;
            return false;
        }
    }
    
    return true;
}

/**
 * @brief 处理接收字节
 */
void SA_ProcessRxByte(uint8_t data)
{
    if (g_smartaudio_data.rx_index < SA_RX_BUFFER_SIZE) {
        g_smartaudio_data.rx_buffer[g_smartaudio_data.rx_index++] = data;
        
        /* 检查是否接收完成 */
        if (g_smartaudio_data.rx_index >= 5) {
            uint8_t expected_len = g_smartaudio_data.rx_buffer[4] + 6;
            if (g_smartaudio_data.rx_index >= expected_len) {
                g_smartaudio_data.rx_complete = true;
            }
        }
    }
}

/**
 * @brief 解析响应
 */
static void SA_ParseResponse(void)
{
    uint8_t *buf = g_smartaudio_data.rx_buffer;
    
    /* 验证帧头 */
    if (buf[0] != SA_SYNC_BYTE || buf[1] != SA_HEADER_BYTE) {
        g_smartaudio_data.status = SA_STATUS_ERROR;
        return;
    }
    
    uint8_t cmd = buf[2];
    uint8_t len = buf[3];
    
    /* 验证 CRC */
    uint8_t crc = SA_CalculateCRC(&buf[2], 2 + len);
    if (crc != buf[4 + len]) {
        g_smartaudio_data.status = SA_STATUS_ERROR;
        return;
    }
    
    /* 解析命令响应 */
    switch (cmd) {
        case SA_RESP_GET_SETTINGS:
            if (len >= 4) {
                g_smartaudio_data.settings.channel = buf[4];
                g_smartaudio_data.settings.power = buf[5];
                g_smartaudio_data.settings.mode = buf[6];
                g_smartaudio_data.settings.frequency = (buf[7] << 8) | buf[8];
                
                /* 解析模式标志 */
                g_smartaudio_data.settings.pit_mode = (buf[6] & SA_MODE_PIT) != 0;
                g_smartaudio_data.settings.unlocked = (buf[6] & SA_MODE_UNLOCKED) == 0;
                
                /* 计算频段 */
                SA_FrequencyToChannel(g_smartaudio_data.settings.frequency,
                                     &g_smartaudio_data.settings.band,
                                     NULL);
                
                g_smartaudio_data.connected = true;
            }
            break;
            
        case SA_RESP_SET_POWER:
        case SA_RESP_SET_CHANNEL:
        case SA_RESP_SET_FREQ:
        case SA_RESP_SET_MODE:
            /* 设置成功 */
            break;
            
        default:
            break;
    }
    
    g_smartaudio_data.status = SA_STATUS_READY;
    g_smartaudio_data.last_update = HAL_GetTick();
}

/**
 * @brief 处理响应
 */
void SA_ProcessResponse(void)
{
    if (g_smartaudio_data.rx_complete) {
        SA_ParseResponse();
        g_smartaudio_data.rx_complete = false;
        g_smartaudio_data.rx_index = 0;
    }
}

/**
 * @brief 获取 VTX 设置
 */
SA_Status_t SA_GetSettings(void)
{
    SA_SendCommand(SA_CMD_GET_SETTINGS, NULL, 0);
    
    if (SA_WaitResponse(SA_TIMEOUT_MS)) {
        SA_ProcessResponse();
        return g_smartaudio_data.status;
    }
    
    return SA_STATUS_TIMEOUT;
}

/**
 * @brief 设置功率等级
 */
SA_Status_t SA_SetPower(uint8_t power)
{
    if (power > 3) {
        return SA_STATUS_ERROR;
    }
    
    uint8_t data[1] = {power};
    SA_SendCommand(SA_CMD_SET_POWER, data, 1);
    
    if (SA_WaitResponse(SA_TIMEOUT_MS)) {
        SA_ProcessResponse();
        if (g_smartaudio_data.status == SA_STATUS_READY) {
            g_smartaudio_data.settings.power = power;
        }
        return g_smartaudio_data.status;
    }
    
    return SA_STATUS_TIMEOUT;
}

/**
 * @brief 设置频道
 */
SA_Status_t SA_SetChannel(uint8_t channel)
{
    if (channel > 39) {
        return SA_STATUS_ERROR;
    }
    
    uint8_t data[1] = {channel};
    SA_SendCommand(SA_CMD_SET_CHANNEL, data, 1);
    
    if (SA_WaitResponse(SA_TIMEOUT_MS)) {
        SA_ProcessResponse();
        if (g_smartaudio_data.status == SA_STATUS_READY) {
            g_smartaudio_data.settings.channel = channel;
            /* 更新频率 */
            uint8_t band = channel / 8;
            uint8_t ch = channel % 8;
            g_smartaudio_data.settings.frequency = sa_frequency_table[band][ch];
        }
        return g_smartaudio_data.status;
    }
    
    return SA_STATUS_TIMEOUT;
}

/**
 * @brief 设置频率
 */
SA_Status_t SA_SetFrequency(uint16_t frequency)
{
    if (frequency < 5600 || frequency > 6000) {
        return SA_STATUS_ERROR;
    }
    
    uint8_t data[2] = {(frequency >> 8) & 0xFF, frequency & 0xFF};
    SA_SendCommand(SA_CMD_SET_FREQ, data, 2);
    
    if (SA_WaitResponse(SA_TIMEOUT_MS)) {
        SA_ProcessResponse();
        if (g_smartaudio_data.status == SA_STATUS_READY) {
            g_smartaudio_data.settings.frequency = frequency;
        }
        return g_smartaudio_data.status;
    }
    
    return SA_STATUS_TIMEOUT;
}

/**
 * @brief 设置 PIT 模式
 */
SA_Status_t SA_SetPitMode(bool enable)
{
    uint8_t mode = enable ? SA_MODE_PIT : SA_MODE_UNLOCKED;
    uint8_t data[1] = {mode};
    
    SA_SendCommand(SA_CMD_SET_MODE, data, 1);
    
    if (SA_WaitResponse(SA_TIMEOUT_MS)) {
        SA_ProcessResponse();
        if (g_smartaudio_data.status == SA_STATUS_READY) {
            g_smartaudio_data.settings.pit_mode = enable;
        }
        return g_smartaudio_data.status;
    }
    
    return SA_STATUS_TIMEOUT;
}

/**
 * @brief 设置频段和频道
 */
SA_Status_t SA_SetBandChannel(uint8_t band, uint8_t channel)
{
    if (band > 4 || channel > 7) {
        return SA_STATUS_ERROR;
    }
    
    uint8_t combined_channel = band * 8 + channel;
    return SA_SetChannel(combined_channel);
}

/**
 * @brief 周期性更新
 */
void SA_Update(void)
{
    /* 处理待处理的响应 */
    SA_ProcessResponse();
    
    /* 定期刷新设置 */
    static uint32_t last_refresh = 0;
    if ((HAL_GetTick() - last_refresh) > 5000) {
        SA_GetSettings();
        last_refresh = HAL_GetTick();
    }
    
    /* 检查连接超时 */
    if ((HAL_GetTick() - g_smartaudio_data.last_update) > 10000) {
        g_smartaudio_data.connected = false;
    }
}

/**
 * @brief 检查连接状态
 */
bool SA_IsConnected(void)
{
    return g_smartaudio_data.connected;
}

/**
 * @brief 获取当前频率
 */
uint16_t SA_GetFrequency(void)
{
    return g_smartaudio_data.settings.frequency;
}

/**
 * @brief 获取当前功率
 */
uint8_t SA_GetPower(void)
{
    return g_smartaudio_data.settings.power;
}

/**
 * @brief 频道转频率
 */
uint16_t SA_ChannelToFrequency(uint8_t band, uint8_t channel)
{
    if (band > 4 || channel > 7) {
        return 0;
    }
    return sa_frequency_table[band][channel];
}

/**
 * @brief 频率转频道
 */
bool SA_FrequencyToChannel(uint16_t frequency, uint8_t *band, uint8_t *channel)
{
    for (uint8_t b = 0; b < 5; b++) {
        for (uint8_t c = 0; c < 8; c++) {
            if (sa_frequency_table[b][c] == frequency) {
                if (band) *band = b;
                if (channel) *channel = c;
                return true;
            }
        }
    }
    
    /* 找最接近的频率 */
    uint16_t min_diff = 0xFFFF;
    uint8_t best_band = 0, best_channel = 0;
    
    for (uint8_t b = 0; b < 5; b++) {
        for (uint8_t c = 0; c < 8; c++) {
            uint16_t diff = (frequency > sa_frequency_table[b][c]) ?
                           (frequency - sa_frequency_table[b][c]) :
                           (sa_frequency_table[b][c] - frequency);
            if (diff < min_diff) {
                min_diff = diff;
                best_band = b;
                best_channel = c;
            }
        }
    }
    
    if (band) *band = best_band;
    if (channel) *channel = best_channel;
    
    return false;
}