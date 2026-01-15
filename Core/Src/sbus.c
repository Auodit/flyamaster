#include "sbus.h"
#include <string.h>

SBUS_Data_t SBUS_Data;
uint8_t SBUS_RxBuffer[SBUS_FRAME_SIZE];
static uint32_t last_sbus_time = 0;

/**
 * @brief 初始化 SBUS 接收
 */
void SBUS_Init(UART_HandleTypeDef *huart) {
    // 开启 DMA 循环接收
    HAL_UART_Receive_DMA(huart, SBUS_RxBuffer, SBUS_FRAME_SIZE);
}

/**
 * @brief 解析 SBUS 数据帧
 * @param buffer 接收到的 25 字节数据
 */
void SBUS_Parse(uint8_t *buffer) {
    // Risk #053 修复：添加指针空检查
    if (buffer == NULL) return;
    
    if (buffer[0] == 0x0F && buffer[24] == 0x00) { // 帧头帧尾校验
        // 解析 16 个通道 (11 bits per channel)
        SBUS_Data.channels[0]  = ((buffer[1]       | buffer[2] << 8)                    & 0x07FF);
        SBUS_Data.channels[1]  = ((buffer[2] >> 3  | buffer[3] << 5)                    & 0x07FF);
        SBUS_Data.channels[2]  = ((buffer[3] >> 6  | buffer[4] << 2 | buffer[5] << 10)  & 0x07FF);
        SBUS_Data.channels[3]  = ((buffer[5] >> 1  | buffer[6] << 7)                    & 0x07FF);
        SBUS_Data.channels[4]  = ((buffer[6] >> 4  | buffer[7] << 4)                    & 0x07FF);
        SBUS_Data.channels[5]  = ((buffer[7] >> 7  | buffer[8] << 1 | buffer[9] << 9)   & 0x07FF);
        SBUS_Data.channels[6]  = ((buffer[9] >> 2  | buffer[10] << 6)                   & 0x07FF);
        SBUS_Data.channels[7]  = ((buffer[10] >> 5 | buffer[11] << 3)                   & 0x07FF);
        SBUS_Data.channels[8]  = ((buffer[12]      | buffer[13] << 8)                   & 0x07FF);
        SBUS_Data.channels[9]  = ((buffer[13] >> 3 | buffer[14] << 5)                   & 0x07FF);
        SBUS_Data.channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
        SBUS_Data.channels[11] = ((buffer[16] >> 1 | buffer[17] << 7)                   & 0x07FF);
        SBUS_Data.channels[12] = ((buffer[17] >> 4 | buffer[18] << 4)                   & 0x07FF);
        SBUS_Data.channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9)  & 0x07FF);
        SBUS_Data.channels[14] = ((buffer[20] >> 2 | buffer[21] << 6)                   & 0x07FF);
        SBUS_Data.channels[15] = ((buffer[21] >> 5 | buffer[22] << 3)                   & 0x07FF);

        // 解析标志位
        SBUS_Data.frame_lost = buffer[23] & 0x04;
        SBUS_Data.failsafe = buffer[23] & 0x08;

        // 更新时间戳
        last_sbus_time = HAL_GetTick();
    }
}

/**
 * @brief 检查 SBUS 是否连接 (超时检测)
 * @return 1: 连接正常, 0: 信号丢失
 */
uint8_t SBUS_IsConnected(void) {
    if (HAL_GetTick() - last_sbus_time > 100) { // 100ms 超时
        return 0;
    }
    return 1;
}

/**
 * @brief 检查解锁手势 (油门最低 + 方向舵最右)
 *
 * 解锁条件:
 * 1. 油门通道值 < ARM_THRESHOLD_LOW (低于最低位置)
 * 2. 偏航通道值 > ARM_THRESHOLD_HIGH (方向舵打到最右)
 *
 * @return 1: 解锁手势有效, 0: 无效
 */
uint8_t SBUS_CheckArmGesture(void) {
    uint16_t throttle = SBUS_Data.channels[CH_THROTTLE];
    uint16_t yaw = SBUS_Data.channels[CH_YAW];
    
    // 油门最低 + 方向舵最右 = 解锁
    if (throttle < (SBUS_MIN_VALUE + ARM_THRESHOLD_LOW) &&
        yaw > ARM_THRESHOLD_HIGH) {
        return 1;
    }
    return 0;
}

/**
 * @brief 检查上锁手势 (油门最低 + 方向舵最左)
 *
 * 上锁条件:
 * 1. 油门通道值 < ARM_THRESHOLD_LOW (低于最低位置)
 * 2. 偏航通道值 < (SBUS_MIN_VALUE + 200) (方向舵打到最左)
 *
 * @return 1: 上锁手势有效, 0: 无效
 */
uint8_t SBUS_CheckDisarmGesture(void) {
    uint16_t throttle = SBUS_Data.channels[CH_THROTTLE];
    uint16_t yaw = SBUS_Data.channels[CH_YAW];
    
    // 油门最低 + 方向舵最左 = 上锁
    if (throttle < (SBUS_MIN_VALUE + ARM_THRESHOLD_LOW) &&
        yaw < (SBUS_MIN_VALUE + 200)) {
        return 1;
    }
    return 0;
}

/**
 * @brief 将通道值转换为归一化值
 * @param channel 通道索引
 * @param min_out 输出最小值
 * @param max_out 输出最大值
 * @return 归一化后的值
 */
float SBUS_GetNormalized(uint8_t channel, float min_out, float max_out) {
    if (channel >= SBUS_CHANNEL_NUMBER) return 0.0f;
    
    uint16_t value = SBUS_Data.channels[channel];
    
    // 限幅
    if (value < SBUS_MIN_VALUE) value = SBUS_MIN_VALUE;
    if (value > SBUS_MAX_VALUE) value = SBUS_MAX_VALUE;
    
    // 线性映射: [SBUS_MIN, SBUS_MAX] -> [min_out, max_out]
    float normalized = (float)(value - SBUS_MIN_VALUE) / (float)(SBUS_MAX_VALUE - SBUS_MIN_VALUE);
    return min_out + normalized * (max_out - min_out);
}

/**
 * @brief 获取油门归一化值 (0.0 ~ 1.0)
 */
float SBUS_GetThrottle(void) {
    return SBUS_GetNormalized(CH_THROTTLE, 0.0f, 1.0f);
}

/**
 * @brief 获取横滚归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetRoll(void) {
    return SBUS_GetNormalized(CH_ROLL, -1.0f, 1.0f);
}

/**
 * @brief 获取俯仰归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetPitch(void) {
    return SBUS_GetNormalized(CH_PITCH, -1.0f, 1.0f);
}

/**
 * @brief 获取偏航归一化值 (-1.0 ~ 1.0)
 */
float SBUS_GetYaw(void) {
    return SBUS_GetNormalized(CH_YAW, -1.0f, 1.0f);
}
