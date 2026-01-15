#include "optical_flow.h"

OpticalFlow_t Flow_Data;
uint8_t flow_rx_buf[64]; // 接收缓冲区 (供外部 DMA 使用)

/**
 * @brief 初始化光流模块
 */
void OpticalFlow_Init(UART_HandleTypeDef *huart) {
    // 清零光流数据
    Flow_Data.flow_x = 0;
    Flow_Data.flow_y = 0;
    Flow_Data.distance = 0;
    Flow_Data.quality = 0;
    Flow_Data.valid = 0;
    Flow_Data.last_update = 0;
    
    // 开启空闲中断接收 (IDLE Interrupt)
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, flow_rx_buf, 64);
}

/**
 * @brief 解析光流数据 (优象 LC302 协议)
 * 帧格式: 0xFE 0x04 [flow_x:2] [flow_y:2] [distance:2] [quality:1] [checksum:1]
 * 总长度: 10 字节
 */
void OpticalFlow_Parse(uint8_t *buf, uint16_t len) {
    // 协议解析逻辑 (LC302 协议)
    for (int i = 0; i < len - 9; i++) {
        if (buf[i] == 0xFE && buf[i+1] == 0x04) {
            // 解析光流像素位移
            Flow_Data.flow_x = (int16_t)(buf[i+2] | (buf[i+3] << 8));
            Flow_Data.flow_y = (int16_t)(buf[i+4] | (buf[i+5] << 8));
            
            // 解析激光测距高度 (mm)
            Flow_Data.distance = (uint16_t)(buf[i+6] | (buf[i+7] << 8));
            
            // 解析质量
            Flow_Data.quality = buf[i+8];
            
            // 校验和验证 (可选)
            // uint8_t checksum = buf[i+9];
            
            Flow_Data.valid = 1;
            Flow_Data.last_update = HAL_GetTick();
            return;
        }
    }
}

/**
 * @brief 获取带高度补偿的物理速度
 * @param vx 输出X轴速度 (m/s)
 * @param vy 输出Y轴速度 (m/s)
 * @return 1=数据有效, 0=数据无效
 *
 * 物理速度计算公式: V = Δpixel × height × K
 * 其中 height 是激光测距高度，K 是缩放系数
 */
uint8_t OpticalFlow_GetVelocity(float *vx, float *vy) {
    // Risk #048 修复：添加指针空检查
    if (vx == NULL || vy == NULL) return 0;
    
    // 检查数据有效性
    if (!Flow_Data.valid) {
        *vx = 0.0f;
        *vy = 0.0f;
        return 0;
    }
    
    // 检查数据时效性 (100ms 超时)
    if (HAL_GetTick() - Flow_Data.last_update > 100) {
        *vx = 0.0f;
        *vy = 0.0f;
        Flow_Data.valid = 0;
        return 0;
    }
    
    // 检查质量阈值
    if (Flow_Data.quality < FLOW_MIN_QUALITY) {
        *vx = 0.0f;
        *vy = 0.0f;
        return 0;
    }
    
    // 获取高度 (转换为米)
    float height_m = Flow_Data.distance / 1000.0f;
    
    // 限制高度范围
    if (height_m < FLOW_MIN_HEIGHT) {
        height_m = FLOW_MIN_HEIGHT;  // 最小 5cm
    } else if (height_m > FLOW_MAX_HEIGHT) {
        height_m = FLOW_MAX_HEIGHT;  // 最大 3m
    }
    
    // 计算物理速度 (带高度补偿)
    // V = flow × height × scale_factor
    *vx = Flow_Data.flow_x * height_m * FLOW_SCALE_FACTOR;
    *vy = Flow_Data.flow_y * height_m * FLOW_SCALE_FACTOR;
    
    return 1;
}

/**
 * @brief 获取激光测距高度
 * @return 高度 (m)，无效时返回 0
 */
float OpticalFlow_GetHeight(void) {
    if (!Flow_Data.valid || Flow_Data.distance == 0) {
        return 0.0f;
    }
    return Flow_Data.distance / 1000.0f;
}
