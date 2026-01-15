#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H

#include "main.h"

// 光流缩放系数 (需根据模块校准)
// 将像素位移转换为物理速度: V = flow * height * FLOW_SCALE
#define FLOW_SCALE_FACTOR   0.01f
#define FLOW_MIN_HEIGHT     0.05f   // 最小有效高度 5cm
#define FLOW_MAX_HEIGHT     3.0f    // 最大有效高度 3m
#define FLOW_MIN_QUALITY    30      // 最小有效质量

// 光流数据结构体 (兼容优象/匿名协议)
typedef struct {
    int16_t flow_x;         // X轴像素位移
    int16_t flow_y;         // Y轴像素位移
    uint16_t distance;      // 激光测距高度 (mm)
    uint8_t quality;        // 质量 (0~255)
    uint8_t valid;          // 数据有效标志
    uint32_t last_update;   // 最后更新时间
} OpticalFlow_t;

extern OpticalFlow_t Flow_Data;

// 初始化光流模块
void OpticalFlow_Init(UART_HandleTypeDef *huart);

// 解析光流数据帧
void OpticalFlow_Parse(uint8_t *buf, uint16_t len);

// 获取带高度补偿的物理速度 (m/s)
// 返回值: 1=数据有效, 0=数据无效
uint8_t OpticalFlow_GetVelocity(float *vx, float *vy);

// 获取高度 (m)
float OpticalFlow_GetHeight(void);

#endif
