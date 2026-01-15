/**
 * @file altitude.h
 * @brief 高度融合与定高控制模块
 * @details 使用互补滤波融合激光高度和加速度计
 * @author AI Assistant
 * @date 2026-01-14
 */

#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#include <stdint.h>
#include "mahony.h"

/*==============================================================================
 * 常量定义
 *============================================================================*/
#define ALT_CF_GAIN         1.0f      // 互补滤波增益 K
#define ALT_ACC_SCALE       9.81f     // 加速度转 m/s^2 (1g = 9.81m/s^2)
#define ALT_MIN_HEIGHT      0.05f     // 最小高度 (m) - 防止除零
#define ALT_MAX_HEIGHT      3.0f      // 最大可信激光高度 (m)
#define ALT_LASER_TIMEOUT   200       // 激光数据超时 (ms)

// ✅ 智能数据源切换阈值 (2026-01-14)
#define ALT_LASER_ONLY_MAX  1.8f      // < 1.8m: 100% 激光
#define ALT_BARO_ONLY_MIN   2.2f      // > 2.2m: 100% 气压
// 1.8m ~ 2.2m: 线性混合过渡区

/*==============================================================================
 * 数据结构
 *============================================================================*/
typedef struct {
    // 状态估计
    float height;           // 估计高度 (m)
    float velocity;         // 估计垂直速度 (m/s) 向上为正
    
    // 传感器输入
    float acc_z_earth;      // 地球坐标系 Z 轴加速度 (去重力, m/s^2)
    float height_laser;     // 激光测量高度 (m)
    float height_baro;      // 气压计高度 (m) ✅ 新增
    uint32_t laser_last_update;  // 激光最后更新时间
    uint32_t baro_last_update;   // 气压计最后更新时间 ✅ 新增
    
    // 滤波器参数
    float cf_gain;          // 互补滤波增益
    
    // 数据有效性
    uint8_t laser_valid;    // 激光数据是否有效
    uint8_t baro_valid;     // 气压计数据是否有效 ✅ 新增
    uint8_t fusion_ready;   // 融合是否就绪
} Altitude_TypeDef;

/*==============================================================================
 * 全局变量
 *============================================================================*/
extern Altitude_TypeDef Alt_Data;

/*==============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief 初始化高度融合模块
 * @param alt 高度数据结构体指针
 */
void Altitude_Init(Altitude_TypeDef *alt);

/**
 * @brief 将机体坐标系加速度转换到地球坐标系 Z 轴
 * @param mahony Mahony 姿态解算结构体指针 (提供四元数)
 * @param ax 机体 X 轴加速度 (g)
 * @param ay 机体 Y 轴加速度 (g)
 * @param az 机体 Z 轴加速度 (g)
 * @return 地球坐标系 Z 轴加速度 (去除重力后, m/s^2)
 */
float Altitude_TransformAccZ(Mahony_TypeDef *mahony, float ax, float ay, float az);

/**
 * @brief 更新激光高度测量值
 * @param alt 高度数据结构体指针
 * @param height_m 激光测量高度 (m)
 */
void Altitude_UpdateLaser(Altitude_TypeDef *alt, float height_m);

/**
 * @brief 更新气压计高度测量值
 * @param alt 高度数据结构体指针
 * @param height_m 气压计测量高度 (m)
 */
void Altitude_UpdateBarometer(Altitude_TypeDef *alt, float height_m);

/**
 * @brief 互补滤波融合更新
 * @param alt 高度数据结构体指针
 * @param acc_z_earth 地球坐标系 Z 轴加速度 (m/s^2)
 * @param dt 时间间隔 (s)
 */
void Altitude_FusionUpdate(Altitude_TypeDef *alt, float acc_z_earth, float dt);

/**
 * @brief 获取当前估计高度
 * @param alt 高度数据结构体指针
 * @return 估计高度 (m)
 */
float Altitude_GetHeight(Altitude_TypeDef *alt);

/**
 * @brief 获取当前估计垂直速度
 * @param alt 高度数据结构体指针
 * @return 估计垂直速度 (m/s)
 */
float Altitude_GetVelocity(Altitude_TypeDef *alt);

/**
 * @brief 检查高度融合是否就绪
 * @param alt 高度数据结构体指针
 * @return 1: 就绪, 0: 未就绪
 */
uint8_t Altitude_IsReady(Altitude_TypeDef *alt);

#endif /* __ALTITUDE_H */
