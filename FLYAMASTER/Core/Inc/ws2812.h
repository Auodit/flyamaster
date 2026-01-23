/**
 * @file ws2812.h
 * @brief WS2812 RGB LED 驱动模块 (TIM1_CH1 + DMA)
 * @version 1.1.0
 * @date 2026-01-18
 *
 * @details
 * 使用 TIM1_CH1 (PA8) + DMA 驱动 WS2812 灯珠
 *
 * 硬件变更记录:
 * - v1.0: 原设计使用 TIM1_CH1 (PA8)
 * - v1.0.1: TIM1 被用作 HAL 时基，改用 TIM4_CH4 (PB9)
 * - v1.1.0: HAL 时基改为 TIM4，TIM1_CH1 (PA8) 恢复用于 WS2812
 *
 * 时序要求 (800kHz):
 * - T0H: 0.4us (高电平)
 * - T0L: 0.85us (低电平)
 * - T1H: 0.8us (高电平)
 * - T1L: 0.45us (低电平)
 * - Reset: >50us (低电平)
 *
 * TIM1 配置 (APB2, 168MHz):
 * - 时钟: 168MHz
 * - Prescaler: 0
 * - ARR: 209 (168MHz / 210 = 800kHz)
 * - T0H CCR: 56 (0.33us ≈ 0.4us)
 * - T1H CCR: 112 (0.67us ≈ 0.8us)
 */

#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define WS2812_LED_COUNT        4       /**< LED 数量 */
#define WS2812_BITS_PER_LED     24      /**< 每个 LED 24 位 (GRB) */
#define WS2812_RESET_SLOTS      50      /**< 复位脉冲数量 */

/* PWM 占空比值 (基于 ARR=209, TIM1 时钟 168MHz) */
/* 168MHz / 210 = 800kHz，周期 1.25us */
/* T0H: 0.4us -> 0.4/1.25 * 210 = 67 */
/* T1H: 0.8us -> 0.8/1.25 * 210 = 134 */
#define WS2812_PWM_HI           134     /**< 逻辑 1 的 CCR 值 (~0.8us) */
#define WS2812_PWM_LO           67      /**< 逻辑 0 的 CCR 值 (~0.4us) */

/* DMA 缓冲区大小 */
#define WS2812_DMA_BUFFER_SIZE  (WS2812_LED_COUNT * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief RGB 颜色结构体
 */
typedef struct {
    uint8_t r;      /**< 红色 (0-255) */
    uint8_t g;      /**< 绿色 (0-255) */
    uint8_t b;      /**< 蓝色 (0-255) */
} WS2812_Color_t;

/**
 * @brief 预定义颜色
 */
typedef enum {
    WS2812_COLOR_OFF = 0,
    WS2812_COLOR_RED,
    WS2812_COLOR_GREEN,
    WS2812_COLOR_BLUE,
    WS2812_COLOR_WHITE,
    WS2812_COLOR_YELLOW,
    WS2812_COLOR_CYAN,
    WS2812_COLOR_MAGENTA,
    WS2812_COLOR_ORANGE,
    WS2812_COLOR_PURPLE
} WS2812_PresetColor_t;

/**
 * @brief LED 模式枚举
 */
typedef enum {
    WS2812_MODE_STATIC = 0,     /**< 静态颜色 */
    WS2812_MODE_BLINK,          /**< 闪烁 */
    WS2812_MODE_BREATHE,        /**< 呼吸灯 */
    WS2812_MODE_RAINBOW,        /**< 彩虹渐变 */
    WS2812_MODE_CHASE           /**< 追逐效果 */
} WS2812_Mode_t;

/* Exported variables --------------------------------------------------------*/
extern WS2812_Color_t g_ws2812_leds[WS2812_LED_COUNT];  /**< LED 颜色数组 */
extern volatile bool g_ws2812_busy;                     /**< DMA 传输中标志 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 WS2812 驱动
 */
void WS2812_Init(void);

/**
 * @brief 设置单个 LED 颜色
 * @param index LED 索引 (0 - WS2812_LED_COUNT-1)
 * @param r 红色 (0-255)
 * @param g 绿色 (0-255)
 * @param b 蓝色 (0-255)
 */
void WS2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置单个 LED 颜色 (结构体)
 * @param index LED 索引
 * @param color 颜色结构体
 */
void WS2812_SetColorStruct(uint8_t index, WS2812_Color_t color);

/**
 * @brief 设置所有 LED 为同一颜色
 * @param r 红色
 * @param g 绿色
 * @param b 蓝色
 */
void WS2812_SetAllColor(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 使用预设颜色
 * @param index LED 索引
 * @param preset 预设颜色枚举
 */
void WS2812_SetPresetColor(uint8_t index, WS2812_PresetColor_t preset);

/**
 * @brief 关闭所有 LED
 */
void WS2812_Clear(void);

/**
 * @brief 更新 LED 显示 (启动 DMA 传输)
 */
void WS2812_Update(void);

/**
 * @brief 设置 LED 亮度
 * @param brightness 亮度 (0-255)
 */
void WS2812_SetBrightness(uint8_t brightness);

/**
 * @brief 设置 LED 模式
 * @param mode 显示模式
 */
void WS2812_SetMode(WS2812_Mode_t mode);

/**
 * @brief 更新动画效果 (在定时器中调用)
 */
void WS2812_AnimationUpdate(void);

/**
 * @brief HSV 转 RGB
 * @param h 色相 (0-359)
 * @param s 饱和度 (0-255)
 * @param v 明度 (0-255)
 * @return WS2812_Color_t RGB 颜色
 */
WS2812_Color_t WS2812_HSVtoRGB(uint16_t h, uint8_t s, uint8_t v);

/**
 * @brief DMA 传输完成回调
 * @note 在 HAL_TIM_PWM_PulseFinishedCallback 中调用
 */
void WS2812_DMA_CompleteCallback(void);

/**
 * @brief 获取预设颜色的 RGB 值
 * @param preset 预设颜色
 * @return WS2812_Color_t RGB 颜色
 */
WS2812_Color_t WS2812_GetPresetRGB(WS2812_PresetColor_t preset);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */