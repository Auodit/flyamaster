/**
 * @file ws2812.c
 * @brief WS2812 RGB LED 驱动实现
 * @version 1.1.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include "tim.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
/* 硬件变更: HAL 时基改为 TIM4，TIM1_CH1 (PA8) 恢复用于 WS2812 */
#define WS2812_TIM          htim1
#define WS2812_TIM_CHANNEL  TIM_CHANNEL_1

/* Exported variables --------------------------------------------------------*/
WS2812_Color_t g_ws2812_leds[WS2812_LED_COUNT] = {0};
volatile bool g_ws2812_busy = false;

/* Private variables ---------------------------------------------------------*/
static uint16_t ws2812_dma_buffer[WS2812_DMA_BUFFER_SIZE];
static uint8_t ws2812_brightness = 255;
static WS2812_Mode_t ws2812_mode = WS2812_MODE_STATIC;
static uint16_t ws2812_animation_step = 0;

/* Private function prototypes -----------------------------------------------*/
static void WS2812_FillBuffer(void);
static uint8_t WS2812_ApplyBrightness(uint8_t value);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 WS2812 驱动
 */
void WS2812_Init(void)
{
    /* 清空 LED 数组 */
    memset(g_ws2812_leds, 0, sizeof(g_ws2812_leds));
    
    /* 清空 DMA 缓冲区 */
    memset(ws2812_dma_buffer, 0, sizeof(ws2812_dma_buffer));
    
    g_ws2812_busy = false;
    ws2812_brightness = 255;
    ws2812_mode = WS2812_MODE_STATIC;
    
    /* 初始化完成后关闭所有 LED */
    WS2812_Clear();
    WS2812_Update();
}

/**
 * @brief 设置单个 LED 颜色
 */
void WS2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT) return;
    
    g_ws2812_leds[index].r = r;
    g_ws2812_leds[index].g = g;
    g_ws2812_leds[index].b = b;
}

/**
 * @brief 设置单个 LED 颜色 (结构体)
 */
void WS2812_SetColorStruct(uint8_t index, WS2812_Color_t color)
{
    if (index >= WS2812_LED_COUNT) return;
    g_ws2812_leds[index] = color;
}

/**
 * @brief 设置所有 LED 为同一颜色
 */
void WS2812_SetAllColor(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < WS2812_LED_COUNT; i++) {
        g_ws2812_leds[i].r = r;
        g_ws2812_leds[i].g = g;
        g_ws2812_leds[i].b = b;
    }
}

/**
 * @brief 使用预设颜色
 */
void WS2812_SetPresetColor(uint8_t index, WS2812_PresetColor_t preset)
{
    WS2812_Color_t color = WS2812_GetPresetRGB(preset);
    WS2812_SetColorStruct(index, color);
}

/**
 * @brief 关闭所有 LED
 */
void WS2812_Clear(void)
{
    memset(g_ws2812_leds, 0, sizeof(g_ws2812_leds));
}

/**
 * @brief 更新 LED 显示
 */
void WS2812_Update(void)
{
    /* 等待上次传输完成 */
    if (g_ws2812_busy) return;
    
    /* 填充 DMA 缓冲区 */
    WS2812_FillBuffer();
    
    /* 标记忙碌 */
    g_ws2812_busy = true;
    
    /* 启动 DMA 传输 */
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_CHANNEL, 
                          (uint32_t *)ws2812_dma_buffer, 
                          WS2812_DMA_BUFFER_SIZE);
}

/**
 * @brief 设置 LED 亮度
 */
void WS2812_SetBrightness(uint8_t brightness)
{
    ws2812_brightness = brightness;
}

/**
 * @brief 设置 LED 模式
 */
void WS2812_SetMode(WS2812_Mode_t mode)
{
    ws2812_mode = mode;
    ws2812_animation_step = 0;
}

/**
 * @brief 更新动画效果
 */
void WS2812_AnimationUpdate(void)
{
    switch (ws2812_mode) {
        case WS2812_MODE_STATIC:
            /* 静态模式，不需要更新 */
            break;
            
        case WS2812_MODE_BLINK:
            /* 闪烁模式 */
            if (ws2812_animation_step % 2 == 0) {
                WS2812_SetAllColor(255, 0, 0);
            } else {
                WS2812_Clear();
            }
            ws2812_animation_step++;
            break;
            
        case WS2812_MODE_BREATHE:
            /* 呼吸灯模式 */
            {
                uint8_t brightness;
                if (ws2812_animation_step < 128) {
                    brightness = ws2812_animation_step * 2;
                } else {
                    brightness = (255 - ws2812_animation_step) * 2;
                }
                WS2812_SetBrightness(brightness);
                ws2812_animation_step = (ws2812_animation_step + 1) % 256;
            }
            break;
            
        case WS2812_MODE_RAINBOW:
            /* 彩虹渐变模式 */
            for (uint8_t i = 0; i < WS2812_LED_COUNT; i++) {
                uint16_t hue = (ws2812_animation_step + i * 360 / WS2812_LED_COUNT) % 360;
                g_ws2812_leds[i] = WS2812_HSVtoRGB(hue, 255, 255);
            }
            ws2812_animation_step = (ws2812_animation_step + 1) % 360;
            break;
            
        case WS2812_MODE_CHASE:
            /* 追逐效果 */
            WS2812_Clear();
            WS2812_SetColor(ws2812_animation_step % WS2812_LED_COUNT, 0, 255, 0);
            ws2812_animation_step++;
            break;
            
        default:
            break;
    }
    
    WS2812_Update();
}

/**
 * @brief HSV 转 RGB
 */
WS2812_Color_t WS2812_HSVtoRGB(uint16_t h, uint8_t s, uint8_t v)
{
    WS2812_Color_t rgb;
    uint8_t region, remainder, p, q, t;
    
    if (s == 0) {
        rgb.r = v;
        rgb.g = v;
        rgb.b = v;
        return rgb;
    }
    
    region = h / 60;
    remainder = (h - (region * 60)) * 255 / 60;
    
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    
    switch (region) {
        case 0:
            rgb.r = v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = v;
            break;
        default:
            rgb.r = v; rgb.g = p; rgb.b = q;
            break;
    }
    
    return rgb;
}

/**
 * @brief DMA 传输完成回调
 */
void WS2812_DMA_CompleteCallback(void)
{
    HAL_TIM_PWM_Stop_DMA(&WS2812_TIM, WS2812_TIM_CHANNEL);
    g_ws2812_busy = false;
}

/**
 * @brief 获取预设颜色的 RGB 值
 */
WS2812_Color_t WS2812_GetPresetRGB(WS2812_PresetColor_t preset)
{
    WS2812_Color_t color = {0, 0, 0};
    
    switch (preset) {
        case WS2812_COLOR_OFF:
            color.r = 0; color.g = 0; color.b = 0;
            break;
        case WS2812_COLOR_RED:
            color.r = 255; color.g = 0; color.b = 0;
            break;
        case WS2812_COLOR_GREEN:
            color.r = 0; color.g = 255; color.b = 0;
            break;
        case WS2812_COLOR_BLUE:
            color.r = 0; color.g = 0; color.b = 255;
            break;
        case WS2812_COLOR_WHITE:
            color.r = 255; color.g = 255; color.b = 255;
            break;
        case WS2812_COLOR_YELLOW:
            color.r = 255; color.g = 255; color.b = 0;
            break;
        case WS2812_COLOR_CYAN:
            color.r = 0; color.g = 255; color.b = 255;
            break;
        case WS2812_COLOR_MAGENTA:
            color.r = 255; color.g = 0; color.b = 255;
            break;
        case WS2812_COLOR_ORANGE:
            color.r = 255; color.g = 128; color.b = 0;
            break;
        case WS2812_COLOR_PURPLE:
            color.r = 128; color.g = 0; color.b = 255;
            break;
        default:
            break;
    }
    
    return color;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 填充 DMA 缓冲区
 */
static void WS2812_FillBuffer(void)
{
    uint16_t buffer_index = 0;
    
    /* 填充 LED 数据 (GRB 顺序) */
    for (uint8_t led = 0; led < WS2812_LED_COUNT; led++) {
        uint8_t g = WS2812_ApplyBrightness(g_ws2812_leds[led].g);
        uint8_t r = WS2812_ApplyBrightness(g_ws2812_leds[led].r);
        uint8_t b = WS2812_ApplyBrightness(g_ws2812_leds[led].b);
        
        /* Green */
        for (int8_t bit = 7; bit >= 0; bit--) {
            ws2812_dma_buffer[buffer_index++] = (g & (1 << bit)) ? WS2812_PWM_HI : WS2812_PWM_LO;
        }
        
        /* Red */
        for (int8_t bit = 7; bit >= 0; bit--) {
            ws2812_dma_buffer[buffer_index++] = (r & (1 << bit)) ? WS2812_PWM_HI : WS2812_PWM_LO;
        }
        
        /* Blue */
        for (int8_t bit = 7; bit >= 0; bit--) {
            ws2812_dma_buffer[buffer_index++] = (b & (1 << bit)) ? WS2812_PWM_HI : WS2812_PWM_LO;
        }
    }
    
    /* 填充复位脉冲 (低电平) */
    for (uint8_t i = 0; i < WS2812_RESET_SLOTS; i++) {
        ws2812_dma_buffer[buffer_index++] = 0;
    }
}

/**
 * @brief 应用亮度
 */
static uint8_t WS2812_ApplyBrightness(uint8_t value)
{
    return (uint8_t)((uint16_t)value * ws2812_brightness / 255);
}