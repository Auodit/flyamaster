/**
 * @file led_indicator.h
 * @brief 4 颗 LED 状态指示灯驱动
 * @version 2.0.0
 * @date 2026-01-24
 * 
 * LED 功能定义 (PB12-PB15):
 * - PB12: LED_SYS  (蓝色) - 系统状态
 * - PB13: LED_MODE (白色) - 飞行模式
 * - PB14: LED_GPS  (绿色) - GPS 状态
 * - PB15: LED_VTX  (青色) - 图传/警告
 * 
 * 硬件接法: 灌电流 (Sink Mode), 低电平亮
 */

#ifndef __LED_INDICATOR_H
#define __LED_INDICATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== LED 引脚定义 ==================== */

#ifndef LED_SYS_Pin
#define LED_SYS_Pin         GPIO_PIN_12
#define LED_SYS_GPIO_Port   GPIOB
#endif

#ifndef LED_MODE_Pin
#define LED_MODE_Pin        GPIO_PIN_13
#define LED_MODE_GPIO_Port  GPIOB
#endif

#ifndef LED_GPS_Pin
#define LED_GPS_Pin         GPIO_PIN_14
#define LED_GPS_GPIO_Port   GPIOB
#endif

#ifndef LED_VTX_Pin
#define LED_VTX_Pin         GPIO_PIN_15
#define LED_VTX_GPIO_Port   GPIOB
#endif

/* ==================== LED 索引枚举 ==================== */

typedef enum {
    LED_ID_SYS = 0,     // 系统心跳 (蓝色) - PB12
    LED_ID_MODE,        // 飞行模式 (白色) - PB13
    LED_ID_GPS,         // GPS 状态 (绿色) - PB14
    LED_ID_VTX,         // 图传/警告 (青色) - PB15
    LED_ID_COUNT        // LED 总数 = 4
} LED_ID_t;

/* ==================== LED 状态枚举 ==================== */

typedef enum {
    LED_STATE_OFF = 0,      // 熄灭
    LED_STATE_ON,           // 常亮
    LED_STATE_BLINK_SLOW,   // 慢闪 (1Hz)
    LED_STATE_BLINK_FAST,   // 快闪 (4Hz)
    LED_STATE_BLINK_PULSE   // 脉冲闪烁 (短亮长灭)
} LED_State_t;

/* ==================== 系统状态枚举 (用于自动控制) ==================== */

// LED_SYS 系统状态
typedef enum {
    SYS_STATE_INIT = 0,     // 初始化中 (灭)
    SYS_STATE_STANDBY,      // 待机正常 (慢闪)
    SYS_STATE_ARMED,        // 已解锁 (常亮)
    SYS_STATE_FLYING        // 飞行中 (快闪)
} SysState_t;

// LED_MODE 飞行模式
typedef enum {
    MODE_STATE_ANGLE = 0,   // 自稳/半自稳 (灭)
    MODE_STATE_ACRO,        // 手动 Acro (常亮)
    MODE_STATE_RTH          // 返航 (闪烁)
} ModeState_t;

// LED_GPS 状态
typedef enum {
    GPS_STATE_NONE = 0,     // 无 GPS (灭)
    GPS_STATE_SEARCHING,    // 搜星中 (闪烁)
    GPS_STATE_LOCKED        // 3D Fix (常亮)
} GpsState_t;

// LED_VTX 图传/警告状态
typedef enum {
    VTX_STATE_NORMAL = 0,   // 正常 (灭)
    VTX_STATE_PIT,          // PIT 模式 (常亮)
    VTX_STATE_WARNING       // 低电压警告 (快闪)
} VtxState_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化 LED 指示灯系统
 * @note 所有 LED 初始化为熄灭状态
 */
void LED_Init(void);

/**
 * @brief LED 周期更新函数 (需在定时任务中调用)
 * @param tick_ms 当前系统时间 (毫秒)
 * @note 建议在 10Hz 任务中调用
 */
void LED_Update(uint32_t tick_ms);

/* ==================== 底层控制 API ==================== */

/**
 * @brief 设置单个 LED 状态
 * @param led_id LED 索引
 * @param state LED 状态
 */
void LED_SetState(LED_ID_t led_id, LED_State_t state);

/**
 * @brief 直接控制 LED 开关
 * @param led_id LED 索引
 * @param on true=亮, false=灭
 */
void LED_Set(LED_ID_t led_id, bool on);

/**
 * @brief 切换 LED 状态
 * @param led_id LED 索引
 */
void LED_Toggle(LED_ID_t led_id);

/**
 * @brief 关闭所有 LED
 */
void LED_AllOff(void);

/**
 * @brief 点亮所有 LED (用于自检)
 */
void LED_AllOn(void);

/* ==================== 高级状态控制 API ==================== */

/**
 * @brief 设置系统状态 (自动控制 LED_SYS)
 * @param state 系统状态
 */
void LED_SetSysState(SysState_t state);

/**
 * @brief 设置飞行模式 (自动控制 LED_MODE)
 * @param state 飞行模式
 */
void LED_SetModeState(ModeState_t state);

/**
 * @brief 设置 GPS 状态 (自动控制 LED_GPS)
 * @param state GPS 状态
 */
void LED_SetGpsState(GpsState_t state);

/**
 * @brief 设置图传/警告状态 (自动控制 LED_VTX)
 * @param state 图传状态
 */
void LED_SetVtxState(VtxState_t state);

/* ==================== 特效 API ==================== */

/**
 * @brief 启动自检灯效 (所有 LED 依次点亮)
 * @note 非阻塞，需要持续调用 LED_Update()
 */
void LED_StartSelfTest(void);

/**
 * @brief 检查自检是否完成
 * @return true=完成, false=进行中
 */
bool LED_IsSelfTestDone(void);

/**
 * @brief 启动解锁灯效 (快速闪烁 3 次)
 */
void LED_StartArmEffect(void);

/**
 * @brief 启动上锁灯效 (渐灭效果)
 */
void LED_StartDisarmEffect(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_INDICATOR_H */