/**
 * @file led_indicator.c
 * @brief 7 颗 LED 状态指示灯驱动实现
 * @version 1.0.0
 * @date 2026-01-23
 */

#include "led_indicator.h"

/* ==================== 私有类型定义 ==================== */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} LED_GPIO_t;

typedef struct {
    LED_State_t state;
    bool current_on;        // 当前是否点亮
    uint32_t last_toggle;   // 上次切换时间
} LED_Control_t;

/* ==================== 私有变量 ==================== */

// LED GPIO 映射表
static const LED_GPIO_t led_gpio[LED_ID_COUNT] = {
    [LED_ID_SYS]  = { LED_SYS_GPIO_Port,  LED_SYS_Pin  },
    [LED_ID_ERR]  = { LED_ERR_GPIO_Port,  LED_ERR_Pin  },
    [LED_ID_GPS]  = { LED_GPS_GPIO_Port,  LED_GPS_Pin  },
    [LED_ID_RX]   = { LED_RX_GPIO_Port,   LED_RX_Pin   },
    [LED_ID_MODE] = { LED_MODE_GPIO_Port, LED_MODE_Pin },
    [LED_ID_LOG]  = { LED_LOG_GPIO_Port,  LED_LOG_Pin  },
    [LED_ID_VTX]  = { LED_VTX_GPIO_Port,  LED_VTX_Pin  },
};

// LED 控制状态
static LED_Control_t led_ctrl[LED_ID_COUNT];

// 闪烁周期定义 (毫秒)
#define BLINK_SLOW_PERIOD   1000    // 慢闪 1Hz (500ms 亮, 500ms 灭)
#define BLINK_FAST_PERIOD   250     // 快闪 4Hz (125ms 亮, 125ms 灭)
#define BLINK_PULSE_ON      100     // 脉冲亮时间
#define BLINK_PULSE_OFF     900     // 脉冲灭时间

// 自检状态
static bool self_test_active = false;
static uint8_t self_test_step = 0;
static uint32_t self_test_start = 0;
#define SELF_TEST_STEP_TIME 200     // 每个 LED 亮 200ms

// 特效状态
static bool arm_effect_active = false;
static uint8_t arm_effect_count = 0;
static uint32_t arm_effect_start = 0;

/* ==================== 私有函数 ==================== */

/**
 * @brief 直接控制 LED 硬件 (灌电流接法: 低电平亮)
 */
static void LED_HW_Set(LED_ID_t led_id, bool on)
{
    if (led_id >= LED_ID_COUNT) return;
    
    // 灌电流接法: on=true 时输出低电平
    HAL_GPIO_WritePin(led_gpio[led_id].port, led_gpio[led_id].pin, 
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    led_ctrl[led_id].current_on = on;
}

/* ==================== 公共函数实现 ==================== */

void LED_Init(void)
{
    // 初始化所有 LED 为熄灭状态
    for (int i = 0; i < LED_ID_COUNT; i++) {
        led_ctrl[i].state = LED_STATE_OFF;
        led_ctrl[i].current_on = false;
        led_ctrl[i].last_toggle = 0;
        LED_HW_Set((LED_ID_t)i, false);
    }
    
    self_test_active = false;
    arm_effect_active = false;
}

void LED_Update(uint32_t tick_ms)
{
    // 处理自检灯效
    if (self_test_active) {
        uint32_t elapsed = tick_ms - self_test_start;
        uint8_t current_step = elapsed / SELF_TEST_STEP_TIME;
        
        if (current_step >= LED_ID_COUNT * 2) {
            // 自检完成
            self_test_active = false;
            LED_AllOff();
        } else if (current_step < LED_ID_COUNT) {
            // 依次点亮
            LED_AllOff();
            LED_HW_Set((LED_ID_t)current_step, true);
        } else {
            // 依次熄灭
            LED_AllOn();
            for (uint8_t i = 0; i <= current_step - LED_ID_COUNT; i++) {
                LED_HW_Set((LED_ID_t)i, false);
            }
        }
        return;
    }
    
    // 处理解锁灯效
    if (arm_effect_active) {
        uint32_t elapsed = tick_ms - arm_effect_start;
        uint8_t flash_count = elapsed / 100;  // 每 100ms 切换一次
        
        if (flash_count >= 6) {  // 3 次闪烁 = 6 次切换
            arm_effect_active = false;
        } else {
            bool on = (flash_count % 2) == 0;
            for (int i = 0; i < LED_ID_COUNT; i++) {
                LED_HW_Set((LED_ID_t)i, on);
            }
        }
        return;
    }
    
    // 正常状态更新
    for (int i = 0; i < LED_ID_COUNT; i++) {
        LED_Control_t *ctrl = &led_ctrl[i];
        
        switch (ctrl->state) {
            case LED_STATE_OFF:
                if (ctrl->current_on) {
                    LED_HW_Set((LED_ID_t)i, false);
                }
                break;
                
            case LED_STATE_ON:
                if (!ctrl->current_on) {
                    LED_HW_Set((LED_ID_t)i, true);
                }
                break;
                
            case LED_STATE_BLINK_SLOW:
                if (tick_ms - ctrl->last_toggle >= BLINK_SLOW_PERIOD / 2) {
                    LED_HW_Set((LED_ID_t)i, !ctrl->current_on);
                    ctrl->last_toggle = tick_ms;
                }
                break;
                
            case LED_STATE_BLINK_FAST:
                if (tick_ms - ctrl->last_toggle >= BLINK_FAST_PERIOD / 2) {
                    LED_HW_Set((LED_ID_t)i, !ctrl->current_on);
                    ctrl->last_toggle = tick_ms;
                }
                break;
                
            case LED_STATE_BLINK_PULSE:
                {
                    uint32_t period = BLINK_PULSE_ON + BLINK_PULSE_OFF;
                    uint32_t phase = (tick_ms - ctrl->last_toggle) % period;
                    bool should_on = (phase < BLINK_PULSE_ON);
                    if (should_on != ctrl->current_on) {
                        LED_HW_Set((LED_ID_t)i, should_on);
                    }
                }
                break;
        }
    }
}

void LED_SetState(LED_ID_t led_id, LED_State_t state)
{
    if (led_id >= LED_ID_COUNT) return;
    led_ctrl[led_id].state = state;
    led_ctrl[led_id].last_toggle = 0;  // 重置闪烁计时
}

void LED_Set(LED_ID_t led_id, bool on)
{
    if (led_id >= LED_ID_COUNT) return;
    led_ctrl[led_id].state = on ? LED_STATE_ON : LED_STATE_OFF;
    LED_HW_Set(led_id, on);
}

void LED_Toggle(LED_ID_t led_id)
{
    if (led_id >= LED_ID_COUNT) return;
    LED_HW_Set(led_id, !led_ctrl[led_id].current_on);
}

void LED_AllOff(void)
{
    for (int i = 0; i < LED_ID_COUNT; i++) {
        led_ctrl[i].state = LED_STATE_OFF;
        LED_HW_Set((LED_ID_t)i, false);
    }
}

void LED_AllOn(void)
{
    for (int i = 0; i < LED_ID_COUNT; i++) {
        led_ctrl[i].state = LED_STATE_ON;
        LED_HW_Set((LED_ID_t)i, true);
    }
}

/* ==================== 高级状态控制 ==================== */

void LED_SetSysState(SysState_t state)
{
    switch (state) {
        case SYS_STATE_INIT:
            LED_SetState(LED_ID_SYS, LED_STATE_OFF);
            break;
        case SYS_STATE_STANDBY:
            LED_SetState(LED_ID_SYS, LED_STATE_BLINK_SLOW);
            break;
        case SYS_STATE_ARMED:
            LED_SetState(LED_ID_SYS, LED_STATE_ON);
            break;
        case SYS_STATE_FLYING:
            LED_SetState(LED_ID_SYS, LED_STATE_BLINK_FAST);
            break;
    }
}

void LED_SetErrState(ErrState_t state)
{
    switch (state) {
        case ERR_STATE_NONE:
            LED_SetState(LED_ID_ERR, LED_STATE_OFF);
            break;
        case ERR_STATE_LOW_BATTERY:
            LED_SetState(LED_ID_ERR, LED_STATE_BLINK_SLOW);
            break;
        case ERR_STATE_SENSOR_FAIL:
            LED_SetState(LED_ID_ERR, LED_STATE_BLINK_FAST);
            break;
        case ERR_STATE_CRITICAL:
            LED_SetState(LED_ID_ERR, LED_STATE_ON);
            break;
    }
}

void LED_SetGpsState(GpsState_t state)
{
    switch (state) {
        case GPS_STATE_NONE:
            LED_SetState(LED_ID_GPS, LED_STATE_OFF);
            break;
        case GPS_STATE_SEARCHING:
            LED_SetState(LED_ID_GPS, LED_STATE_BLINK_SLOW);
            break;
        case GPS_STATE_LOCKED:
            LED_SetState(LED_ID_GPS, LED_STATE_ON);
            break;
    }
}

void LED_SetRxState(RxState_t state)
{
    switch (state) {
        case RX_STATE_FAILSAFE:
            LED_SetState(LED_ID_RX, LED_STATE_OFF);
            break;
        case RX_STATE_WEAK:
            LED_SetState(LED_ID_RX, LED_STATE_BLINK_FAST);
            break;
        case RX_STATE_OK:
            LED_SetState(LED_ID_RX, LED_STATE_ON);
            break;
    }
}

void LED_SetModeState(ModeState_t state)
{
    switch (state) {
        case MODE_STATE_ANGLE:
            LED_SetState(LED_ID_MODE, LED_STATE_OFF);
            break;
        case MODE_STATE_ACRO:
            LED_SetState(LED_ID_MODE, LED_STATE_ON);
            break;
        case MODE_STATE_RTH:
            LED_SetState(LED_ID_MODE, LED_STATE_BLINK_SLOW);
            break;
    }
}

void LED_SetLogState(LogState_t state)
{
    switch (state) {
        case LOG_STATE_IDLE:
            LED_SetState(LED_ID_LOG, LED_STATE_OFF);
            break;
        case LOG_STATE_RECORDING:
            LED_SetState(LED_ID_LOG, LED_STATE_BLINK_FAST);
            break;
    }
}

void LED_SetVtxState(VtxState_t state)
{
    switch (state) {
        case VTX_STATE_NORMAL:
            LED_SetState(LED_ID_VTX, LED_STATE_OFF);
            break;
        case VTX_STATE_PIT:
            LED_SetState(LED_ID_VTX, LED_STATE_ON);
            break;
    }
}

/* ==================== 特效 API ==================== */

void LED_StartSelfTest(void)
{
    self_test_active = true;
    self_test_step = 0;
    self_test_start = HAL_GetTick();
    LED_AllOff();
}

bool LED_IsSelfTestDone(void)
{
    return !self_test_active;
}

void LED_StartArmEffect(void)
{
    arm_effect_active = true;
    arm_effect_count = 0;
    arm_effect_start = HAL_GetTick();
}

void LED_StartDisarmEffect(void)
{
    // 简单实现: 所有灯熄灭
    LED_AllOff();
}