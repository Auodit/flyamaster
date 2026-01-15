#include "buzzer.h"

// 蜂鸣器连接到 PC13 引脚 (需要三极管驱动电路)
// 参考 hardware_design_guide.md 中的电路设计

static BuzzerState_t g_buzzer_state = BUZZER_OFF;
static uint32_t buzzer_timer = 0;
static uint8_t buzzer_toggle = 0;
static uint8_t beep_count = 0;

/**
 * @brief 初始化蜂鸣器
 */
void Buzzer_Init(void) {
    // GPIO 已在 CubeMX 中配置，这里无需额外初始化
    Buzzer_Off();
}

/**
 * @brief 打开蜂鸣器
 */
void Buzzer_On(void) {
    // 使用 PC13 引脚控制蜂鸣器 (高电平响)
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

/**
 * @brief 关闭蜂鸣器
 */
void Buzzer_Off(void) {
    // 使用 PC13 引脚控制蜂鸣器 (低电平停)
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 设置蜂鸣器状态
 */
void Buzzer_SetState(BuzzerState_t state) {
    if (g_buzzer_state != state) {
        g_buzzer_state = state;
        buzzer_timer = HAL_GetTick();
        buzzer_toggle = 0;
        beep_count = 0;
        
        // 立即响应某些状态
        if (state == BUZZER_OFF) {
            Buzzer_Off();
        }
    }
}

/**
 * @brief 蜂鸣器状态机更新 (需要周期性调用，建议 10ms)
 */
void Buzzer_Update(void) {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - buzzer_timer;
    
    switch (g_buzzer_state) {
        case BUZZER_OFF:
            Buzzer_Off();
            break;
            
        case BUZZER_ARMED:
            // 解锁: 短响 100ms
            if (elapsed < 100) {
                Buzzer_On();
            } else if (elapsed < 200) {
                Buzzer_Off();
            } else {
                g_buzzer_state = BUZZER_OFF;
            }
            break;
            
        case BUZZER_DISARMED:
            // 上锁: 长响 300ms
            if (elapsed < 300) {
                Buzzer_On();
            } else if (elapsed < 400) {
                Buzzer_Off();
            } else {
                g_buzzer_state = BUZZER_OFF;
            }
            break;
            
        case BUZZER_LOW_VOLTAGE:
            // 低电压: 慢速间歇 (200ms 响, 800ms 停)
            if (elapsed % 1000 < 200) {
                Buzzer_On();
            } else {
                Buzzer_Off();
            }
            break;
            
        case BUZZER_CRITICAL:
            // 危急电压: 快速间歇 (100ms 响, 100ms 停)
            if (elapsed % 200 < 100) {
                Buzzer_On();
            } else {
                Buzzer_Off();
            }
            break;
            
        case BUZZER_FAILSAFE:
            // 失控: 连续响
            Buzzer_On();
            break;
            
        case BUZZER_CALIBRATING:
            // 校准中: 两声短响
            if (beep_count < 2) {
                if (buzzer_toggle == 0) {
                    if (elapsed < 100) {
                        Buzzer_On();
                    } else {
                        Buzzer_Off();
                        buzzer_toggle = 1;
                        buzzer_timer = now;
                    }
                } else {
                    if (elapsed > 100) {
                        buzzer_toggle = 0;
                        buzzer_timer = now;
                        beep_count++;
                    }
                }
            } else {
                g_buzzer_state = BUZZER_OFF;
            }
            break;
            
        default:
            Buzzer_Off();
            break;
    }
}
