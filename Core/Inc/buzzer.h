#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

// 蜂鸣器状态
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ARMED,        // 解锁提示音 (短响一声)
    BUZZER_DISARMED,     // 上锁提示音 (长响一声)
    BUZZER_LOW_VOLTAGE,  // 低电压警告 (慢速间歇)
    BUZZER_CRITICAL,     // 危急电压 (快速间歇)
    BUZZER_FAILSAFE,     // 失控告警 (连续响)
    BUZZER_CALIBRATING   // 校准中 (两声)
} BuzzerState_t;

// 蜂鸣器配置 (假设使用 GPIO 控制无源蜂鸣器)
// 如果使用 TIM PWM 输出，需要修改驱动

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_SetState(BuzzerState_t state);
void Buzzer_Update(void);  // 需要周期性调用 (建议 10ms)

#endif
