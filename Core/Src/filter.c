#include "filter.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化二阶巴特沃斯低通滤波器
 * @param sample_freq 采样频率 (Hz)
 * @param cutoff_freq 截止频率 (Hz)
 */
void LPF2_Init(LPF2_TypeDef *lpf, float sample_freq, float cutoff_freq) {
    // Risk #049 修复：参数验证
    if (lpf == NULL) return;
    
    // 防止违反奈奎斯特定理（截止频率必须 < 采样频率/2）
    if (cutoff_freq >= sample_freq / 2.5f) {
        cutoff_freq = sample_freq / 2.5f;  // 自动限制到安全范围
    }
    
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);
    float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    lpf->b0 = ohm * ohm / c;
    lpf->b1 = 2.0f * lpf->b0;
    lpf->b2 = lpf->b0;
    lpf->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    lpf->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;

    lpf->x1 = lpf->x2 = 0.0f;
    lpf->y1 = lpf->y2 = 0.0f;
}

/**
 * @brief 应用滤波器
 */
float LPF2_Apply(LPF2_TypeDef *lpf, float input) {
    float output = lpf->b0 * input + lpf->b1 * lpf->x1 + lpf->b2 * lpf->x2 
                 - lpf->a1 * lpf->y1 - lpf->a2 * lpf->y2;

    // 更新历史值
    lpf->x2 = lpf->x1;
    lpf->x1 = input;
    lpf->y2 = lpf->y1;
    lpf->y1 = output;

    return output;
}
