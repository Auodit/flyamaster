#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

// 二阶巴特沃斯低通滤波器结构体
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2; // 输入历史
    float y1, y2; // 输出历史
} LPF2_TypeDef;

void LPF2_Init(LPF2_TypeDef *lpf, float sample_freq, float cutoff_freq);
float LPF2_Apply(LPF2_TypeDef *lpf, float input);

#endif
