#ifndef __MAHONY_H
#define __MAHONY_H

typedef struct {
    float q0, q1, q2, q3; // 四元数
    float integralFBx, integralFBy, integralFBz; // 积分误差
    float Kp, Ki; // 算法参数
} Mahony_TypeDef;

void Mahony_Init(Mahony_TypeDef *mahony, float kp, float ki);
void Mahony_Update(Mahony_TypeDef *mahony, float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Mahony_GetEulerAngle(Mahony_TypeDef *mahony, float *roll, float *pitch, float *yaw);

#endif
