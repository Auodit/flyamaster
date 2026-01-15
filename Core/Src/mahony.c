#include "mahony.h"
#include <math.h>
#include <stddef.h>

// 快速平方根倒数算法 (Quake III Arena)
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(Mahony_TypeDef *mahony, float kp, float ki) {
    mahony->q0 = 1.0f;
    mahony->q1 = 0.0f;
    mahony->q2 = 0.0f;
    mahony->q3 = 0.0f;
    mahony->integralFBx = 0.0f;
    mahony->integralFBy = 0.0f;
    mahony->integralFBz = 0.0f;
    mahony->Kp = kp;
    mahony->Ki = ki;
}

void Mahony_Update(Mahony_TypeDef *mahony, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    // Risk #054 修复：添加指针空检查
    if (mahony == NULL) return;
    
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果加速度计数据无效，则只进行陀螺仪积分
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向 (参考坐标系转机体坐标系)
        halfvx = mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2;
        halfvy = mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3;
        halfvz = mahony->q0 * mahony->q0 - 0.5f + mahony->q3 * mahony->q3;

        // 计算误差 (叉乘)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分误差 (用于消除陀螺仪零偏)
        if(mahony->Ki > 0.0f) {
            mahony->integralFBx += mahony->Ki * halfex * dt;
            mahony->integralFBy += mahony->Ki * halfey * dt;
            mahony->integralFBz += mahony->Ki * halfez * dt;
            gx += mahony->integralFBx;
            gy += mahony->integralFBy;
            gz += mahony->integralFBz;
        } else {
            mahony->integralFBx = 0.0f;
            mahony->integralFBy = 0.0f;
            mahony->integralFBz = 0.0f;
        }

        // 比例修正
        gx += mahony->Kp * halfex;
        gy += mahony->Kp * halfey;
        gz += mahony->Kp * halfez;
    }

    // 四元数微分方程积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = mahony->q0;
    qb = mahony->q1;
    qc = mahony->q2;
    mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
    mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
    mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
    mahony->q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= recipNorm;
    mahony->q1 *= recipNorm;
    mahony->q2 *= recipNorm;
    mahony->q3 *= recipNorm;
}

void Mahony_GetEulerAngle(Mahony_TypeDef *mahony, float *roll, float *pitch, float *yaw) {
    // Risk #054 修复：添加指针空检查
    if (mahony == NULL || roll == NULL || pitch == NULL || yaw == NULL) return;
    
    *roll = atan2f(2.0f * (mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3), 1.0f - 2.0f * (mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2));
    *pitch = asinf(2.0f * (mahony->q0 * mahony->q2 - mahony->q3 * mahony->q1));
    *yaw = atan2f(2.0f * (mahony->q0 * mahony->q3 + mahony->q1 * mahony->q2), 1.0f - 2.0f * (mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3));
    
    // 弧度转角度
    *roll *= 57.29578f;
    *pitch *= 57.29578f;
    *yaw *= 57.29578f;
}
