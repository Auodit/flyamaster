# 算法设计与优化记录

> 文档版本: 1.0.0  
> 更新日期: 2026-01-18  
> 项目: FLYAMASTER 四轴飞控

---

## 目录

1. [姿态解算算法 (Mahony Filter)](#1-姿态解算算法-mahony-filter)
2. [串级 PID 控制器](#2-串级-pid-控制器)
3. [电机混控算法](#3-电机混控算法)
4. [气压高度计算](#4-气压高度计算)
5. [快速数学函数](#5-快速数学函数)
6. [滤波算法](#6-滤波算法)

---

## 1. 姿态解算算法 (Mahony Filter)

### 1.1 算法概述

Mahony 互补滤波器是一种基于四元数的姿态估计算法，通过融合陀螺仪和加速度计数据，实现高精度、低延迟的姿态解算。

**文件位置**: [`mahony.c`](../FLYAMASTER/Core/Src/mahony.c), [`mahony.h`](../FLYAMASTER/Core/Inc/mahony.h)

### 1.2 数学原理

#### 四元数表示

姿态使用四元数 $\mathbf{q} = [q_0, q_1, q_2, q_3]^T$ 表示，其中：
- $q_0$ 为标量部分 (w)
- $q_1, q_2, q_3$ 为向量部分 (x, y, z)

约束条件：$\|\mathbf{q}\| = 1$

#### 四元数微分方程

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}$$

其中 $\boldsymbol{\omega} = [0, \omega_x, \omega_y, \omega_z]^T$ 为角速度四元数。

展开形式：
$$\begin{bmatrix} \dot{q}_0 \\ \dot{q}_1 \\ \dot{q}_2 \\ \dot{q}_3 \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -q_1 & -q_2 & -q_3 \\ q_0 & -q_3 & q_2 \\ q_3 & q_0 & -q_1 \\ -q_2 & q_1 & q_0 \end{bmatrix} \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

#### 误差估计

利用加速度计测量的重力方向与四元数估计的重力方向之间的误差：

估计的重力方向（从四元数计算）：
$$\mathbf{v} = \begin{bmatrix} 2(q_1 q_3 - q_0 q_2) \\ 2(q_0 q_1 + q_2 q_3) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix}$$

误差向量（叉积）：
$$\mathbf{e} = \mathbf{a} \times \mathbf{v}$$

其中 $\mathbf{a}$ 为归一化的加速度计测量值。

#### PI 校正

使用 PI 控制器校正陀螺仪漂移：

$$\boldsymbol{\omega}_{corrected} = \boldsymbol{\omega}_{gyro} + K_p \mathbf{e} + K_i \int \mathbf{e} \, dt$$

### 1.3 伪代码

```
function Mahony_UpdateIMU(gx, gy, gz, ax, ay, az):
    dt = 1 / sample_freq
    
    // 归一化加速度计
    norm = sqrt(ax² + ay² + az²)
    ax, ay, az = ax/norm, ay/norm, az/norm
    
    // 估计重力方向
    vx = q1*q3 - q0*q2
    vy = q0*q1 + q2*q3
    vz = q0² - 0.5 + q3²
    
    // 计算误差 (叉积)
    ex = ay*vz - az*vy
    ey = az*vx - ax*vz
    ez = ax*vy - ay*vx
    
    // 积分误差
    integralFBx += Ki * ex * dt
    integralFBy += Ki * ey * dt
    integralFBz += Ki * ez * dt
    
    // 校正角速度
    gx += Kp * ex + integralFBx
    gy += Kp * ey + integralFBy
    gz += Kp * ez + integralFBz
    
    // 积分四元数
    gx *= 0.5 * dt
    gy *= 0.5 * dt
    gz *= 0.5 * dt
    
    q0 += -q1*gx - q2*gy - q3*gz
    q1 += q0*gx + q2*gz - q3*gy
    q2 += q0*gy - q1*gz + q3*gx
    q3 += q0*gz + q1*gy - q2*gx
    
    // 归一化四元数
    normalize(q0, q1, q2, q3)
```

### 1.4 四元数转欧拉角

$$\phi = \arctan2(2(q_0 q_1 + q_2 q_3), 1 - 2(q_1^2 + q_2^2))$$

$$\theta = \arcsin(2(q_0 q_2 - q_3 q_1))$$

$$\psi = \arctan2(2(q_0 q_3 + q_1 q_2), 1 - 2(q_2^2 + q_3^2))$$

### 1.5 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MAHONY_DEFAULT_KP` | 2.0 | 比例增益，越大收敛越快但可能振荡 |
| `MAHONY_DEFAULT_KI` | 0.005 | 积分增益，消除稳态误差 |
| `MAHONY_SAMPLE_FREQ` | 500 Hz | 采样频率 |

---

## 2. 串级 PID 控制器

### 2.1 算法概述

采用串级 PID 控制结构：
- **外环 (角度环)**: 目标角度 → 目标角速度
- **内环 (角速度环)**: 目标角速度 → 电机输出

**文件位置**: [`pid.c`](../FLYAMASTER/Core/Src/pid.c), [`pid.h`](../FLYAMASTER/Core/Inc/pid.h)

### 2.2 数学原理

#### 标准 PID 公式

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

离散化形式：
$$u[k] = K_p e[k] + K_i \sum_{i=0}^{k} e[i] \Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

#### 串级控制结构

```
目标角度 ──┬──► [角度环 PID] ──► 目标角速度 ──┬──► [角速度环 PID] ──► 电机输出
           │                                  │
当前角度 ──┘                    当前角速度 ──┘
```

### 2.3 抗积分饱和 (Anti-Windup)

积分项限幅：
$$I_{limited} = \text{clamp}(I, -I_{max}, I_{max})$$

其中 $I_{max} = \frac{\text{integral\_limit}}{K_i}$

### 2.4 微分滤波

使用一阶低通滤波器平滑微分项：

$$D_{filtered}[k] = \alpha \cdot D_{raw}[k] + (1-\alpha) \cdot D_{filtered}[k-1]$$

其中 $\alpha = 0.5$ (可配置)

### 2.5 伪代码

```
function PID_Compute(pid, setpoint, measurement, dt):
    error = setpoint - measurement
    
    // P 项
    p_term = Kp * error
    
    // I 项 (带限幅)
    integral += error * dt
    integral = clamp(integral, -integral_limit/Ki, integral_limit/Ki)
    i_term = Ki * integral
    
    // D 项 (带滤波)
    derivative = (error - prev_error) / dt
    derivative = alpha * derivative + (1-alpha) * prev_derivative
    d_term = Kd * derivative
    
    // 保存状态
    prev_error = error
    prev_derivative = derivative
    
    // 计算输出
    output = p_term + i_term + d_term
    output = clamp(output, -output_limit, output_limit)
    
    return output
```

### 2.6 参数配置

#### 角度环 (外环)

| 轴 | Kp | Ki | Kd | 输出限幅 |
|----|----|----|----|----|
| Roll | 4.0 | 0.02 | 0.0 | ±500 °/s |
| Pitch | 4.0 | 0.02 | 0.0 | ±500 °/s |

#### 角速度环 (内环)

| 轴 | Kp | Ki | Kd | 输出限幅 |
|----|----|----|----|----|
| Roll | 0.7 | 0.3 | 0.02 | ±500 |
| Pitch | 0.7 | 0.3 | 0.02 | ±500 |
| Yaw | 2.0 | 0.5 | 0.0 | ±500 |

---

## 3. 电机混控算法

### 3.1 算法概述

X 型四轴机架的电机混控，将油门和姿态控制量映射到四个电机的 PWM 输出。

**文件位置**: [`motor.c`](../FLYAMASTER/Core/Src/motor.c), [`motor.h`](../FLYAMASTER/Core/Inc/motor.h)

### 3.2 机架布局

```
        前
    M3     M1
      \   /
       \ /
        X
       / \
      /   \
    M2     M4
        后

M1 (右前): CCW (逆时针)
M2 (左后): CCW (逆时针)
M3 (左前): CW  (顺时针)
M4 (右后): CW  (顺时针)
```

### 3.3 混控公式

$$M_1 = T - R - P - Y$$
$$M_2 = T + R + P - Y$$
$$M_3 = T + R - P + Y$$
$$M_4 = T - R + P + Y$$

其中：
- $T$ = 油门 (Throttle)
- $R$ = 横滚控制量 (Roll)
- $P$ = 俯仰控制量 (Pitch)
- $Y$ = 偏航控制量 (Yaw)

### 3.4 Airmode 算法

当电机输出超出范围时，整体偏移以保持姿态控制能力：

```
function Airmode(motor_out[4]):
    max_out = max(motor_out)
    min_out = min(motor_out)
    
    if max_out > PWM_MAX:
        offset = PWM_MAX - max_out
    else if min_out < PWM_IDLE:
        offset = PWM_IDLE - min_out
    else:
        offset = 0
    
    for i in 0..3:
        motor_out[i] += offset
```

### 3.5 PWM 映射

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR_PWM_MIN` | 1000 μs | 电机停止 |
| `MOTOR_PWM_IDLE` | 1100 μs | 怠速 |
| `MOTOR_PWM_MAX` | 2000 μs | 最大油门 |
| `MOTOR_CCR_MIN` | 2000 | TIM8 CCR 最小值 |
| `MOTOR_CCR_MAX` | 4000 | TIM8 CCR 最大值 |

CCR 转换公式：$CCR = PWM \times 2$

---

## 4. 气压高度计算

### 4.1 算法概述

使用 SPL06-001 气压计测量气压，通过国际标准大气模型计算高度。

**文件位置**: [`spl06.c`](../FLYAMASTER/Core/Src/spl06.c), [`spl06.h`](../FLYAMASTER/Core/Inc/spl06.h)

### 4.2 气压高度公式

国际标准大气模型 (ISA)：

$$h = \frac{T_0}{L} \left[ 1 - \left( \frac{P}{P_0} \right)^{\frac{R \cdot L}{g \cdot M}} \right]$$

简化公式：

$$h = 44330 \times \left[ 1 - \left( \frac{P}{P_0} \right)^{0.1903} \right]$$

其中：
- $P$ = 当前气压 (Pa)
- $P_0$ = 海平面气压 (101325 Pa)
- $T_0$ = 海平面温度 (288.15 K)
- $L$ = 温度递减率 (0.0065 K/m)
- $R$ = 气体常数 (8.31447 J/(mol·K))
- $g$ = 重力加速度 (9.80665 m/s²)
- $M$ = 空气摩尔质量 (0.0289644 kg/mol)

### 4.3 温度补偿

SPL06 使用多项式补偿：

$$T_{comp} = \frac{c_0}{2} + c_1 \cdot T_{raw,sc}$$

$$P_{comp} = c_{00} + P_{raw,sc} \cdot (c_{10} + P_{raw,sc} \cdot (c_{20} + P_{raw,sc} \cdot c_{30})) + T_{raw,sc} \cdot c_{01} + T_{raw,sc} \cdot P_{raw,sc} \cdot (c_{11} + P_{raw,sc} \cdot c_{21})$$

其中 $T_{raw,sc}$ 和 $P_{raw,sc}$ 为缩放后的原始值。

### 4.4 高度滤波

一阶低通滤波：

$$h_{filtered}[k] = (1-\alpha) \cdot h_{filtered}[k-1] + \alpha \cdot h_{raw}[k]$$

其中 $\alpha = 0.1$

### 4.5 垂直速度计算

$$v_z = \frac{h_{filtered}[k] - h_{filtered}[k-1]}{\Delta t}$$

---

## 5. 快速数学函数

### 5.1 快速平方根倒数 (Fast Inverse Square Root)

著名的 Quake III 算法，用于四元数归一化：

**文件位置**: [`mahony.c`](../FLYAMASTER/Core/Src/mahony.c) - `InvSqrt()`

```c
float InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);      // 魔数
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y)); // 第一次牛顿迭代
    y = y * (1.5f - (halfx * y * y)); // 第二次牛顿迭代
    return y;
}
```

#### 原理

利用 IEEE 754 浮点数格式的特性，通过位操作快速估计 $1/\sqrt{x}$，然后用牛顿迭代法精化。

牛顿迭代公式：
$$y_{n+1} = y_n \cdot \left( \frac{3}{2} - \frac{x}{2} \cdot y_n^2 \right)$$

---

## 6. 滤波算法

### 6.1 一阶低通滤波器 (LPF)

$$y[k] = \alpha \cdot x[k] + (1-\alpha) \cdot y[k-1]$$

截止频率与 $\alpha$ 的关系：
$$\alpha = \frac{2\pi f_c \Delta t}{1 + 2\pi f_c \Delta t}$$

应用场景：
- 高度滤波 ($\alpha = 0.1$)
- PID 微分滤波 ($\alpha = 0.5$)

### 6.2 互补滤波器

Mahony 算法本质上是一个互补滤波器：
- 高频信号来自陀螺仪积分
- 低频信号来自加速度计

$$\hat{\theta} = \alpha \cdot (\hat{\theta}_{prev} + \omega \cdot \Delta t) + (1-\alpha) \cdot \theta_{acc}$$

---

## 优化记录

### [待记录] 优化项目

| 日期 | 优化内容 | 优化前 | 优化后 | 改进 |
|------|----------|--------|--------|------|
| - | - | - | - | - |

---

## 参考文献

1. Mahony, R., Hamel, T., & Pflimlin, J. M. (2008). Nonlinear complementary filters on the special orthogonal group. *IEEE Transactions on Automatic Control*.

2. Madgwick, S. O. (2010). An efficient orientation filter for inertial and inertial/magnetic sensor arrays. *Report x-io and University of Bristol*.

3. Lomont, C. (2003). Fast inverse square root. *Tech-Report*.

4. International Standard Atmosphere (ISA). *ICAO Doc 7488*.