# 问题追踪日志 (Issue Tracker)

> 项目: FLYAMASTER 飞控系统  
> 创建日期: 2026-01-18  
> 最后更新: 2026-01-22  
> 整合自: `03_issue_tracker.md` + `issues_and_feedback.md`

---

## 状态说明

| 状态 | 说明 |
|------|------|
| 🔴 OPEN | 待处理 |
| 🟡 IN_PROGRESS | 处理中 |
| 🟢 CLOSED | 已解决 |
| ⚪ WONTFIX | 不修复 |
| ⚠️ MITIGATED | 风险已缓解 |

---

## 📋 文档说明

本文档是飞控项目的 **"问题追踪中心"**，所有 Bug、潜在风险 (Risk) 和待解决问题都必须记录在此。

**强制规则**: 
1. 发现 Bug 时，立即在此文档中创建条目
2. 修复 Bug 后，必须标记为 `🟢 CLOSED` 并注明修复时间
3. 发现潜在隐患时，必须创建 Risk 条目并评估影响
4. 区分 `[SW]` 软件问题和 `[HW]` 硬件问题

---

## 🔴 待处理问题 (Active Issues)

### Issue #001 [SW] IMU 任务 DMA 同步问题 - 严重

**发现日期**: 2026-01-18  
**状态**: 🔴 OPEN  
**严重程度**: 🔴 高 (可能导致数据竞争)  
**文件**: [`freertos.c`](../FLYAMASTER/MDK-ARM/../Core/Src/freertos.c:264-280)

**问题描述**:
IMU 任务中使用 `osDelay(1)` 等待 DMA 完成，这是不可靠的同步方式：
```c
/* 触发 DMA 读取 MPU6050 */
MPU6050_ReadDMA();

/* 等待 DMA 完成 (通过信号量或短延时) */
osDelay(1);  // ❌ 不可靠！

/* 处理 DMA 数据 */
MPU6050_ProcessDMAData();
```

**风险**:
1. DMA 可能未完成就开始处理数据
2. 数据竞争导致姿态解算错误
3. 飞行中可能出现抖动或失控

**建议修复**:
使用 FreeRTOS 信号量或事件标志进行同步：
```c
/* 在 DMA 完成回调中释放信号量 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        osSemaphoreRelease(imuDataReadySem);
    }
}

/* 在任务中等待信号量 */
MPU6050_ReadDMA();
osSemaphoreAcquire(imuDataReadySem, osWaitForever);
MPU6050_ProcessDMAData();
```

---

### Issue #011 [SW] 缺少看门狗实现

**发现日期**: 2026-01-18  
**状态**: 🔴 OPEN  
**严重程度**: 🟡 中 (安全关键)  
**文件**: [`freertos.c`](../FLYAMASTER/MDK-ARM/../Core/Src/freertos.c:224)

**问题描述**:
看门狗喂狗代码被注释掉：
```c
/* 喂狗 (如果启用了 IWDG) */
/* HAL_IWDG_Refresh(&hiwdg); */
```

飞控系统应该启用看门狗以防止软件死锁。

**建议修复**:
1. 在 CubeMX 中启用 IWDG
2. 取消注释喂狗代码
3. 设置合适的超时时间 (建议 500ms-1s)

---

### Issue #015 [SW] 电池电流标定参数可能不正确

**发现日期**: 2026-01-18
**状态**: 🔴 OPEN
**严重程度**: 🟢 低 (需要硬件验证)
**文件**: [`battery.h`](../FLYAMASTER/MDK-ARM/../Core/Inc/battery.h:48-49)

**问题描述**:
电流标定参数假设特定的电流传感器特性：
```c
#define BATTERY_CURRENT_SCALE       33.3f   /* 电流比例 (A/V), 3.3V = 100A */
#define BATTERY_CURRENT_OFFSET      0.0f    /* 零点偏移 (V) */
```

实际电调/PDB 的电流传感器可能使用不同的标定值。

**建议**:
1. 查阅实际使用的电调/PDB 数据手册
2. 添加运行时标定功能
3. 在 `06_operation_guide.md` 中记录标定步骤

---

### Issue #018 [SW] SBUS 帧尾验证过于宽松

**发现日期**: 2026-01-18
**状态**: 🔴 OPEN
**严重程度**: 🟢 低
**文件**: [`sbus.c`](../FLYAMASTER/MDK-ARM/../Core/Src/sbus.c:107-109)

**问题描述**:
SBUS 帧尾验证被注释掉：
```c
/* 帧尾可以是 0x00, 0x04, 0x14, 0x24, 0x34 等 */
/* 只检查低 4 位 */
if ((buffer[24] & 0x0F) != 0x00 && (buffer[24] & 0x0F) != 0x04) {
    /* 某些接收机帧尾不标准，放宽检查 */
}
```

**风险**:
- 可能接受无效帧
- 但由于 SBUS 协议本身有帧头验证，风险较低

**建议**:
保持当前实现，但添加帧计数器统计无效帧比例。

---

### Issue #020 [SW] 电机混控 Airmode 偏移计算边界情况

**发现日期**: 2026-01-18
**状态**: 🔴 OPEN
**严重程度**: 🟢 低
**文件**: [`motor.c`](../FLYAMASTER/MDK-ARM/../Core/Src/motor.c:156-177)

**问题描述**:
Airmode 偏移计算只考虑单边超限：
```c
if (max_out > MOTOR_PWM_MAX) {
    offset = MOTOR_PWM_MAX - max_out;
} else if (min_out < MOTOR_PWM_IDLE) {
    offset = MOTOR_PWM_IDLE - min_out;
}
```

当控制输出范围超过可用 PWM 范围时 (max - min > PWM_MAX - PWM_IDLE)，无法同时满足两边约束。

**风险**:
- 极端机动时控制响应可能不正确
- 但在正常飞行中很少触发

**建议**:
添加范围压缩逻辑，当输出范围过大时按比例缩放。

---

## ⚠️ 潜在风险 (Risks)

### Risk #045 [SW] Flash 参数结构体内存对齐问题

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-14 16:00  
**缓解时间**: 2026-01-14 16:30  
**严重程度**: 🟡 Medium  
**模块**: Flash Params ([`flash_params.h`](../FLYAMASTER/Core/Inc/flash_params.h))

**风险描述**: 
- `FlashParams_t` 结构体未使用 `__attribute__((packed))`，编译器可能插入 padding
- 导致 Flash 读写时数据错位，CRC16 校验失败

**影响**: 
- PID 参数掉电后无法恢复
- 飞控每次上电都使用默认参数，无法保存调参结果

**缓解措施**:
- 在 [`flash_params.h:70`](../FLYAMASTER/Core/Inc/flash_params.h:70) 添加 `__attribute__((packed))` 属性
- 重新测试 Flash 读写功能

**后续行动**: 
- [ ] 在实际硬件上验证 Flash 存储是否稳定
- [ ] 添加更多的数据完整性检查 (如双 CRC 校验)

---

### Risk #066 [SW] SPL06 气压计数据竞争 (Race Condition)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-14 18:00  
**缓解时间**: 2026-01-14 18:15  
**严重程度**: 🔴 High  
**模块**: SPL06 ([`spl06.h`](../FLYAMASTER/Core/Inc/spl06.h))

**风险描述**: 
- `SPL06_Data` 全局变量在多个 RTOS 任务中读写，但未加 `volatile` 修饰符
- 可能导致编译器优化时缓存旧数据，不同任务读取到不一致的高度值

**影响**: 
- 高度控制震荡或失效
- 定高模式下飞机突然坠落

**缓解措施**:
- 在 [`spl06.h:102`](../FLYAMASTER/Core/Inc/spl06.h:102) 添加 `volatile` 修饰符:
  ```c
  extern volatile SPL06_TypeDef SPL06_Data;
  ```
- 在关键读写操作处添加 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 临界区保护

**后续行动**: 
- [x] 检查所有 RTOS 共享变量，确保都加了 `volatile`
- [ ] 使用 FreeRTOS 互斥锁 (Mutex) 替代临界区，避免优先级反转

---

### Risk #088 [HW] QMC5883L 磁力计 I2C 总线冲突

**状态**: ⚠️ MITIGATED (已缓解 - 不适用)  
**发现时间**: 2026-01-15 09:00  
**缓解时间**: 2026-01-18 13:15  
**严重程度**: 🟡 Medium  
**模块**: QMC5883L ([`qmc5883l.c`](../FLYAMASTER/Core/Src/qmc5883l.c))

**风险描述**:
- QMC5883L 和其他 I2C3 设备 (如扩展传感器) 共享总线，可能发生地址冲突
- QMC5883L 固定地址 0x0D，如果其他设备也是 0x0D，会导致通信失败

**影响**:
- 磁力计数据读取失败
- 航向角锁定功能失效

**缓解措施 (已确认)**:
- **I2C3 总线上只有 QMC5883L 一个设备**，不存在地址冲突风险
- 无需添加总线扫描或错误重试机制

**修复人**: 指挥官确认

---

### Risk #107 [SW] GPS 静态缓冲区线程安全隐患

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-16 11:50  
**缓解时间**: 2026-01-18 13:10  
**严重程度**: 🟡 Medium  
**模块**: GPS ([`gps.c`](../FLYAMASTER/Core/Src/gps.c))

**风险描述**:
- `gps.c` 中的 `Get_Field` 函数使用了 `static` 局部缓冲区
- 如果未来有多个 RTOS 任务同时调用 GPS 解析函数，会导致数据竞争和内存破坏

**影响**:
- GPS 数据解析错乱
- 系统随机 Crash

**缓解措施 (已实施)**:
- 重构 `Get_Field()` 函数签名，移除 `static` 缓冲区
- 改为传入外部缓冲区: `Get_Field(buf, field_index, out_field, out_size)`
- 调用方在栈上分配 32 字节缓冲区

**修复人**: Doro

---

### Risk #108 [SW] QMC5883L 倾斜补偿死循环风险

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-16 11:55  
**缓解时间**: 2026-01-18 13:24  
**严重程度**: 🔴 High  
**模块**: QMC5883L Algorithm ([`qmc5883l.c`](../FLYAMASTER/Core/Src/qmc5883l.c))

**风险描述**:
- 磁力计倾斜补偿公式中，当 Roll/Pitch 接近 90 度时，分母 `cos(pitch)` 或 `cos(roll)` 趋近于 0
- 可能导致除零错误或浮点数溢出 (NaN/Inf)，进而导致 PID 计算异常

**影响**:
- 飞机在大角度机动时突然疯转 (Yaw Spin)
- 系统死机

**缓解措施 (已实施)**:
1. **asinf 输入范围保护**: 将输入钳位到 [-1.0, 1.0]，防止 NaN
2. **大角度保护**: 当 Roll 或 Pitch 超过 ±70° 时，返回上次有效航向
3. 新增宏 `MAG_TILT_LIMIT_RAD` (70° in radians)

**修复人**: Doro

---

### Risk #111 [SW] 高频任务中的 printf 阻塞风险

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18  
**缓解时间**: 2026-01-18 13:00  
**严重程度**: 🔴 High  
**模块**: RTOS Tasks ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c)), RingBuffer ([`ring_buffer.c`](../FLYAMASTER/Core/Src/ring_buffer.c))

**风险描述**:
- 在 `imu_task` (500Hz) 和 `ctl_task` (250Hz) 中直接调用了 `printf`
- 标准库的 `printf` 通常重定向到 `HAL_UART_Transmit` (阻塞模式)
- 即使有降频处理，单次打印也可能阻塞数毫秒，导致姿态解算和 PID 控制周期抖动

**影响**:
- 飞行手感卡顿，姿态控制不稳定
- 严重时看门狗复位或系统崩溃

**缓解措施 (已实施)**:
1. **Phase 1 (临时修复)**: 移除高频任务中的所有 `printf` 调用
2. **Phase 2 (永久修复)**: 实现 RingBuffer + DMA 异步日志系统
   - 新增 `ring_buffer.c/h` 模块
   - 512 字节环形缓冲区 + DMA2_Stream7 异步传输
   - 重写 `fputc()` 为非阻塞模式
   - 提供 `Log_Printf()` 替代函数

**修复人**: Doro

---

### Risk #112 [SW] Failsafe 执行层缺失 (专家反馈)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 12:00  
**缓解时间**: 2026-01-18 13:10  
**严重程度**: 🔴 High  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))

**风险描述**:
- 原有失控保护仅执行 `Motor_Stop()` 立即上锁
- 缺少分阶段保护，可能导致高空失控时直接坠落

**影响**:
- 信号丢失时飞机直接坠落
- 无法给飞手恢复信号的时间窗口

**缓解措施 (已实施)**:
- 实现 3 阶段 Failsafe 保护:
  - Stage 1 (< 1s): 保持当前姿态，等待信号恢复
  - Stage 2 (1s ~ 5s): 自稳模式 + 35% 下降油门
  - Stage 3 (> 5s): 强制上锁
- 新增 `FailsafeStage_t` 状态机和相关宏定义

**修复人**: Doro

---

### Risk #113 [SW] GPS 模式解锁检查缺失 (专家反馈)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 12:00  
**缓解时间**: 2026-01-18 13:10  
**严重程度**: 🔴 High  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))

**风险描述**:
- 定点模式下解锁时未检查 GPS 有效性
- 可能导致无 GPS 信号时进入定点模式，飞机失控

**影响**:
- 飞机在无 GPS 信号时尝试定点，导致漂移或失控

**缓解措施 (已实施)**:
- 在 `PreArm_SafetyCheck()` 中添加 GPS 检查:
  - `fix_type >= 3` (3D Fix)
  - `satellites >= 6`
- 仅在定点模式 (MODE_POSITION) 下强制检查

**修复人**: Doro

---

### Risk #114 [SW] 磁力计电流补偿未调用 (专家反馈)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 12:00  
**缓解时间**: 2026-01-18 13:10  
**严重程度**: 🟡 Medium  
**模块**: IMU Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))

**风险描述**:
- `QMC5883L_CompensateCurrent()` 接口已实现但未在 imu_task 中调用
- 电机电流产生的磁场干扰未被补偿

**影响**:
- 高油门时航向角漂移
- 定点模式下飞机画圈

**缓解措施 (已实施)**:
- 在 `imu_task` 的磁力计读取后调用 `QMC5883L_CompensateCurrent(throttle)`
- 使用当前油门值估算电机电流干扰

**修复人**: Doro

---

### Risk #115 [SW] SBUS 全局变量缺少 volatile 修饰符 (代码审查)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 13:30  
**缓解时间**: 2026-01-18 13:31  
**严重程度**: 🟡 Medium  
**模块**: SBUS ([`sbus.c`](../FLYAMASTER/Core/Src/sbus.c))

**风险描述**:
- `SBUS_Data` 和 `last_sbus_time` 在 DMA/中断回调中被更新，在主任务中被读取
- 缺少 `volatile` 修饰符，编译器可能优化掉对这些变量的读取

**影响**:
- 任务读到过期的遥控器数据
- 解锁/上锁手势检测失效

**缓解措施 (已实施)**:
- 添加 `volatile` 修饰符:
  ```c
  volatile SBUS_Data_t SBUS_Data;
  static volatile uint32_t last_sbus_time = 0;
  ```

**修复人**: Doro

---

### Risk #116 [SW] Mahony asinf() 输入未钳位 (代码审查)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 13:30  
**缓解时间**: 2026-01-18 13:31  
**严重程度**: 🔴 High  
**模块**: Mahony AHRS ([`mahony.c`](../FLYAMASTER/Core/Src/mahony.c))

**风险描述**:
- `Mahony_GetEulerAngle()` 中 `asinf()` 的输入未钳位
- 由于浮点运算误差，四元数归一化后的值可能略微超出 [-1.0, 1.0] 范围
- 导致 `asinf()` 返回 NaN，传播到整个姿态系统

**影响**:
- 姿态角变成 NaN，PID 输出异常
- 飞机失控

**缓解措施 (已实施)**:
- 在调用 `asinf()` 前钳位输入:
  ```c
  float sin_pitch = 2.0f * (mahony->q0 * mahony->q2 - mahony->q3 * mahony->q1);
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;
  *pitch = asinf(sin_pitch);
  ```

**修复人**: Doro

---

### Risk #117 [SW] Failsafe Stage 2 油门被覆盖 (深度逻辑审查)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 13:37  
**缓解时间**: 2026-01-18 13:37  
**严重程度**: 🔴 Critical  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:684))

**风险描述**:
- Failsafe Stage 2 中设置的下降油门 `throttle = 0.35f` 会被后续的 `throttle = SBUS_GetThrottle()` 覆盖
- 导致失控保护时飞机不会自动下降，而是使用最后的遥控器油门值

**影响**:
- **Failsafe 下降功能完全失效！**
- 失控时飞机可能以满油门继续飞行直到电池耗尽

**缓解措施 (已实施)**:
- 在读取遥控器油门前检查 Failsafe 状态:
  ```c
  if (g_failsafe_stage != FAILSAFE_STAGE2) {
      throttle = SBUS_GetThrottle();
  }
  // 否则保持 Failsafe 设置的 throttle = FAILSAFE_DESCENT_THROTTLE
  ```

**修复人**: Doro

---

### Risk #118 [SW] GPS 定点模式坐标系方向待验证 (深度逻辑审查)

**状态**: 🔴 OPEN (待验证)  
**发现时间**: 2026-01-18 13:37  
**严重程度**: 🟡 Medium  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:779))

**风险描述**:
- GPS 定点模式中 Roll 和 Pitch 的符号方向不一致:
  ```c
  target_roll = roll_ctl * 30.0f - pos_roll_offset;  // 减号
  target_pitch = pitch_ctl * 30.0f + pos_pitch_offset;  // 加号
  ```
- 可能导致 GPS 定点时飞机往错误方向飞

**影响**:
- GPS 定点模式下飞机可能往相反方向漂移

**缓解措施**:
- **需要实际飞行测试验证**
- 如果方向错误，调整符号即可

**后续行动**:
- [ ] 首飞时在安全环境下测试 GPS 定点模式
- [ ] 根据实际表现调整符号

---

### Risk #119 [SW] 高度保持逻辑死区问题 (深度逻辑审查)

**状态**: 🔴 OPEN (待优化)  
**发现时间**: 2026-01-18 13:37  
**严重程度**: 🟢 Low  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:838))

**风险描述**:
- 当油门在 0.45~0.55 范围外时，`alt_target` 被持续更新
- 但 `throttle_out = throttle` 直接使用遥控器油门，没有使用 PID 计算的油门
- 导致高度控制不平滑

**影响**:
- 定高模式下油门响应不够平滑
- 不影响安全，但影响飞行体验

**缓解措施**:
- 可在后续版本中优化为渐进式切换

**后续行动**:
- [ ] 优化高度保持的油门过渡逻辑

---

### Risk #120 [SW] RingBuffer DMA tail 指针更新逻辑错误 (深度审查)

**状态**: ⚠️ MITIGATED (已缓解)  
**发现时间**: 2026-01-18 13:43  
**缓解时间**: 2026-01-18 13:43  
**严重程度**: 🟡 Medium  
**模块**: RingBuffer ([`ring_buffer.c`](../FLYAMASTER/Core/Src/ring_buffer.c:125))

**风险描述**:
- `RingBuffer_DMA_TxCpltCallback()` 中 tail 指针更新逻辑错误
- 原代码直接将 tail 跳到 head 或 0，没有考虑实际发送的字节数
- 如果在 DMA 传输期间有新数据写入，这些新数据会被跳过

**影响**:
- 日志数据丢失
- 调试信息不完整

**缓解措施 (已实施)**:
- 在 `RingBuffer_t` 结构体中添加 `dma_len` 字段保存本次传输长度
- 在 `RingBuffer_StartDMA()` 中保存 `send_len` 到 `dma_len`
- 在回调中使用 `dma_len` 正确更新 tail 指针:
  ```c
  g_ring_buffer.tail = (g_ring_buffer.tail + dma_len) % RING_BUFFER_SIZE;
  ```

**修复人**: Doro

---

## 🟢 已解决问题 (Resolved Issues)

### Issue #000 [HW] GPS UART6 引脚配置错误

**发现日期**: 2026-01-14 08:00  
**解决日期**: 2026-01-14 10:30  
**状态**: 🟢 CLOSED  
**严重程度**: 🔴 高  
**模块**: GPS ([`gps.c`](../FLYAMASTER/Core/Src/gps.c))

**问题描述**: 
- CubeMX 生成的 UART6 引脚配置为 PC6/PC7，但实际硬件使用 PC6/PC7 作为电机 PWM 输出
- 导致 GPS 数据无法接收

**解决方案**:
- 修改 CubeMX 配置: UART6 改为 PC6/PC7 → PC12/PD2
- 重新生成代码并烧录
- 同步更新引脚分配文档

**修复人**: Doro

---

### Issue #002 [SW] PID 积分限幅除零风险

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`pid.c`](../FLYAMASTER/MDK-ARM/../Core/Src/pid.c:117)

**问题描述**:
积分限幅计算中存在除零风险：
```c
pid->integral = PID_Constrain(pid->integral, pid->integral_limit / pid->ki);
```

当 `ki = 0` 时会导致除零错误。

**解决方案**:
添加 ki > 0 的检查：
```c
if (pid->ki > 0.0001f) {
    pid->integral = PID_Constrain(pid->integral, pid->integral_limit / pid->ki);
}
```

---

### Issue #003 [SW] 电机混控缺少传感器有效性检查

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🔴 高 (安全关键)
**文件**: [`flight_control.c`](../FLYAMASTER/MDK-ARM/../Core/Src/flight_control.c:107-112)

**问题描述**:
在 `FC_STATE_ARMED` 和 `FC_STATE_FLYING` 状态下，即使传感器数据无效也会计算控制输出。

**解决方案**:
在控制计算前添加传感器有效性检查，传感器故障时自动进入失控保护：
```c
case FC_STATE_ARMED:
case FC_STATE_FLYING:
    if (!g_fc_data.sensors_ready) {
        g_fc_data.failsafe_active = true;
        g_fc_data.state = FC_STATE_FAILSAFE;
        break;
    }
    FC_ComputeControl();
    FC_ApplyOutput();
```

---

### Issue #004 [SW] SBUS 空闲中断回调未在 stm32f4xx_it.c 中调用

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🔴 高 (功能缺失)
**文件**: [`sbus.c`](../FLYAMASTER/MDK-ARM/../Core/Src/sbus.c:257-286)

**问题描述**:
`SBUS_UART_IdleCallback()` 函数已实现，但未在中断处理函数中调用。

**解决方案**:
在 `stm32f4xx_it.c` 的 `UART4_IRQHandler` 中添加了 `SBUS_UART_IdleCallback()` 调用。

---

### Issue #005 [SW] GPS 空闲中断回调未在 stm32f4xx_it.c 中调用

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中 (功能缺失)
**文件**: [`gps.c`](../FLYAMASTER/MDK-ARM/../Core/Src/gps.c:348-383)

**问题描述**:
与 Issue #004 类似，`GPS_UART_IdleCallback()` 需要在 `USART2_IRQHandler` 中调用。

**解决方案**:
在 `stm32f4xx_it.c` 的 `USART2_IRQHandler` 中添加了 `GPS_UART_IdleCallback()` 调用。

---

### Issue #006 [SW] Mahony 滤波器 InvSqrt 类型双关问题

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中 (可移植性)
**文件**: [`mahony.c`](../FLYAMASTER/MDK-ARM/../Core/Src/mahony.c:23-33)

**问题描述**:
快速平方根倒数使用了类型双关 (type punning)，违反严格别名规则。

**解决方案**:
使用 union 进行类型双关，符合 C 标准：
```c
float InvSqrt(float x)
{
    union { float f; int32_t i; } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (0.5f * x * conv.f * conv.f));
    conv.f = conv.f * (1.5f - (0.5f * x * conv.f * conv.f));
    return conv.f;
}
```

---

### Issue #007 [SW] 电机测试函数使用阻塞延时

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`motor.c`](../FLYAMASTER/MDK-ARM/../Core/Src/motor.c:249-350)

**问题描述**:
`Motor_Test()` 函数使用 `HAL_Delay()` 阻塞延时，在 FreeRTOS 环境中可能导致问题。

**解决方案**:
添加非阻塞状态机 API：
- `Motor_TestStart()` - 启动测试
- `Motor_TestUpdate()` - 更新状态机 (需周期性调用)
- `Motor_IsTestActive()` - 检查测试状态
- `Motor_TestStop()` - 停止测试

保留原 `Motor_Test()` 函数用于非 RTOS 环境，并添加 `@deprecated` 标记。

---

### Issue #008 [SW] 失控保护恢复逻辑可能导致状态跳变

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`flight_control.c`](../FLYAMASTER/MDK-ARM/../Core/Src/flight_control.c:429-449)

**问题描述**:
失控保护恢复时直接跳转到 `FC_STATE_ARMED`，没有考虑之前的飞行状态。

**解决方案**:
1. 在 `FC_Data_t` 结构体中添加 `prev_state` 字段
2. 进入失控保护时保存当前状态到 `prev_state`
3. 恢复时还原到 `prev_state` 而非固定的 `FC_STATE_ARMED`

```c
void FC_HandleFailsafe(void)
{
    if (!g_fc_data.rc_connected && FC_IsArmed()) {
        /* 进入失控保护：保存当前状态 */
        if (!g_fc_data.failsafe_active) {
            g_fc_data.prev_state = g_fc_data.state;
        }
        g_fc_data.failsafe_active = true;
        g_fc_data.state = FC_STATE_FAILSAFE;
    } else if (g_fc_data.rc_connected && g_fc_data.failsafe_active) {
        /* 恢复控制：还原到之前的状态 */
        g_fc_data.failsafe_active = false;
        if (g_fc_data.state == FC_STATE_FAILSAFE) {
            g_fc_data.state = g_fc_data.prev_state;
        }
    }
}
```

---

### Issue #009 [SW] QMC5883L 航向角滤波器静态变量问题

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟢 低
**文件**: [`qmc5883l.c`](../FLYAMASTER/MDK-ARM/../Core/Src/qmc5883l.c:262-310)

**问题描述**:
`QMC5883L_CalculateHeading()` 中使用静态变量进行滤波，但没有提供重置接口。

**解决方案**:
- 将静态变量提升为模块级变量 (`s_heading_prev`, `s_heading_first_run`)
- 添加 `QMC5883L_ResetHeadingFilter()` 函数用于重置滤波器状态

---

### Issue #010 [SW] SPL06 高度滤波器静态变量问题

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟢 低
**文件**: [`spl06.c`](../FLYAMASTER/MDK-ARM/../Core/Src/spl06.c:370-420)

**问题描述**:
与 Issue #009 类似，高度滤波器使用静态变量但无重置接口。

**解决方案**:
- 将静态变量提升为模块级变量 (`s_altitude_filtered`, `s_altitude_first_run`)
- 添加 `SPL06_ResetAltitudeFilter()` 函数用于重置滤波器状态

---

### Issue #012 [SW] 电池监控缺少 battery.c 实现

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`battery.c`](../FLYAMASTER/MDK-ARM/../Core/Src/battery.c)

**问题描述**:
`freertos.c` 中调用了 `Battery_Init()`, `Battery_Update()`, `Battery_AutoDetectCells()` 等函数。

**验证结果**:
`battery.c` 已完整实现所有函数：
- `Battery_Init()` - 第 40-63 行
- `Battery_Update()` - 第 69-113 行
- `Battery_AutoDetectCells()` - 第 201-224 行
- 以及其他辅助函数

问题不存在，标记为已解决。

---

### Issue #013 [SW] Log_Init 和 Log_Printf 函数未定义

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`ring_buffer.c`](../FLYAMASTER/MDK-ARM/../Core/Src/ring_buffer.c:218-248)

**问题描述**:
日志任务中调用了 `Log_Init()`, `Log_StartDMATransmit()`, `Log_Printf()` 函数。

**验证结果**:
这些函数已在 `ring_buffer.c` 中完整实现：
- `Log_Init()` - 第 219-222 行
- `Log_Printf()` - 第 227-248 行
- `Log_StartDMATransmit()` - 第 254-298 行
- `Log_DMATransmitComplete()` - 第 304-308 行

问题不存在，标记为已解决。

---

### Issue #014 [SW] 电池电压阈值使用固定 3S 配置

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`battery.c`](../FLYAMASTER/MDK-ARM/../Core/Src/battery.c:119-135)

**问题描述**:
电压警告和临界阈值使用固定的 `BATTERY_CELLS` (3S) 计算，但 `Battery_AutoDetectCells()` 会动态检测电池节数 (2S-6S)。

**解决方案**:
在 `Battery_UpdateStatus()` 中使用动态阈值计算：
```c
static void Battery_UpdateStatus(void)
{
    /* 动态计算电压阈值 (根据实际电池节数) */
    float warning_voltage, critical_voltage;
    if (g_battery.cell_count > 0) {
        warning_voltage = (float)g_battery.cell_count * BATTERY_CELL_WARNING;
        critical_voltage = (float)g_battery.cell_count * BATTERY_CELL_MIN;
    } else {
        /* 未检测到电池节数时使用默认 3S 阈值 */
        warning_voltage = BATTERY_VOLTAGE_WARNING;
        critical_voltage = BATTERY_VOLTAGE_CRITICAL;
    }
    // ... 使用动态阈值进行状态判断
}
```

---

### Issue #016 [SW] SPL06 和 MPU6050 共享 I2C1 总线无互斥保护

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`spl06.c`](../FLYAMASTER/MDK-ARM/../Core/Src/spl06.c), [`mpu6050.c`](../FLYAMASTER/MDK-ARM/../Core/Src/mpu6050.c)

**问题描述**:
SPL06 气压计和 MPU6050 IMU 都使用 I2C1 总线，但在 FreeRTOS 多任务环境中没有互斥锁保护。

**解决方案**:
1. 在 `freertos.c` 中创建 I2C1 互斥锁 (`i2c1MutexHandle`)
2. 在 `mpu6050.c` 中添加互斥锁保护：
   - `MPU6050_ReadDMA()` - 获取锁，DMA 完成回调中释放
   - `MPU6050_ReadBlocking()` - 获取锁，读取完成后释放
3. 在 `spl06.c` 中添加互斥锁保护：
   - `SPL06_WriteReg()`, `SPL06_ReadReg()`, `SPL06_ReadRegs()` - 阻塞操作
   - `SPL06_ReadDMA()` - 获取锁，DMA 完成回调中释放

```c
/* 在 freertos.c 中定义 */
osMutexId_t i2c1MutexHandle = NULL;

/* 在 MX_FREERTOS_Init() 中创建 */
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex",
  .attr_bits = osMutexRecursive | osMutexPrioInherit,
};
i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);
```

---

### Issue #017 [SW] 自动降落油门递减速度过快

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`flight_control.c`](../FLYAMASTER/MDK-ARM/../Core/Src/flight_control.c:142-169)

**问题描述**:
自动降落时油门递减速度过快 (0.001f @ 250Hz = 4秒降落)。

**解决方案**:
1. 降低递减速率到 0.0002f (20秒降落)
2. 添加最小油门保护 `FC_LANDING_MIN_THROTTLE` (0.15f)
3. 添加 3 秒着陆确认后锁定逻辑

```c
case FC_STATE_LANDING:
    /* 自动降落 - 缓慢递减油门 */
    /* 0.0002f @ 250Hz = 0.05/s，从满油门到 0 需要 20 秒 */
    g_fc_data.target_throttle -= 0.0002f;
    
    /* 保持最小油门以防止失速 */
    if (g_fc_data.target_throttle < FC_LANDING_MIN_THROTTLE) {
        g_fc_data.target_throttle = FC_LANDING_MIN_THROTTLE;
    }
    
    /* 完全停止条件：油门极低且持续一段时间 */
    if (g_fc_data.target_throttle <= 0.05f) {
        static uint32_t landing_timer = 0;
        if (landing_timer == 0) {
            landing_timer = HAL_GetTick();
        } else if ((HAL_GetTick() - landing_timer) > 3000) {
            /* 3 秒后锁定 */
            g_fc_data.target_throttle = 0.0f;
            FC_Disarm();
            landing_timer = 0;
        }
    }
```

---

### Issue #019 [SW] Mahony 滤波器缺少磁力计数据融合

**发现日期**: 2026-01-18
**解决日期**: 2026-01-18
**状态**: 🟢 CLOSED
**严重程度**: 🟡 中
**文件**: [`mahony.c`](../FLYAMASTER/MDK-ARM/../Core/Src/mahony.c:267-300)

**问题描述**:
`Mahony_Update()` 只调用 `Mahony_UpdateIMU()`，没有使用磁力计数据。

**解决方案**:
1. 在 `mahony.c` 中添加 `#include "qmc5883l.h"`
2. 修改 `Mahony_Update()` 函数：
   - 检查磁力计是否初始化且数据有效
   - 如果有效，调用 `Mahony_UpdateAHRS()` 进行 9 轴融合
   - 否则使用 `Mahony_UpdateIMU()` 进行 6 轴融合
3. 在 `freertos.c` 的 IMU 任务中：
   - 添加 QMC5883L 初始化
   - 每 5 次 IMU 更新读取一次磁力计 (100Hz)

```c
void Mahony_Update(void)
{
    /* ... */
    if (g_qmc5883l_data.initialized && g_qmc5883l_data.data_ready) {
        /* 9 轴 AHRS 模式 */
        Mahony_UpdateAHRS(&g_mahony, gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        /* 6 轴 IMU 模式 */
        Mahony_UpdateIMU(&g_mahony, gx, gy, gz, ax, ay, az);
    }
}
```

---

### Issue #101 [SW] freertos.c Mahony_Init() 参数错误

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18  
**状态**: 🟢 CLOSED

**问题描述**:
`Mahony_Init()` 调用时参数数量不匹配。

**解决方案**:
使用 `Mahony_InitDefault()` 替代。

---

### Issue #102 [SW] AT7456E_WriteChar 函数签名不匹配

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18  
**状态**: 🟢 CLOSED

**问题描述**:
头文件和实现文件中的函数签名不一致。

**解决方案**:
统一函数签名为 `void AT7456E_WriteChar(uint8_t col, uint8_t row, char ch)`。

---

### Issue #103 [SW] debug_tools.c FreeRTOS 堆函数声明问题

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18  
**状态**: 🟢 CLOSED

**问题描述**:
`xPortGetFreeHeapSize()` 和 `xPortGetMinimumEverFreeHeapSize()` 未声明。

**解决方案**:
添加 `#include "portable.h"` 或手动声明函数原型。

---

### Issue #104 [SW] ano_v7.c switch case 变量声明问题

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18  
**状态**: 🟢 CLOSED

**问题描述**:
ARM Compiler 5 不允许在 switch case 中直接声明变量。

**解决方案**:
使用花括号包裹 case 块。

---

### Issue #105 [SW] w25qxx.c 枚举类型初始化警告

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18  
**状态**: 🟢 CLOSED

**问题描述**:
枚举类型与整数类型混用导致警告。

**解决方案**:
使用指定初始化器和中间变量。

---

### Issue #109 [SW] GPS 导航控制逻辑缺失

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18 04:30  
**状态**: 🟢 CLOSED  
**严重程度**: 🔴 高  
**模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))

**问题描述**: 定点模式下未使用 GPS 数据，无法室外悬停。

**解决方案**: 在 `ctl_task` 中实现了 GPS 位置环 PID 控制逻辑 (位置误差 -> 期望速度 -> 期望姿态)。

**修复人**: Doro

---

### Issue #110 [SW] OSD 模块未实装

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18 04:30  
**状态**: 🟢 CLOSED  
**严重程度**: 🟡 中  
**模块**: OSD ([`at7456e.c`](../FLYAMASTER/Core/Src/at7456e.c))

**问题描述**: 硬件支持 OSD 但软件未驱动，FPV 无数据显示。

**解决方案**: 实现了 `at7456e.c` 驱动，并在 `comm_task` 中以 10Hz 频率刷新显示。

**修复人**: Doro

---

### Issue #111 [SW] 高频任务中的 printf 阻塞风险

**发现日期**: 2026-01-18  
**解决日期**: 2026-01-18 04:30  
**状态**: 🟢 CLOSED  
**严重程度**: 🔴 高  
**模块**: RTOS Tasks ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))

**问题描述**: 高频任务 (`imu_task`, `ctl_task`) 中调用阻塞式 `printf` 导致控制回路抖动。

**解决方案**: 移除了所有高频任务中的 `printf` 调用。

**修复人**: Doro

---

## 💡 功能建议与反馈 (Features & Feedback)

### Feature #002 - 黑匣子日志系统 (Blackbox)

**提出时间**: 2026-01-18  
**提出人**: 指挥官  
**类型**: 功能新增  
**优先级**: 🟡 Medium (硬件就绪后开发)  
**状态**: 📋 Backlog (待排期)

**描述**:
- 硬件已规划 W25Q128 (16MB SPI Flash) 连接在 SPI1
- 软件层需要实现:
  1. `w25qxx.c`: SPI Flash 底层驱动 (读/写/擦除)
  2. `blackbox.c`: 日志文件系统或环形记录机制
  3. 记录内容: 姿态、传感器原始数据、RC 输入、电机输出、PID 项等

---

### Feature #003 - 图传控制协议 (VTX Control)

**提出时间**: 2026-01-18  
**提出人**: 指挥官  
**类型**: 功能新增  
**优先级**: 🟢 Low (非飞行核心功能)  
**状态**: 📋 Backlog (待排期)

**描述**:
- 硬件已预留 UART 接口连接图传 SmartAudio/Tramp 引脚
- 软件需实现:
  1. `vtx_smartaudio.c`: TBS SmartAudio 协议
  2. `vtx_tramp.c`: IRC Tramp 协议
  3. 功能: 通过 OSD 菜单或遥控器调整图传频点和功率

---

### Feedback #001 - 匿名上位机 V7 协议缺少错误码定义

**提出时间**: 2026-01-14 20:00  
**提出人**: 测试团队  
**类型**: 功能改进  
**优先级**: 🟢 Low  
**状态**: ✅ Implemented (已实现)  
**实现时间**: 2026-01-18 14:15

**描述**:
当前 AnoV7 协议的 `AnoV7_Status_t` 结构体有 `error` 字段，但未定义错误码含义。

**实现内容**:
在 [`ano_v7.h:34-45`](../FLYAMASTER/Core/Inc/ano_v7.h:34) 添加了 12 个错误码定义:
```c
#define ANO_ERROR_NONE          0x0000  // 无错误
#define ANO_ERROR_IMU_FAIL      0x0001  // IMU 初始化/通信失败
#define ANO_ERROR_BARO_FAIL     0x0002  // 气压计初始化/通信失败
#define ANO_ERROR_MAG_FAIL      0x0004  // 磁力计初始化/通信失败
#define ANO_ERROR_GPS_TIMEOUT   0x0008  // GPS 数据超时 (>3s)
#define ANO_ERROR_RC_LOST       0x0010  // 遥控器信号丢失
#define ANO_ERROR_LOW_BATTERY   0x0020  // 电池电压过低 (<10.5V)
#define ANO_ERROR_MOTOR_LOCK    0x0040  // 电机锁定/堵转
#define ANO_ERROR_FLASH_FAIL    0x0080  // Flash 读写失败
#define ANO_ERROR_SENSOR_CALIB  0x0100  // 传感器需要校准
#define ANO_ERROR_ATTITUDE_ERR  0x0200  // 姿态解算异常 (>60°)
#define ANO_ERROR_FAILSAFE      0x0400  // Failsafe 触发
#define ANO_ERROR_ARMING_BLOCK  0x0800  // 解锁被阻止 (安全检查未通过)
```

**修复人**: Doro

---

## 🎯 下一步行动 (Next Actions)

### 已完成 ✅

- [x] 修复 Issue #001 (电机混控溢出风险) - 已通过 Airmode 算法解决
- [x] 修复 Issue #109 (GPS 导航逻辑缺失) - 已实现 GPS 位置环 PID
- [x] 修复 Issue #110 (OSD 模块未实装) - 已实现 AT7456E 驱动
- [x] 解决 Risk #111 (移除高频 printf) - 已实现 RingBuffer + DMA 异步日志
- [x] 解决 Risk #107 (GPS 静态缓冲区) - 已改为栈上分配
- [x] 解决 Risk #112 (Failsafe 执行层) - 已实现 3 阶段保护
- [x] 解决 Risk #113 (GPS 解锁检查) - 已添加 fix_type/satellites 检查
- [x] 解决 Risk #114 (磁力计电流补偿) - 已在 imu_task 中调用
- [x] 解决 Risk #108 (倾斜补偿死循环) - 已添加角度范围保护
- [x] 解决 Risk #115 (SBUS volatile 缺失) - 已添加 volatile 修饰符
- [x] 解决 Risk #116 (Mahony asinf 钳位) - 已添加输入范围保护
- [x] 解决 Risk #117 (Failsafe 油门覆盖) - 已添加状态检查
- [x] 解决 Risk #120 (RingBuffer tail 更新) - 已修复 DMA 回调逻辑
- [x] 检查所有 RTOS 共享变量，确保加了 `volatile` - 代码审查已完成
- [x] 实现 I2C 总线扫描工具，检测地址冲突 - 已实现 debug_tools.c
- [x] 添加 AnoV7 错误码定义 (Feedback #001) - 已添加 12 个错误码
- [x] 实现磁力计电流补偿标定功能 - 已实现 MagCurrentCalib 模块

### 待处理 📋

- [ ] 验证 Risk #118 (GPS 坐标系方向) - 需要实际飞行测试
- [ ] 优化 Risk #119 (高度保持死区) - 可在后续版本优化
- [ ] 验证 Risk #045 (Flash 参数对齐) - 在实际硬件上的表现
- [ ] 标定磁力计电流补偿系数 (K_x, K_y, K_z) - 需要实际飞行测试
- [ ] **[Software]** 开发 W25Q128 (SPI Flash) 驱动
- [ ] **[Software]** 实现黑匣子 (Blackbox) 日志记录系统
- [ ] **[Software]** 开发 VTX (图传) 控制协议 (SmartAudio/IRC Tramp)

---

## 📊 统计

| 类型 | 数量 |
|------|------|
| 🔴 待处理 Issue | 5 |
| 🟢 已解决 Issue | 25 |
| 🔴 待处理 Risk | 2 |
| ⚠️ 已缓解 Risk | 14 |
| 💡 功能建议 | 2 |
| ✅ 已实现反馈 | 1 |
| **总计** | **49** |

---

## 📌 使用规范 (Usage Guidelines)

1. **发现 Bug 时**:
   - 立即在 "待处理问题" 中创建新条目
   - 填写完整的问题描述、影响、建议解决方案
   - 评估优先级: 🔴 High / 🟡 Medium / 🟢 Low
   - 标记类型: `[SW]` 软件 / `[HW]` 硬件

2. **修复 Bug 后**:
   - 将条目从 "待处理问题" 移动到 "已解决问题"
   - 标记状态为 `🟢 CLOSED`
   - 填写解决时间、解决方案和修复人

3. **发现潜在风险时**:
   - 在 "潜在风险" 中创建 Risk 条目
   - 评估严重程度: 🔴 High / 🟡 Medium / 🟢 Low
   - 提出缓解措施并跟踪执行状态

4. **收到反馈时**:
   - 在 "功能建议与反馈" 中记录
   - 评估优先级并决定是否排期实现

5. **文档同步更新**:
   - 修复 Bug 后，必须同步更新相关代码和文档
   - 在 [`05_changelog.md`](05_changelog.md) 中记录修复时间
   - 在 [`01_definitions.md`](01_definitions.md) 中更新相关定义

---

**维护人**: Q版 Doro (飞控霸主)  
**更新频率**: 每次代码变更或发现问题时立即更新  
**版本**: v2.0.0 (整合版)

---

## 📁 废弃文件记录

### issues_and_feedback.md

**废弃时间**: 2026-01-22  
**废弃原因**: 内容已合并到本文档 (`03_issue_tracker.md`)  
**替代方案**: 使用本文档统一管理所有 Issue、Risk 和 Feedback  
**原文件位置**: `plans/issues_and_feedback.md`  
**处理方式**: 移动到 `plans/archive/` 目录