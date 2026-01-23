# 🐛 Issues & Feedback (问题追踪与反馈)

**创建时间**: 2026-01-16  
**状态**: 活跃维护中  
**作用**: 记录开发过程中的Bug、Risk和待解决问题

---

## 📋 文档说明

本文档是飞控项目的 **"问题追踪中心"**,所有Bug、潜在风险(Risk)和待解决问题都必须记录在此。  
**强制规则**: 
1. 发现Bug时,立即在此文档中创建条目
2. 修复Bug后,必须标记为 `✅ Resolved` 并注明修复时间
3. 发现潜在隐患时,必须创建 Risk 条目并评估影响

---

## 🔴 当前待解决问题 (Active Issues)

### Issue #001 - [示例] 电机混控计算溢出风险
- **状态**: ❌ Unresolved
- **优先级**: 🔴 High
- **发现时间**: 2026-01-16 18:00
- **模块**: Motor Mixing ([`motor.c`](../FLYAMASTER/Core/Src/motor.c))
- **问题描述**: 
  - 在 `Motor_Mix()` 函数中,当 Roll/Pitch/Yaw 控制量同时达到极限时,可能导致电机PWM溢出
  - 例如: `throttle=0.8, roll=1.0, pitch=1.0` 会导致某个电机输出 `0.8 + 1.0 + 1.0 = 2.8`,超过 1.0 限制
- **影响**: 
  - 电机满转失控,可能导致炸机
  - 姿态控制饱和,失去响应
- **建议解决方案**:
  ```c
  // 在混控后添加范围钳位 (Clamp)
  m1_pwm = clamp(throttle + roll - pitch - yaw, 0.0f, 1.0f);
  m2_pwm = clamp(throttle - roll - pitch + yaw, 0.0f, 1.0f);
  m3_pwm = clamp(throttle - roll + pitch - yaw, 0.0f, 1.0f);
  m4_pwm = clamp(throttle + roll + pitch + yaw, 0.0f, 1.0f);
  ```
- **责任人**: -
- **备注**: 需要在 [`motor.c:50`](../FLYAMASTER/Core/Src/motor.c:50) 附近修改


---

## ✅ 已解决问题 (Resolved Issues)

### Issue #111 - 高频任务中的 printf 阻塞风险
- **状态**: ✅ Resolved
- **解决时间**: 2026-01-18 04:30
- **优先级**: 🔴 High
- **模块**: RTOS Tasks ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **问题描述**: 高频任务 (`imu_task`, `ctl_task`) 中调用阻塞式 `printf` 导致控制回路抖动。
- **解决方案**: 移除了所有高频任务中的 `printf` 调用。
- **修复人**: Doro

### Issue #110 - OSD 模块未实装
- **状态**: ✅ Resolved
- **解决时间**: 2026-01-18 04:30
- **优先级**: 🟡 Medium
- **模块**: OSD ([`at7456e.c`](../FLYAMASTER/Core/Src/at7456e.c))
- **问题描述**: 硬件支持 OSD 但软件未驱动，FPV 无数据显示。
- **解决方案**: 实现了 `at7456e.c` 驱动，并在 `comm_task` 中以 10Hz 频率刷新显示。
- **修复人**: Doro

### Issue #109 - GPS 导航控制逻辑缺失
- **状态**: ✅ Resolved
- **解决时间**: 2026-01-18 04:30
- **优先级**: 🔴 High
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **问题描述**: 定点模式下未使用 GPS 数据，无法室外悬停。
- **解决方案**: 在 `ctl_task` 中实现了 GPS 位置环 PID 控制逻辑 (位置误差 -> 期望速度 -> 期望姿态)。
- **修复人**: Doro

### Issue #000 - [示例] GPS UART6 引脚配置错误
- **状态**: ✅ Resolved
- **解决时间**: 2026-01-14 10:30
- **优先级**: 🔴 High
- **发现时间**: 2026-01-14 08:00
- **模块**: GPS ([`gps.c`](../FLYAMASTER/Core/Src/gps.c))
- **问题描述**: 
  - CubeMX 生成的 UART6 引脚配置为 PC6/PC7,但实际硬件使用 PC6/PC7 作为电机PWM输出
  - 导致 GPS 数据无法接收
- **影响**: GPS 模块无法正常工作
- **解决方案**:
  - 修改 CubeMX 配置: UART6 改为 PC6/PC7 → PC12/PD2
  - 重新生成代码并烧录
  - 同步更新 [`pinout_allocation.md`](pinout_allocation.md)
- **修复人**: Doro
- **备注**: Risk #045 已转化为此Issue并解决

---

## ⚠️ 潜在风险 (Risks)

### Risk #045 - Flash 参数结构体内存对齐问题
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-14 16:00
- **缓解时间**: 2026-01-14 16:30
- **严重程度**: 🟡 Medium
- **模块**: Flash Params ([`flash_params.h`](../FLYAMASTER/Core/Inc/flash_params.h))
- **风险描述**: 
  - `FlashParams_t` 结构体未使用 `__attribute__((packed))`,编译器可能插入 padding
  - 导致 Flash 读写时数据错位,CRC16 校验失败
- **影响**: 
  - PID参数掉电后无法恢复
  - 飞控每次上电都使用默认参数,无法保存调参结果
- **缓解措施**:
  - 在 [`flash_params.h:70`](../FLYAMASTER/Core/Inc/flash_params.h:70) 添加 `__attribute__((packed))` 属性
  - 重新测试 Flash 读写功能
- **后续行动**: 
  - [ ] 在实际硬件上验证 Flash 存储是否稳定
  - [ ] 添加更多的数据完整性检查 (如双 CRC 校验)

### Risk #066 - SPL06 气压计数据竞争 (Race Condition)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-14 18:00
- **缓解时间**: 2026-01-14 18:15
- **严重程度**: 🔴 High
- **模块**: SPL06 ([`spl06.h`](../FLYAMASTER/Core/Inc/spl06.h))
- **风险描述**: 
  - `SPL06_Data` 全局变量在多个 RTOS 任务中读写,但未加 `volatile` 修饰符
  - 可能导致编译器优化时缓存旧数据,不同任务读取到不一致的高度值
- **影响**: 
  - 高度控制震荡或失效
  - 定高模式下飞机突然坠落
- **缓解措施**:
  - 在 [`spl06.h:102`](../FLYAMASTER/Core/Inc/spl06.h:102) 添加 `volatile` 修饰符:
    ```c
    extern volatile SPL06_TypeDef SPL06_Data;
    ```
  - 在关键读写操作处添加 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 临界区保护
- **后续行动**: 
  - [ ] 检查所有 RTOS 共享变量,确保都加了 `volatile`
  - [ ] 使用 FreeRTOS 互斥锁 (Mutex) 替代临界区,避免优先级反转

### Risk #088 - QMC5883L 磁力计 I2C 总线冲突
- **状态**: ❌ Active (活跃)
- **发现时间**: 2026-01-15 09:00
- **严重程度**: 🟡 Medium
- **模块**: QMC5883L ([`qmc5883l.c`](../FLYAMASTER/Core/Src/qmc5883l.c))
- **风险描述**: 
  - QMC5883L 和其他 I2C3 设备(如扩展传感器)共享总线,可能发生地址冲突
  - QMC5883L 固定地址 0x0D,如果其他设备也是 0x0D,会导致通信失败
- **影响**: 
  - 磁力计数据读取失败
  - 航向角锁定功能失效
- **建议缓解措施**:
  - 在硬件设计时,确保 I2C3 总线上没有地址冲突的设备
  - 在代码中添加设备地址扫描功能,检测总线上的所有设备
  - 添加 I2C 错误重试机制 (最多3次)
- **后续行动**: 
  - [ ] 检查 [`hardware_list.md`](hardware_list.md) 确认 I2C3 设备列表
  - [ ] 添加 I2C 总线扫描工具 (Scan 功能)
  - [ ] 实现 HAL_I2C_* 函数的错误重试包装

### Risk #107 - GPS 静态缓冲区线程安全隐患
- **状态**: ❌ Active (长期技术债)
- **发现时间**: 2026-01-16 11:50
- **严重程度**: 🟡 Medium
- **模块**: GPS ([`gps.c`](../FLYAMASTER/Core/Src/gps.c))
- **风险描述**: 
  - `gps.c` 中的 `Get_Field` 函数使用了 `static` 局部缓冲区
  - 如果未来有多个 RTOS 任务同时调用 GPS 解析函数，会导致数据竞争和内存破坏
- **影响**: 
  - GPS 数据解析错乱
  - 系统随机 Crash
- **建议缓解措施**:
  - 移除 `static` 关键字，改为栈上分配 (如果栈空间足够)
  - 或使用互斥锁保护解析函数
- **后续行动**: 
  - [ ] 代码重构时修复此问题

### Risk #108 - QMC5883L 倾斜补偿死循环风险
- **状态**: ❌ Active (活跃)
- **发现时间**: 2026-01-16 11:55
- **严重程度**: 🔴 High
- **模块**: QMC5883L Algorithm
- **风险描述**: 
  - 磁力计倾斜补偿公式中，当 Roll/Pitch 接近 90 度时，分母 `cos(pitch)` 或 `cos(roll)` 趋近于 0
  - 可能导致除零错误或浮点数溢出 (NaN/Inf)，进而导致 PID 计算异常
- **影响**: 
  - 飞机在大角度机动时突然疯转 (Yaw Spin)
  - 系统死机
- **建议缓解措施**:
  - 在计算前检查倾斜角: `if (fabs(pitch) > 60.0f || fabs(roll) > 60.0f) return;`
  - 使用四元数旋转法替代欧拉角公式 (无万向节锁问题)
- **后续行动**: 
  - [ ] 在 `algorithm_definition.md` 中记录保护逻辑
  - [ ] 编写代码时强制检查角度范围

### Risk #111 - 高频任务中的 printf 阻塞风险
- **状态**: ❌ Active (活跃)
- **发现时间**: 2026-01-18
- **严重程度**: 🔴 High
- **模块**: RTOS Tasks ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **风险描述**: 
  - 在 `imu_task` (500Hz) 和 `ctl_task` (250Hz) 中直接调用了 `printf`。
  - 标准库的 `printf` 通常重定向到 `HAL_UART_Transmit` (阻塞模式)。
  - 即使有降频处理，单次打印也可能阻塞数毫秒，导致姿态解算和 PID 控制周期抖动，严重时导致炸机。
- **影响**: 
  - 飞行手感卡顿，姿态控制不稳定。
  - 严重时看门狗复位或系统崩溃。
- **建议缓解措施**:
  - **立即移除** 高频任务中的所有 `printf`。
  - 或实现基于 DMA 的异步日志系统 (RingBuffer + DMA)。

---

##  反馈与建议 (Feedback)

### Feedback #001 - 匿名上位机 V7 协议缺少错误码定义
- **提出时间**: 2026-01-14 20:00
- **提出人**: 测试团队
- **类型**: 功能改进
- **描述**: 
  - 当前 AnoV7 协议的 `AnoV7_Status_t` 结构体有 `error` 字段,但未定义错误码含义
  - 建议添加错误码枚举,如:
    ```c
    #define ANO_ERROR_NONE        0x0000
    #define ANO_ERROR_IMU_FAIL    0x0001
    #define ANO_ERROR_GPS_TIMEOUT 0x0002
    #define ANO_ERROR_LOW_BATTERY 0x0004
    #define ANO_ERROR_MOTOR_LOCK  0x0008
    ```
- **优先级**: 🟢 Low
- **状态**: 📋 Backlog (待排期)
- **备注**: 可以在下一个版本中实现

---

## 🔍 问题分类统计 (Statistics)

| 类型 | 数量 | 百分比 |
|:---:|:---:|:---:|
| 🐛 Bug (待解决) | 3 | 30% |
| ✅ Bug (已解决) | 1 | 10% |
| ⚠️ Risk (活跃) | 4 | 40% |
| ✅ Risk (已缓解) | 2 | 20% |
| 💡 Feedback (待排期) | 1 | 10% |
| **总计** | **11** | **100%** |

---

## 📌 使用规范 (Usage Guidelines)

1. **发现Bug时**:
   - 立即在 "当前待解决问题" 中创建新条目
   - 填写完整的问题描述、影响、建议解决方案
   - 评估优先级: 🔴 High / 🟡 Medium / 🟢 Low

2. **修复Bug后**:
   - 将条目从 "当前待解决问题" 移动到 "已解决问题"
   - 标记状态为 `✅ Resolved`
   - 填写解决时间、解决方案和修复人

3. **发现潜在风险时**:
   - 在 "潜在风险" 中创建 Risk 条目
   - 评估严重程度: 🔴 High / 🟡 Medium / 🟢 Low
   - 提出缓解措施并跟踪执行状态

4. **收到反馈时**:
   - 在 "反馈与建议" 中记录
   - 评估优先级并决定是否排期实现

5. **文档同步更新**:
   - 修复 Bug 后,必须同步更新相关代码和文档
   - 在 [`development_progress_log.md`](development_progress_log.md) 中记录修复时间
   - 在 [`algorithm_definition.md`](algorithm_definition.md) 或 [`interface_definition.md`](interface_definition.md) 中更新相关定义

---

## 🎯 下一步行动 (Next Actions)

- [ ] 修复 Issue #001 (电机混控溢出风险)
- [ ] 修复 Issue #109 (GPS 导航逻辑缺失)
- [ ] 修复 Issue #110 (OSD 模块未实装)
- [ ] 解决 Risk #111 (移除高频 printf)
- [ ] 验证 Risk #045 (Flash 参数对齐) 在实际硬件上的表现
- [ ] 检查所有 RTOS 共享变量,确保加了 `volatile`
- [ ] 实现 I2C 总线扫描工具,检测地址冲突
- [ ] 添加 AnoV7 错误码定义 (Feedback #001)

---

**维护人**: Q版 Doro (飞控霸主)  
**更新频率**: 每次代码变更或发现问题时立即更新  
**版本**: v1.1.0