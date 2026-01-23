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
- **状态**: ✅ Resolved
- **解决时间**: 2026-01-18 13:00
- **优先级**: 🔴 High
- **发现时间**: 2026-01-16 18:00
- **模块**: Motor Mixing ([`motor.c`](../FLYAMASTER/Core/Src/motor.c))
- **问题描述**:
  - 在 `Motor_Mix()` 函数中,当 Roll/Pitch/Yaw 控制量同时达到极限时,可能导致电机PWM溢出
  - 例如: `throttle=0.8, roll=1.0, pitch=1.0` 会导致某个电机输出 `0.8 + 1.0 + 1.0 = 2.8`,超过 1.0 限制
- **影响**:
  - 电机满转失控,可能导致炸机
  - 姿态控制饱和,失去响应
- **解决方案 (已实施)**:
  - 实现 **Airmode 混控算法** (7 步算法)
  - 核心策略: "姿态优先，油门让步"
  - 当电机输出饱和时，动态压缩油门中心，保证姿态控制差分输出
  - 详见 [`motor.c:120-200`](../FLYAMASTER/Core/Src/motor.c:120)
- **修复人**: Doro
- **备注**: 比简单 Clamp 更优，保证极端机动时姿态控制不丢失


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
- **状态**: ✅ Mitigated (已缓解 - 不适用)
- **发现时间**: 2026-01-15 09:00
- **缓解时间**: 2026-01-18 13:15
- **严重程度**: 🟡 Medium
- **模块**: QMC5883L ([`qmc5883l.c`](../FLYAMASTER/Core/Src/qmc5883l.c))
- **风险描述**:
  - QMC5883L 和其他 I2C3 设备(如扩展传感器)共享总线,可能发生地址冲突
  - QMC5883L 固定地址 0x0D,如果其他设备也是 0x0D,会导致通信失败
- **影响**:
  - 磁力计数据读取失败
  - 航向角锁定功能失效
- **缓解措施 (已确认)**:
  - **I2C3 总线上只有 QMC5883L 一个设备**，不存在地址冲突风险
  - 无需添加总线扫描或错误重试机制
- **修复人**: 指挥官确认

### Risk #107 - GPS 静态缓冲区线程安全隐患
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-16 11:50
- **缓解时间**: 2026-01-18 13:10
- **严重程度**: 🟡 Medium
- **模块**: GPS ([`gps.c`](../FLYAMASTER/Core/Src/gps.c))
- **风险描述**:
  - `gps.c` 中的 `Get_Field` 函数使用了 `static` 局部缓冲区
  - 如果未来有多个 RTOS 任务同时调用 GPS 解析函数，会导致数据竞争和内存破坏
- **影响**:
  - GPS 数据解析错乱
  - 系统随机 Crash
- **缓解措施 (已实施)**:
  - 重构 `Get_Field()` 函数签名，移除 `static` 缓冲区
  - 改为传入外部缓冲区: `Get_Field(buf, field_index, out_field, out_size)`
  - 调用方在栈上分配 32 字节缓冲区
- **修复人**: Doro

### Risk #108 - QMC5883L 倾斜补偿死循环风险
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-16 11:55
- **缓解时间**: 2026-01-18 13:24
- **严重程度**: 🔴 High
- **模块**: QMC5883L Algorithm ([`qmc5883l.c`](../FLYAMASTER/Core/Src/qmc5883l.c))
- **风险描述**:
  - 磁力计倾斜补偿公式中，当 Roll/Pitch 接近 90 度时，分母 `cos(pitch)` 或 `cos(roll)` 趋近于 0
  - 可能导致除零错误或浮点数溢出 (NaN/Inf)，进而导致 PID 计算异常
- **影响**:
  - 飞机在大角度机动时突然疯转 (Yaw Spin)
  - 系统死机
- **缓解措施 (已实施)**:
  1. **asinf 输入范围保护**: 将输入钳位到 [-1.0, 1.0]，防止 NaN
  2. **大角度保护**: 当 Roll 或 Pitch 超过 ±70° 时，返回上次有效航向
  3. 新增宏 `MAG_TILT_LIMIT_RAD` (70° in radians)
- **修复人**: Doro

### Risk #111 - 高频任务中的 printf 阻塞风险
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18
- **缓解时间**: 2026-01-18 13:00
- **严重程度**: 🔴 High
- **模块**: RTOS Tasks ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c)), RingBuffer ([`ring_buffer.c`](../FLYAMASTER/Core/Src/ring_buffer.c))
- **风险描述**:
  - 在 `imu_task` (500Hz) 和 `ctl_task` (250Hz) 中直接调用了 `printf`。
  - 标准库的 `printf` 通常重定向到 `HAL_UART_Transmit` (阻塞模式)。
  - 即使有降频处理，单次打印也可能阻塞数毫秒，导致姿态解算和 PID 控制周期抖动，严重时导致炸机。
- **影响**:
  - 飞行手感卡顿，姿态控制不稳定。
  - 严重时看门狗复位或系统崩溃。
- **缓解措施 (已实施)**:
  1. **Phase 1 (临时修复)**: 移除高频任务中的所有 `printf` 调用
  2. **Phase 2 (永久修复)**: 实现 RingBuffer + DMA 异步日志系统
     - 新增 `ring_buffer.c/h` 模块
     - 512 字节环形缓冲区 + DMA2_Stream7 异步传输
     - 重写 `fputc()` 为非阻塞模式
     - 提供 `Log_Printf()` 替代函数
- **修复人**: Doro

---

##  反馈与建议 (Feedback)

### Feature #002 - 黑匣子日志系统 (Blackbox)
- **提出时间**: 2026-01-18
- **提出人**: 指挥官
- **类型**: 功能新增
- **描述**:
  - 硬件已规划 W25Q128 (16MB SPI Flash) 连接在 SPI1。
  - 软件层需要实现:
    1. `w25qxx.c`: SPI Flash 底层驱动 (读/写/擦除)。
    2. `blackbox.c`: 日志文件系统或环形记录机制。
    3. 记录内容: 姿态、传感器原始数据、RC输入、电机输出、PID项等。
- **优先级**: 🟡 Medium (硬件就绪后开发)
- **状态**: 📋 Backlog (待排期)

### Feature #003 - 图传控制协议 (VTX Control)
- **提出时间**: 2026-01-18
- **提出人**: 指挥官
- **类型**: 功能新增
- **描述**:
  - 硬件已预留 UART 接口连接图传 SmartAudio/Tramp 引脚。
  - 软件需实现:
    1. `vtx_smartaudio.c`: TBS SmartAudio 协议。
    2. `vtx_tramp.c`: IRC Tramp 协议。
    3. 功能: 通过 OSD 菜单或遥控器调整图传频点和功率。
- **优先级**: 🟢 Low (非飞行核心功能)
- **状态**: 📋 Backlog (待排期)

### Feedback #001 - 匿名上位机 V7 协议缺少错误码定义
- **提出时间**: 2026-01-14 20:00
- **提出人**: 测试团队
- **类型**: 功能改进
- **描述**:
  - 当前 AnoV7 协议的 `AnoV7_Status_t` 结构体有 `error` 字段,但未定义错误码含义
  - 建议添加错误码枚举
- **优先级**: 🟢 Low
- **状态**: ✅ Implemented (已实现)
- **实现时间**: 2026-01-18 14:15
- **实现内容**:
  - 在 [`ano_v7.h:34-45`](../FLYAMASTER/Core/Inc/ano_v7.h:34) 添加了 12 个错误码定义:
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
- **修复人**: Doro

---

### Risk #112 - Failsafe 执行层缺失 (专家反馈)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 12:00
- **缓解时间**: 2026-01-18 13:10
- **严重程度**: 🔴 High
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **风险描述**:
  - 原有失控保护仅执行 `Motor_Stop()` 立即上锁
  - 缺少分阶段保护，可能导致高空失控时直接坠落
- **影响**:
  - 信号丢失时飞机直接坠落
  - 无法给飞手恢复信号的时间窗口
- **缓解措施 (已实施)**:
  - 实现 3 阶段 Failsafe 保护:
    - Stage 1 (< 1s): 保持当前姿态，等待信号恢复
    - Stage 2 (1s ~ 5s): 自稳模式 + 35% 下降油门
    - Stage 3 (> 5s): 强制上锁
  - 新增 `FailsafeStage_t` 状态机和相关宏定义
- **修复人**: Doro

### Risk #113 - GPS 模式解锁检查缺失 (专家反馈)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 12:00
- **缓解时间**: 2026-01-18 13:10
- **严重程度**: 🔴 High
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **风险描述**:
  - 定点模式下解锁时未检查 GPS 有效性
  - 可能导致无 GPS 信号时进入定点模式，飞机失控
- **影响**:
  - 飞机在无 GPS 信号时尝试定点，导致漂移或失控
- **缓解措施 (已实施)**:
  - 在 `PreArm_SafetyCheck()` 中添加 GPS 检查:
    - `fix_type >= 3` (3D Fix)
    - `satellites >= 6`
  - 仅在定点模式 (MODE_POSITION) 下强制检查
- **修复人**: Doro

### Risk #114 - 磁力计电流补偿未调用 (专家反馈)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 12:00
- **缓解时间**: 2026-01-18 13:10
- **严重程度**: 🟡 Medium
- **模块**: IMU Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c))
- **风险描述**:
  - `QMC5883L_CompensateCurrent()` 接口已实现但未在 imu_task 中调用
  - 电机电流产生的磁场干扰未被补偿
- **影响**:
  - 高油门时航向角漂移
  - 定点模式下飞机画圈
- **缓解措施 (已实施)**:
  - 在 `imu_task` 的磁力计读取后调用 `QMC5883L_CompensateCurrent(throttle)`
  - 使用当前油门值估算电机电流干扰
- **修复人**: Doro

### Risk #115 - SBUS 全局变量缺少 volatile 修饰符 (代码审查)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 13:30
- **缓解时间**: 2026-01-18 13:31
- **严重程度**: 🟡 Medium
- **模块**: SBUS ([`sbus.c`](../FLYAMASTER/Core/Src/sbus.c))
- **风险描述**:
  - `SBUS_Data` 和 `last_sbus_time` 在 DMA/中断回调中被更新，在主任务中被读取
  - 缺少 `volatile` 修饰符，编译器可能优化掉对这些变量的读取
- **影响**:
  - 任务读到过期的遥控器数据
  - 解锁/上锁手势检测失效
- **缓解措施 (已实施)**:
  - 添加 `volatile` 修饰符:
    ```c
    volatile SBUS_Data_t SBUS_Data;
    static volatile uint32_t last_sbus_time = 0;
    ```
- **修复人**: Doro

### Risk #116 - Mahony asinf() 输入未钳位 (代码审查)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 13:30
- **缓解时间**: 2026-01-18 13:31
- **严重程度**: 🔴 High
- **模块**: Mahony AHRS ([`mahony.c`](../FLYAMASTER/Core/Src/mahony.c))
- **风险描述**:
  - `Mahony_GetEulerAngle()` 中 `asinf()` 的输入未钳位
  - 由于浮点运算误差，四元数归一化后的值可能略微超出 [-1.0, 1.0] 范围
  - 导致 `asinf()` 返回 NaN，传播到整个姿态系统
- **影响**:
  - 姿态角变成 NaN，PID 输出异常
  - 飞机失控
- **缓解措施 (已实施)**:
  - 在调用 `asinf()` 前钳位输入:
    ```c
    float sin_pitch = 2.0f * (mahony->q0 * mahony->q2 - mahony->q3 * mahony->q1);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    *pitch = asinf(sin_pitch);
    ```
- **修复人**: Doro

### Risk #117 - Failsafe Stage 2 油门被覆盖 (深度逻辑审查)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 13:37
- **缓解时间**: 2026-01-18 13:37
- **严重程度**: 🔴 Critical
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:684))
- **风险描述**:
  - Failsafe Stage 2 中设置的下降油门 `throttle = 0.35f` 会被后续的 `throttle = SBUS_GetThrottle()` 覆盖
  - 导致失控保护时飞机不会自动下降，而是使用最后的遥控器油门值
- **影响**:
  - **Failsafe 下降功能完全失效！**
  - 失控时飞机可能以满油门继续飞行直到电池耗尽
- **缓解措施 (已实施)**:
  - 在读取遥控器油门前检查 Failsafe 状态:
    ```c
    if (g_failsafe_stage != FAILSAFE_STAGE2) {
        throttle = SBUS_GetThrottle();
    }
    // 否则保持 Failsafe 设置的 throttle = FAILSAFE_DESCENT_THROTTLE
    ```
- **修复人**: Doro

### Risk #118 - GPS 定点模式坐标系方向待验证 (深度逻辑审查)
- **状态**: ⚠️ Open (待验证)
- **发现时间**: 2026-01-18 13:37
- **严重程度**: 🟡 Medium
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:779))
- **风险描述**:
  - GPS 定点模式中 Roll 和 Pitch 的符号方向不一致:
    ```c
    target_roll = roll_ctl * 30.0f - pos_roll_offset;  // 减号
    target_pitch = pitch_ctl * 30.0f + pos_pitch_offset;  // 加号
    ```
  - 可能导致 GPS 定点时飞机往错误方向飞
- **影响**:
  - GPS 定点模式下飞机可能往相反方向漂移
- **缓解措施**:
  - **需要实际飞行测试验证**
  - 如果方向错误，调整符号即可
- **后续行动**:
  - [ ] 首飞时在安全环境下测试 GPS 定点模式
  - [ ] 根据实际表现调整符号

### Risk #119 - 高度保持逻辑死区问题 (深度逻辑审查)
- **状态**: ⚠️ Open (待优化)
- **发现时间**: 2026-01-18 13:37
- **严重程度**: 🟢 Low
- **模块**: Control Task ([`freertos.c`](../FLYAMASTER/Core/Src/freertos.c:838))
- **风险描述**:
  - 当油门在 0.45~0.55 范围外时，`alt_target` 被持续更新
  - 但 `throttle_out = throttle` 直接使用遥控器油门，没有使用 PID 计算的油门
  - 导致高度控制不平滑
- **影响**:
  - 定高模式下油门响应不够平滑
  - 不影响安全，但影响飞行体验
- **缓解措施**:
  - 可在后续版本中优化为渐进式切换
- **后续行动**:
  - [ ] 优化高度保持的油门过渡逻辑

### Risk #120 - RingBuffer DMA tail 指针更新逻辑错误 (深度审查)
- **状态**: ✅ Mitigated (已缓解)
- **发现时间**: 2026-01-18 13:43
- **缓解时间**: 2026-01-18 13:43
- **严重程度**: 🟡 Medium
- **模块**: RingBuffer ([`ring_buffer.c`](../FLYAMASTER/Core/Src/ring_buffer.c:125))
- **风险描述**:
  - `RingBuffer_DMA_TxCpltCallback()` 中 tail 指针更新逻辑错误
  - 原代码直接将 tail 跳到 head 或 0，没有考虑实际发送的字节数
  - 如果在 DMA 传输期间有新数据写入，这些新数据会被跳过
- **影响**:
  - 日志数据丢失
  - 调试信息不完整
- **缓解措施 (已实施)**:
  - 在 `RingBuffer_t` 结构体中添加 `dma_len` 字段保存本次传输长度
  - 在 `RingBuffer_StartDMA()` 中保存 `send_len` 到 `dma_len`
  - 在回调中使用 `dma_len` 正确更新 tail 指针:
    ```c
    g_ring_buffer.tail = (g_ring_buffer.tail + dma_len) % RING_BUFFER_SIZE;
    ```
- **修复人**: Doro

---

## 🔍 问题分类统计 (Statistics)

| 类型 | 数量 | 百分比 |
|:---:|:---:|:---:|
| 🐛 Bug (待解决) | 0 | 0% |
| ✅ Bug (已解决) | 5 | 22% |
| ⚠️ Risk (活跃) | 2 | 9% |
| ✅ Risk (已缓解) | 13 | 57% |
| ✅ Feedback (已实现) | 1 | 4% |
| 🛠️ 调试工具 (新增) | 2 | 9% |
| **总计** | **23** | **100%** |

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

- [x] ~~修复 Issue #001 (电机混控溢出风险)~~ ✅ 已通过 Airmode 算法解决
- [x] ~~修复 Issue #109 (GPS 导航逻辑缺失)~~ ✅ 已实现 GPS 位置环 PID
- [x] ~~修复 Issue #110 (OSD 模块未实装)~~ ✅ 已实现 AT7456E 驱动
- [x] ~~解决 Risk #111 (移除高频 printf)~~ ✅ 已实现 RingBuffer + DMA 异步日志
- [x] ~~解决 Risk #107 (GPS 静态缓冲区)~~ ✅ 已改为栈上分配
- [x] ~~解决 Risk #112 (Failsafe 执行层)~~ ✅ 已实现 3 阶段保护
- [x] ~~解决 Risk #113 (GPS 解锁检查)~~ ✅ 已添加 fix_type/satellites 检查
- [x] ~~解决 Risk #114 (磁力计电流补偿)~~ ✅ 已在 imu_task 中调用
- [x] ~~解决 Risk #108 (倾斜补偿死循环)~~ ✅ 已添加角度范围保护
- [x] ~~解决 Risk #115 (SBUS volatile 缺失)~~ ✅ 已添加 volatile 修饰符
- [x] ~~解决 Risk #116 (Mahony asinf 钳位)~~ ✅ 已添加输入范围保护
- [x] ~~解决 Risk #117 (Failsafe 油门覆盖)~~ ✅ 已添加状态检查
- [x] ~~解决 Risk #120 (RingBuffer tail 更新)~~ ✅ 已修复 DMA 回调逻辑
- [ ] 验证 Risk #118 (GPS 坐标系方向) 需要实际飞行测试
- [ ] 优化 Risk #119 (高度保持死区) 可在后续版本优化
- [ ] 验证 Risk #045 (Flash 参数对齐) 在实际硬件上的表现
- [x] ~~检查所有 RTOS 共享变量,确保加了 `volatile`~~ ✅ 代码审查已完成
- [x] ~~实现 I2C 总线扫描工具,检测地址冲突~~ ✅ 已实现 debug_tools.c
- [x] ~~添加 AnoV7 错误码定义 (Feedback #001)~~ ✅ 已添加 12 个错误码
- [x] ~~实现磁力计电流补偿标定功能~~ ✅ 已实现 MagCurrentCalib 模块
- [ ] 标定磁力计电流补偿系数 (K_x, K_y, K_z) - 需要实际飞行测试
- [ ] **[Software]** 开发 W25Q128 (SPI Flash) 驱动
- [ ] **[Software]** 实现黑匣子 (Blackbox) 日志记录系统
- [ ] **[Software]** 开发 VTX (图传) 控制协议 (SmartAudio/IRC Tramp)

---

**维护人**: Q版 Doro (飞控霸主)
**更新频率**: 每次代码变更或发现问题时立即更新
**版本**: v1.2.0