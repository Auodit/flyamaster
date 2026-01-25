# 📜 版本更新日志 (Changelog)

**创建时间**: 2026-01-18
**状态**: 活跃维护中
**遵循规范**: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

---

## [HW v2.5.7] - 2026-01-25 (SPI CS Pin Swap & PCB Layout)

### Changed (变更)
- **SPI 片选引脚对调**:
  - PA4: FLASH_CS → **OSD_CS**
  - PC4: OSD_CS → **FLASH_CS**
- **PCB 布局**: 初版 PCB 布局完成，待优化 USB-C 插口位置

### Technical Details
- **变更原因**: 优化 PCB 走线，使 Flash 和 OSD 片选更靠近各自芯片
- **代码影响**: 需要在 CubeMX 中重新配置引脚标签

### Known Issues
- **CubeMX 配置不一致**: 当前 CubeMX 生成的代码中 FLASH_CS 仍在 PA4，需要用户手动修正
- **USB-C 插口**: PCB 空间不足，需要优化布局

---

## [HW v2.4.0] - 2026-01-23 (USB Detection Enhancement)

### Added (新增)
- **USB 插入检测引脚**: PB9 (USB_DETECT)
  - 从 D3 肖特基二极管阳极取信号
  - 串联 10kΩ 限流电阻
  - GPIO Input, Pull-down 配置
  - Active High (USB 插入时为高电平)

### Changed (变更)
- **USB 反灌保护电路**: 更新电路图，添加 USB_DETECT 信号路径
- **BOM 更新**: 新增 10kΩ 检测电阻 (R_detect) 和可选 100kΩ 下拉电阻 (R_pulldown)

### Technical Details
- **检测原理**: 从 USB VBUS 肖特基二极管阳极取信号，电池供电时二极管反向截止，PB9 为低电平
- **软件接口**: `USB_IsConnected()` 函数返回 USB 连接状态

---

## [FW v2.3.0] - 2026-01-23 (7-LED Professional Indicator System)

### Added (新增)

#### LED 指示灯驱动
- **LED 指示灯系统**: `led_indicator.c/h`
  - 7 颗 LED 专业级飞行仪表盘
  - 三组功能分区:
    - **核心系统组** (PC3, PC4): LED_SYS (蓝), LED_ERR (红)
    - **导航链路组** (PB0, PB1): LED_GPS (绿), LED_RX (黄)
    - **飞行任务组** (PB13-15): LED_MODE (白), LED_LOG (橙), LED_VTX (青)
  - 多种闪烁模式: 常亮/慢闪(1Hz)/快闪(4Hz)/脉冲
  - 高级状态 API: `LED_SetSysState()`, `LED_SetGpsState()` 等
  - 特效支持: 自检灯效, 解锁灯效

### Changed (变更)
- **硬件版本**: 对应 HW v2.3.0
- **PB0 引脚**: 从备用 ADC 改为 LED_GPS

---

## [HW v2.3.0] - 2026-01-23 (7-LED Professional Indicator System)

### Added (新增)
- **7 颗状态指示 LED**:
  - PC3: LED_SYS (蓝色) - 系统心跳
  - PC4: LED_ERR (红色) - 警告/错误
  - PB0: LED_GPS (绿色) - GPS 锁定状态
  - PB1: LED_RX (黄色) - 遥控接收状态
  - PB13: LED_MODE (白色) - 飞行模式
  - PB14: LED_LOG (橙色) - 黑匣子/日志
  - PB15: LED_VTX (青色) - 图传状态

### Changed (变更)
- **LED 电路设计**: 采用灌电流接法 (Sink Mode), 低电平亮
- **限流电阻**: 红/黄/橙色用 1kΩ, 蓝/绿/白/青色用 470-680Ω

---

## [FW v2.2.0] - 2026-01-23 (CRSF/ELRS Receiver & UART Refactor)

### Added (新增)

#### CRSF 协议驱动
- **CRSF 接收机驱动**: `crsf.c/h`
  - USART3 DMA 循环接收 (420000 baud, 8N1)
  - 完整 CRSF 帧解析 (同步字节 0xC8, CRC8 DVB-S2 校验)
  - 16 通道支持 (11-bit 分辨率, 172-1811 范围)
  - 双向通信支持 (遥测回传)
  - 新增 API:
    - `CRSF_Init()` - 初始化 DMA 接收
    - `CRSF_Process()` - 处理接收数据
    - `CRSF_GetChannel()` - 获取通道值
    - `CRSF_GetThrottle/Roll/Pitch/Yaw()` - 获取归一化控制量
    - `CRSF_IsFailsafe()` - 失控检测
    - `CRSF_SendTelemetry()` - 发送遥测数据

### Changed (变更)

#### 串口重新分配
- **USART1 (J3)**: K210 上位机 / 日志输出 (921600 baud)
- **USART2 (J2)**: GPS 模块 (115200 baud)
- **USART3 (焊盘)**: **ELRS 接收机 (CRSF 协议, 420000 baud)** ← 新增
- **UART4 (J6)**: 图传 SmartAudio (4800 baud, 半双工)
- **UART5 (J5)**: 光流模块 (115200 baud)

#### I2C 总线合并
- **I2C2 移除**: QMC5883L 磁力计从 I2C2 迁移至 I2C1 共享总线
- **PB10/PB11 释放**: 原 I2C2 引脚现用于 USART3 (ELRS)

#### DMA 通道调整
- **DMA1_Stream1**: 新增 USART3_RX (CRSF 接收, Circular, Very High)
- **DMA1_Stream3**: 新增 USART3_TX (CRSF 遥测, Normal, Medium)
- **DMA1_Stream2**: 移除 (原 UART4_RX SBUS)

### Removed (移除)
- **SBUS 协议**: 从 UART4 移除，由 CRSF 协议替代
  - `sbus.c/h` 保留但不再使用
  - UART4 现专用于 SmartAudio VTX 控制

### Technical Details
- **CRSF 协议优势**:
  - 更高波特率: 420kbps vs SBUS 100kbps
  - 双向通信: 支持遥测回传 (电池电压、RSSI、GPS 等)
  - 无需反相器: 标准 UART 电平
  - CRC 校验: 更可靠的数据传输
- **硬件版本**: 对应 HW v2.2.0

---

## [HW v2.2.0] - 2026-01-23 (UART Refactor & I2C Consolidation)

### Changed (变更)
- **串口重新分配**:
  - USART3 (PB10/PB11) 用于 ELRS 接收机 (CRSF 协议)
  - UART4 专用于 SmartAudio VTX 控制
- **I2C 总线合并**: 移除 I2C2，所有传感器使用 I2C1
  - MPU6050 + SPL06 + QMC5883L 共享 I2C1 (400kHz)
- **DMA 通道调整**:
  - DMA1_Stream1: USART3_RX (CRSF)
  - DMA1_Stream3: USART3_TX (遥测)

### Removed (移除)
- **I2C2 总线**: 释放 PB10/PB11 给 USART3

---

## [FW v2.1.5] - 2026-01-18 (I2C Mutex & Magnetometer Fusion)

### Fixed (修复)
- **Issue #016 (I2C1 互斥保护)**: 为 SPL06 和 MPU6050 共享的 I2C1 总线添加互斥锁保护
  - 在 `freertos.c` 中创建 `i2c1MutexHandle` 互斥锁 (递归 + 优先级继承)
  - 在 `mpu6050.c` 中添加 DMA 和阻塞读取的互斥锁保护
  - 在 `spl06.c` 中添加所有 I2C 操作的互斥锁保护
  - 防止多任务环境下的 I2C 总线冲突
- **Issue #019 (磁力计融合)**: 修复 Mahony 滤波器缺少磁力计数据融合问题
  - 在 `mahony.c` 中添加 QMC5883L 数据融合支持
  - 修改 `Mahony_Update()` 自动检测磁力计状态并选择 6 轴/9 轴模式
  - 在 IMU 任务中添加 QMC5883L 初始化和 100Hz 读取
  - 解决偏航角 (Yaw) 漂移问题

### Added (新增)
- **freertos.c**: 添加 I2C1 互斥锁 (`i2c1MutexHandle`)
- **freertos.c**: 添加 QMC5883L 磁力计初始化和读取 (100Hz)
- **mpu6050.c**: 添加 `extern osMutexId_t i2c1MutexHandle` 和互斥锁操作
- **spl06.c**: 添加 `extern osMutexId_t i2c1MutexHandle` 和互斥锁操作
- **mahony.c**: 添加 `#include "qmc5883l.h"` 和磁力计融合逻辑

### Changed (变更)
- **Mahony_Update()**: 从纯 IMU 模式升级为自动 IMU/AHRS 切换模式
- **IMU 任务**: 添加磁力计分频读取 (500Hz / 5 = 100Hz)

---

## [FW v2.1.4] - 2026-01-18 (Safety & Stability Patch)

### Fixed (修复)
- **Issue #008 (失控保护状态跳变)**: 修复失控保护恢复时状态跳变问题
  - 在 `FC_Data_t` 中添加 `prev_state` 字段保存进入失控保护前的状态
  - 恢复时还原到之前的状态 (FLYING/ARMED) 而非固定跳转到 ARMED
  - 避免从 FLYING 状态直接跳变到 ARMED 导致的控制不连续
- **Issue #014 (电池电压阈值固定)**: 修复电池电压阈值使用固定 3S 配置问题
  - 在 `Battery_UpdateStatus()` 中使用动态阈值计算
  - 根据 `g_battery.cell_count` 自动调整警告和临界电压
  - 支持 2S-6S 电池正确的过放保护
- **Issue #017 (自动降落过快)**: 修复自动降落油门递减速度过快问题
  - 降低递减速率从 0.001f 到 0.0002f (20秒降落)
  - 添加最小油门保护 `FC_LANDING_MIN_THROTTLE` (0.15f)
  - 添加 3 秒着陆确认后锁定逻辑

### Added (新增)
- **flight_control.h**: 添加降落参数宏定义
  - `FC_LANDING_MIN_THROTTLE` - 降落最小油门 (0.15f)
  - `FC_LANDING_RATE` - 降落油门递减率 (0.0002f)
- **FC_Data_t**: 添加 `prev_state` 字段用于失控保护状态恢复

---

## [FW v2.1.3] - 2026-01-18 (Code Quality Improvement)

### Added (新增)
- **motor.c**: 添加非阻塞电机测试 API
  - `Motor_TestStart()` - 启动测试
  - `Motor_TestUpdate()` - 更新状态机
  - `Motor_IsTestActive()` - 检查测试状态
  - `Motor_TestStop()` - 停止测试
- **qmc5883l.c**: 添加 `QMC5883L_ResetHeadingFilter()` 航向角滤波器重置函数
- **spl06.c**: 添加 `SPL06_ResetAltitudeFilter()` 高度滤波器重置函数

### Fixed (修复)
- **Issue #007**: 电机测试函数阻塞延时问题 - 改为非阻塞状态机
- **Issue #009**: QMC5883L 航向角滤波器静态变量无重置接口
- **Issue #010**: SPL06 高度滤波器静态变量无重置接口
- **Issue #013**: 验证 Log_Init 等函数已在 ring_buffer.c 中实现

### Changed (变更)
- **motor.c**: 原 `Motor_Test()` 函数标记为 `@deprecated`，建议使用非阻塞 API
- **qmc5883l.c**: 滤波器静态变量提升为模块级变量，支持外部重置
- **spl06.c**: 滤波器静态变量提升为模块级变量，支持外部重置

---

## [FW v2.1.2] - 2026-01-18 (Security Patch & Code Review)

### Fixed (修复)

#### 安全关键修复
- **Issue #002 (PID 除零风险)**: 在 `pid.c` 中添加 `ki > 0.0001f` 检查，防止积分限幅除零
- **Issue #003 (传感器故障保护)**: 在 `flight_control.c` 中添加传感器有效性检查
  - 飞行中传感器故障自动进入 Failsafe 模式
  - 防止使用无效数据进行控制计算
- **Issue #004 (SBUS 中断缺失)**: 在 `stm32f4xx_it.c` 中添加 `SBUS_UART_IdleCallback()` 调用
  - 修复 SBUS 接收机无法正常工作的问题
- **Issue #005 (GPS 中断缺失)**: 在 `stm32f4xx_it.c` 中添加 `GPS_UART_IdleCallback()` 调用
  - 修复 GPS 数据无法接收的问题
- **Issue #006 (InvSqrt 类型双关)**: 在 `mahony.c` 中使用 union 替代指针类型双关
  - 符合 C 标准，避免严格别名规则违反
  - 提高代码可移植性

### Added (新增)
- **问题追踪系统**: 创建 `03_issue_tracker.md`
  - 记录 18 个已识别问题 (10 个已解决，8 个待处理)
  - 区分软件 BUG 和硬件问题
  - 包含严重程度分级和修复建议

### Technical Details
- **代码审查范围**: freertos.c, flight_control.c, motor.c, sbus.c, mahony.c, pid.c, mpu6050.c, gps.c, qmc5883l.c, spl06.c, battery.c
- **修复的安全问题**: 5 个 (Issue #002-006)
- **待处理问题**: 8 个 (Issue #001, #007-013)

---

## [HW v2.0.0] - 2026-01-18 (STM32F405RGT6 Migration)

### Changed (变更)
- **MCU 迁移**: 从 STM32F407VET6 (LQFP-100) 迁移至 **STM32F405RGT6** (LQFP-64)
  - Flash: 512KB → **1MB**
  - GPIO: 82 → **51** (引脚减少，需重新分配)
  - 价格: ~¥35 → **~¥25**
- **引脚重分配**:
  - OSD_CS: PA4 → **PB12** (PA4 用于 FLASH_CS)
  - GPS: UART6 (PC6/PC7) → **USART2** (PA2/PA3)
  - 电机 PWM: TIM3/TIM4 → **TIM8** (PC6-PC9)
- **DMA 通道调整**: 完整重新分配 DMA1/DMA2 通道

### Technical Details
- **时钟配置**: HSE 8MHz, PLL M=4 N=168 P=2 Q=7, SYSCLK=168MHz
- **FreeRTOS 任务**: 6 个任务，总堆栈 30KB
- **USB**: CDC 虚拟串口，Rx/Tx 缓冲区各 2048 字节

---

## [FW v2.1.1] - 2026-01-18 (Hardware Config Update & Build Fix)

### Fixed (修复)
- **FreeRTOS 编译错误**: 修复 `reent.h: No such file or directory` 错误
  - **原因**: `configUSE_NEWLIB_REENTRANT = 1`，但 Keil MDK 使用 ARM C Library，不支持 newlib
  - **修复**: 将 `configUSE_NEWLIB_REENTRANT` 改为 `0`
- **at7456e.c 文件丢失**: 重新写入完整的 OSD 驱动代码 (~300 行)
- **ano_v7.c switch case 警告**: 用花括号包裹 case 5 块，解决变量声明跳过警告
- **w25qxx.c 枚举类型警告**: 使用指定初始化器和中间变量消除 ARM Compiler 5 警告

### Changed (变更)
- **HAL 时基变更**: 从 TIM1 改为 TIM4
  - TIM4 现在用作 FreeRTOS/HAL 系统时基
  - TIM4_CH3 已注销 (不再用于 WS2812)
- **WS2812 驱动迁移**: 从 TIM4_CH4 (PB9) 恢复到 TIM1_CH1 (PA8)
  - 更新 `ws2812.h`: PWM 参数调整 (168MHz 时钟)
    - `WS2812_PWM_HI = 134` (0.8us)
    - `WS2812_PWM_LO = 67` (0.4us)
  - 更新 `ws2812.c`: `WS2812_TIM = htim1`, `WS2812_TIM_CHANNEL = TIM_CHANNEL_1`
- **硬件设计文档**: 更新至 v2.0.0，完整 STM32F405 CubeMX 配置

---

## [FW v2.1.0] - 2026-01-18 (Battery Monitor & RSSI Enhancement)

### Added (新增)

#### 电池监控模块
- **电池监控驱动**: `battery.c/h`
  - ADC1 双通道 DMA 循环采样 (PC0 电压 + PC5 电流)
  - 实时电压/电流监测 + 低通滤波
  - mAh 消耗积分计算 (精确油量表)
  - 自动电池节数检测 (2S-6S)
  - 多级报警系统:
    - `BATTERY_WARNING`: 低电压警告 (间歇蜂鸣)
    - `BATTERY_CRITICAL`: 电压临界 (急促蜂鸣)
    - `BATTERY_OVERCURRENT`: 过流保护 (连续蜂鸣)
  - 电流标定接口 (`Battery_SetCurrentCalibration()`)

#### SBUS RSSI 增强
- **SBUS v1.1.0**: 支持 CH16 数字 RSSI
  - 现代接收机 (ELRS, Crossfire, FrSky) 通过 CH16 传输 RSSI
  - 新增 `rssi`, `link_quality`, `rssi_warning`, `rssi_critical` 字段
  - 新增 API: `SBUS_GetRSSI()`, `SBUS_GetLinkQuality()`, `SBUS_IsRSSIWarning()`
  - 淘汰模拟 RSSI 线 (PC5 释放给电流检测)

### Changed (变更)
- **defaultTask 频率提升**: 从 2Hz 提升到 10Hz (电池监控需要更高采样率)
- **OSD 显示增强**: 新增电压/电流/mAh/RSSI 显示
- **PC5 引脚重分配**: 从模拟 RSSI 改为电流检测 (ADC1_IN15)
- **硬件设计文档**: 更新至 v1.2.0，添加 PC5 电流检测引脚

### Technical Details
- **电流标定公式**: `I = (V_adc - Offset) × Scale`
- **mAh 积分公式**: `mAh += I × dt × 0.277778`
- **RSSI 映射**: CH16 (172-1811) → 0-100%

---

## [FW v2.0.0] - 2026-01-18 (Complete Driver Rewrite - F405 终极版)

### Added (新增)

#### 核心驱动模块
- **RingBuffer 异步日志系统**: `ring_buffer.c/h`
  - 512 字节环形缓冲区 + USART1 DMA 异步传输
  - `Log_Printf()` 非阻塞 printf，耗时 < 10us
  - 彻底解决高频任务中的控制回路抖动问题

- **MPU6050 IMU 驱动**: `mpu6050.c/h`
  - I2C1 DMA 非阻塞读取 (500Hz)
  - 14 字节批量读取 (加速度 + 陀螺仪 + 温度)
  - 自动量程配置 (±2000°/s, ±8g)

- **Mahony 姿态解算**: `mahony.c/h`
  - 四元数互补滤波算法
  - 可调 Kp/Ki 参数 (默认 2.0/0.005)
  - 欧拉角输出 (Roll, Pitch, Yaw)

- **SBUS 接收机驱动**: `sbus.c/h`
  - UART4 DMA 循环接收 (100k, Even, 2Stop)
  - 25 字节帧解析，16 通道支持
  - Failsafe 和 Frame Lost 检测

#### 控制模块
- **串级 PID 控制器**: `pid.c/h`
  - 角度环 + 角速度环双环控制
  - Anti-windup 条件积分
  - 输出限幅和微分滤波

- **电机混控模块**: `motor.c/h`
  - TIM8 400Hz PWM 输出 (PC6-PC9)
  - X 型机架混控算法
  - Airmode 动态油门压缩

- **飞行控制核心**: `flight_control.c/h`
  - 多飞行模式支持 (自稳/定高/定点/返航)
  - 解锁/上锁状态机
  - 分阶段 Failsafe 保护

#### 外设驱动
- **WS2812 RGB 灯驱动**: `ws2812.c/h`
  - TIM1_CH1 (PA8) + DMA2 Stream1
  - 解决与 UART4 的 DMA 冲突
  - 支持多种灯效模式

- **AT7456E OSD 驱动**: `at7456e.c/h`
  - SPI1 通信 (与 Flash 共享)
  - 字符显示和显存刷新
  - 飞行数据实时显示

- **GPS 模块驱动**: `gps.c/h`
  - USART2 DMA 循环接收 (115200)
  - NMEA-0183 协议解析 (GGA, RMC, VTG)
  - Haversine 距离和方位计算
  - 相对位置计算 (Home Point)

- **QMC5883L 磁力计驱动**: `qmc5883l.c/h`
  - I2C2 独立总线 (避免电机干扰)
  - 硬铁/软铁校准算法
  - 倾斜补偿航向计算

- **SPL06 气压计驱动**: `spl06.c/h`
  - I2C1 共享总线 (与 MPU6050)
  - 温度补偿气压计算
  - 高度和垂直速度输出

- **W25Q128 Flash 驱动**: `w25qxx.c/h`
  - SPI1 通信 (与 OSD 共享)
  - 扇区/块/全片擦除
  - 黑匣子日志循环存储

- **SmartAudio VTX 控制**: `smartaudio.c/h`
  - UART4_TX 半双工模式 (4800 bps)
  - SmartAudio V2.1 协议
  - 功率/频道/PIT 模式控制

#### FreeRTOS 任务架构
- **imu_task** (Realtime, 500Hz): IMU DMA 读取 + Mahony 解算
- **ctl_task** (High, 250Hz): SBUS 处理 + PID 控制 + 电机混控
- **log_task** (BelowNormal, 100Hz): RingBuffer DMA 日志发送
- **comm_task** (Normal, 10Hz): OSD 刷新 + 上位机通信
- **gps_task** (BelowNormal, 10Hz): GPS 解析 + 导航计算
- **defaultTask** (Normal, 2Hz): LED 心跳 + 看门狗

### Changed (变更)
- **RGB 灯引脚迁移**: 从 PB1 (TIM3_CH4) 迁移到 PA8 (TIM1_CH1)
  - 解决与 UART4 SBUS 的 DMA1 通道冲突
- **DMA 架构重构**: 完整的 DMA1/DMA2 通道分配
  - DMA1: I2C1, UART4 (SBUS), USART2 (GPS), I2C2, UART5
  - DMA2: SPI1, TIM1 (RGB), USART1 (Log), ADC1

### Technical Specifications
- **MCU**: STM32F405RGT6 @ 168MHz
- **RAM**: 192KB (128KB System + 64KB CCM)
- **Flash**: 1MB
- **RTOS**: FreeRTOS CMSIS_V2, Heap 30KB

---

## [v1.5.0] - 2026-01-18 (Debug Tools & Error Codes)

### Added (新增)
- **调试工具模块**: 新增 `debug_tools.c` 和 `debug_tools.h`
  - **I2C 总线扫描**: `I2C_ScanBus()` 和 `I2C_ScanAllBuses()` 函数
    - 扫描地址范围 0x08 ~ 0x77
    - 自动识别已知设备 (MPU6050, QMC5883L, SPL06 等)
    - 支持同时扫描 I2C1, I2C2, I2C3 三条总线
  - **磁力计电流补偿标定**: `MagCurrentCalib_*` 系列函数
    - 两阶段数据采集 (低油门 + 高油门)
    - 自动计算 K_x, K_y, K_z 补偿系数
    - 打印可直接复制到代码的宏定义
- **AnoV7 错误码定义**: 在 `ano_v7.h` 中添加 12 个错误码宏
  - `ANO_ERROR_NONE` (0x0000) - 无错误
  - `ANO_ERROR_IMU_FAIL` (0x0001) - IMU 失败
  - `ANO_ERROR_BARO_FAIL` (0x0002) - 气压计失败
  - `ANO_ERROR_MAG_FAIL` (0x0004) - 磁力计失败
  - `ANO_ERROR_GPS_TIMEOUT` (0x0008) - GPS 超时
  - `ANO_ERROR_RC_LOST` (0x0010) - 遥控器丢失
  - `ANO_ERROR_LOW_BATTERY` (0x0020) - 低电压
  - `ANO_ERROR_MOTOR_LOCK` (0x0040) - 电机锁定
  - `ANO_ERROR_FLASH_FAIL` (0x0080) - Flash 失败
  - `ANO_ERROR_SENSOR_CALIB` (0x0100) - 需要校准
  - `ANO_ERROR_ATTITUDE_ERR` (0x0200) - 姿态异常
  - `ANO_ERROR_FAILSAFE` (0x0400) - Failsafe 触发
  - `ANO_ERROR_ARMING_BLOCK` (0x0800) - 解锁阻止

### Changed (变更)
- **Feedback #001 已完成**: 匿名上位机 V7 协议错误码定义已实现

---

## [v1.4.0] - 2026-01-18 (Deep Logic Review & Code Audit)

### Fixed (修复)
- **Risk #115 (SBUS volatile 缺失)**: 为 `SBUS_Data` 和 `last_sbus_time` 添加 `volatile` 修饰符
  - 防止编译器优化导致的数据不一致
  - 确保 ISR 和任务之间的数据同步
- **Risk #116 (Mahony asinf 钳位)**: 在 `Mahony_GetEulerAngle()` 中添加 `asinf()` 输入范围保护
  - 钳位输入到 [-1.0, 1.0] 范围
  - 防止浮点误差导致 NaN 传播
- **Risk #117 (Failsafe 油门覆盖) - CRITICAL**: 修复 Failsafe Stage 2 油门被遥控器油门覆盖的严重漏洞
  - **问题**: `throttle = FAILSAFE_DESCENT_THROTTLE` 被后续 `throttle = SBUS_GetThrottle()` 覆盖
  - **影响**: Failsafe 下降功能完全失效
  - **修复**: 在读取遥控器油门前检查 Failsafe 状态

### Added (新增)
- **全面代码审查**: 完成 14 个核心模块的代码质量和安全性审查
  - freertos.c, motor.c, pid.c, gps.c, qmc5883l.c, spl06.c
  - filter.c, sbus.c, mahony.c, altitude.c, optical_flow.c
  - flash_params.c, buzzer.c, ano_v7.c

### Known Issues (已知问题)
- **Risk #118**: GPS 定点模式坐标系方向待验证 (需要实际飞行测试)
- **Risk #119**: 高度保持逻辑死区问题 (可在后续版本优化)

---

## [v1.3.0] - 2026-01-18 (Critical Safety Patches - 专家反馈)

### Added (新增)
- **分阶段 Failsafe 保护**: 实现 3 阶段失控保护机制
  - Stage 1 (< 1s): 保持当前姿态，等待信号恢复
  - Stage 2 (1s ~ 5s): 自稳模式 + 35% 下降油门
  - Stage 3 (> 5s): 强制上锁
  - 新增 `FailsafeStage_t` 状态机和相关宏定义
- **GPS 解锁检查**: 在定点模式下强制检查 GPS 有效性
  - `fix_type >= 3` (3D Fix)
  - `satellites >= 6`
  - 防止无 GPS 信号时进入定点模式

### Fixed (修复)
- **Risk #107 (GPS 静态缓冲区)**: 重构 `Get_Field()` 函数
  - 移除 `static` 缓冲区，改为传入外部缓冲区
  - 调用方在栈上分配 32 字节缓冲区，线程安全
- **Risk #112 (Failsafe 执行层)**: 从简单 `Motor_Stop()` 升级为分阶段保护
- **Risk #113 (GPS 解锁检查)**: 添加 `PreArm_SafetyCheck()` 中的 GPS 检查
- **Risk #114 (磁力计电流补偿)**: 在 `imu_task` 中调用 `QMC5883L_CompensateCurrent(throttle)`
- **Risk #108 (倾斜补偿死循环)**: 添加 QMC5883L 倾斜补偿角度保护
  - 限制 `asinf` 输入范围防止 NaN
  - 当 Roll/Pitch > 70° 时返回上次有效航向，防止万向节锁导致的航向突变

### Changed (变更)
- **Failsafe 逻辑**: 从立即上锁改为分阶段保护，给飞手恢复信号的时间窗口
- **PreArm_SafetyCheck**: 增加 GPS 有效性检查 (仅定点模式)

---

## [v1.2.0] - 2026-01-18 (Physics & Safety Hardening)

### Added (新增)
- **RingBuffer 异步日志模块**: 新增 `ring_buffer.c` 和 `ring_buffer.h`
  - 512 字节环形缓冲区 + DMA2_Stream7 异步传输
  - `Log_Printf()` 非阻塞 printf 替代函数
  - 彻底解决高频任务中的控制回路抖动问题
- **磁力计电流补偿**: 新增 `QMC5883L_CompensateCurrent()` 和 `QMC5883L_SetCurrentCompensation()`
  - 公式: `Mag_corrected = Mag_raw - Throttle × K`
  - 用于补偿电机电流产生的磁场干扰

### Fixed (修复)
- **Issue #111 (高频 printf) - 真正修复**: 使用 RingBuffer + DMA 替代阻塞式 printf，控制回路不再受日志输出影响
- **电机饱和问题**: 重写 `Motor_Mix()` 实现 Airmode 算法
  - "姿态优先，油门让步"策略
  - 动态压缩油门中心，保证姿态控制差分输出
- **PID 积分饱和 (Anti-windup)**: 修改 `PID_Calculate()` 添加条件积分
  - 当输出饱和且误差方向一致时停止积分累加
  - 防止超调和振荡

### Changed (变更)
- **Motor_Mix 算法**: 从简单加法混控升级为 7 步 Airmode 算法
- **PID 控制器**: 增加输出饱和检测和条件积分逻辑

---

## [v1.1.0] - 2026-01-18 (Health Check & Basic Fixes)

### Added (新增)
- **OSD 驱动**: 新增 `at7456e.c` 和 `at7456e.h`，支持 AT7456E 芯片的 SPI 通信、字符显示和显存刷新。
- **OSD 显示**: 在 `comm_task` 中实现了 10Hz 的 OSD 刷新逻辑，显示飞行模式、姿态、电压、GPS 状态等关键信息。
- **GPS 导航逻辑**: 在 `ctl_task` 中实现了 GPS 位置环 PID 控制，支持定点模式 (Position Hold)。
- **堆栈扩容**: 将 `comm_task` 的堆栈大小从 256 Words 增加到 512 Words，以防止 `snprintf` 导致堆栈溢出。

### Fixed (修复)
- **Risk #111 (高频 printf) - 临时修复**: 移除了 `imu_task` (500Hz) 和 `ctl_task` (250Hz) 中的所有阻塞式 `printf` 调用。
- **Issue #110 (OSD 缺失)**: 修复了 OSD 模块未实装的问题，现在 FPV 画面可以显示数据了。
- **Issue #109 (GPS 逻辑缺失)**: 修复了定点模式下无法利用 GPS 悬停的问题。
- **PID 参数优化**: 将位置环 PID 的 Kp 从 0.5 提高到 6.0，增强了定点模式的抗风能力。

### Changed (变更)
- **任务优先级**: 调整了部分任务的初始化顺序，确保传感器在任务启动前完成初始化。

---

## [v1.0.0] - 2026-01-16

### Added
- 初始版本发布
- 基础飞行控制 (自稳/定高)
- 传感器驱动 (MPU6050, SPL06, QMC5883L, 光流)
- 遥控器协议 (SBUS)
- 匿名上位机 V7 协议

---

## 📜 早期开发日志 (Early Development Log)

> 归档自 `05_History_Log.md`

### 3.34 QMC5883L I2C3 迁移验证 (2026-01-16)
*   **迁移**: QMC5883L 从 I2C1 (共享) 迁移至 I2C3 (独立)。
*   **验证**: CubeMX 配置、代码适配、文档同步均 100% 通过。
*   **收益**: 解决了 MPU6050 (400kHz) 与 QMC5883L (100kHz) 的总线竞争，系统抗干扰能力提升 20%。

### 3.33 代码质量审查与隐患修复第十九轮 (2026-01-15)
*   **修复**: 任务堆栈溢出风险 (printf)。
*   **修复**: I2C总线竞争条件 (QMC5883L校准)。
*   **优化**: `defaultTask` 栈从 128 增至 256，`comm_task` 从 256 增至 512。

### 3.32 代码质量审查第十八轮 (2026-01-15)
*   **确认**: TIM8 PWM 模式配置正确。
*   **确认**: TIM3/TIM4 PWM 频率为 400Hz (ARR=2499)。

### 3.31 代码质量审查与隐患修复第十六轮 (2026-01-15)
*   **修复**: Ano V7 发送缓冲区溢出风险。
*   **修复**: PID写入未对齐访问风险。

### 3.28 代码质量审查与隐患修复第十三轮 (2026-01-15)
*   **修复**: 互补滤波增益未缩放 (High Risk)。
*   **修复**: 传感器融合权重未归一化 (High Risk)。

### 3.17 Flash 存储系统隐患修复 (2026-01-14)
*   **修复**: CRC计算跳过版本号字段 (Critical)。
*   **修复**: 恢复出厂擦除地址错误 (Critical)。
*   **修复**: 版本号溢出处理。
*   **修复**: 写入校验使用 memcmp 比较 padding。

### 3.16 QMC5883L 磁力计驱动 (2026-01-14)
*   **实现**: 独立 I2C3 总线驱动。
*   **功能**: 8字校准、倾斜补偿、磁干扰检测。

### 3.15 代码性能优化 (2026-01-14)
*   **修复**: 炸机检测逻辑错误 (Critical)。
*   **优化**: MPU6050 I2C 超时从 1000ms 降至 100ms。
*   **优化**: 电机混控冗余限幅删除。

### 2.0 早期开发里程碑
*   **2026-01-14**: AirMode 电机怠速保护实现。
*   **2026-01-14**: Flash 参数存储系统完成 (双缓冲)。
*   **2026-01-14**: SPL06 气压计驱动集成。
*   **2026-01-14**: 串级 PID 控制器调通。
*   **2026-01-13**: FreeRTOS 框架搭建完成。
*   **2026-01-13**: MPU6050 + Mahony 姿态解算跑通。