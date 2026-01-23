# 🛠️ 硬件架构与PCB设计 (Hardware Design)

**版本**: v2.4.0 (STM32F405RGT6)
**最后更新**: 2026-01-23
**状态**: ✅ 已验证

---

## 0. 硬件变更记录 (Hardware Revision Log)

| 版本 | 日期 | 变更内容 |
|:---|:---|:---|
| v1.0.0 | 2026-01-15 | 初始设计 (STM32F407VET6)，TIM1_CH1 用于 WS2812 |
| v1.1.0 | 2026-01-17 | TIM1 改为 HAL 时基，WS2812 迁移至 TIM4_CH4 (PB9) |
| v1.2.0 | 2026-01-18 | 新增电池电流检测 (PC5/ADC1_IN15) |
| v1.3.0 | 2026-01-18 | HAL 时基改为 TIM4，WS2812 恢复使用 TIM1_CH1 (PA8) |
| v2.0.0 | 2026-01-18 | 迁移至 STM32F405RGT6 (LQFP-64)，完整 CubeMX 配置验证 |
| v2.1.0 | 2026-01-22 | 移除软开关，MPU_INT 迁移至 PB5 |
| v2.2.0 | 2026-01-23 | 串口重新分配：USART3 用于 ELRS (CRSF)，UART4 用于 SmartAudio，移除 I2C2，QMC5883L 迁移至 I2C1 |
| v2.3.0 | 2026-01-23 | 新增 7 颗状态指示 LED (PC3/PC4/PB0/PB1/PB13/PB14/PB15)，专业级飞行仪表盘 |
| **v2.4.0** | **2026-01-23** | **新增 USB 插入检测 (PB9)，解决 USB 反灌误判问题** |

---

## 0.1 MCU 规格对比

| 参数 | STM32F407VET6 (旧) | STM32F405RGT6 (新) |
|:---|:---|:---|
| 封装 | LQFP-100 | **LQFP-64** |
| Flash | 512KB | **1MB** |
| RAM | 192KB | 192KB (128KB + 64KB CCM) |
| 主频 | 168MHz | 168MHz |
| GPIO | 82 | **51** |
| 价格 | ~¥35 | **~¥25** |

---

## 1. MCU 引脚分配表 (Pinout Map) - STM32F405RGT6

| Pin | Label | Function | Net Name | Note |
|:---|:---|:---|:---|:---|
| **Power & System** |||||
| PC0 | V_BAT | ADC1_IN10 | VBAT_SENSE | 电池电压检测 (分压 1:11) |
| PC5 | I_BAT | ADC1_IN15 | CURR_SENSE | 电流检测 (来自 PDB/电调 CURR 脚, 0-3.3V) |
| PC2 | BUZZER | GPIO_Output (Low) | BUZZER | 蜂鸣器 (Active High) |
| PC13 | GPIO_OUT | GPIO_Output | - | 备用 GPIO |
| **LED 指示灯组 (7 颗)** |||||
| PC3 | LED_SYS | GPIO_Output (High) | LED_SYS | 系统心跳 (蓝色) - 核心系统组 |
| PC4 | LED_ERR | GPIO_Output (High) | LED_ERR | 警告/错误 (红色) - 核心系统组 |
| PB0 | LED_GPS | GPIO_Output (High) | LED_GPS | GPS 锁定状态 (绿色) - 导航链路组 |
| PB1 | LED_RX | GPIO_Output (High) | LED_RX | 遥控接收状态 (黄色) - 导航链路组 |
| PB13 | LED_MODE | GPIO_Output (High) | LED_MODE | 飞行模式 (白色) - 飞行任务组 |
| PB14 | LED_LOG | GPIO_Output (High) | LED_LOG | 黑匣子/日志 (橙色) - 飞行任务组 |
| PB15 | LED_VTX | GPIO_Output (High) | LED_VTX | 图传状态 (青色) - 飞行任务组 |
| **Sensors (I2C)** |||||
| PB6 | I2C1_SCL | I2C1 | 400kHz Fast | MPU6050 (IMU) + SPL06 (气压计) + QMC5883L (磁力计) |
| PB7 | I2C1_SDA | I2C1 | DMA RX/TX | 共享高速总线 (所有 I2C 传感器) |
| PB5 | MPU_INT | GPIO_EXTI5 | MPU_INT | MPU6050 数据就绪中断 |
| **Communication (UART)** |||||
| PA9 | USART1_TX | USART1 | 921600 Baud | K210 上位机 / 日志输出 (DMA TX) - **J3 接口** |
| PA10 | USART1_RX | USART1 | DMA RX (Circular) | K210 指令上行 |
| PA2 | USART2_TX | USART2 | 115200 Baud | GPS 模块配置 - **J2 接口** |
| PA3 | USART2_RX | USART2 | DMA RX (Circular) | GPS NMEA 数据接收 |
| PB10 | USART3_TX | USART3 | 420000 Baud | **ELRS 接收机 (CRSF 协议) - 焊盘接口** |
| PB11 | USART3_RX | USART3 | DMA1 Stream1 (Circular) | CRSF 信号接收 + 遥测回传 |
| PC10 | UART4_TX | UART4 | Half-Duplex | **图传 SmartAudio 调参 - J6 接口** |
| PC11 | UART4_RX | UART4 | 115200 Baud | SmartAudio 双向通信 |
| PC12 | UART5_TX | UART5 | 115200 Baud | 光流模块 (LC302) - **J5 接口** (DMA TX) |
| PD2 | UART5_RX | UART5 | Interrupt | 光流数据接收 |
| **USB** |||||
| PA11 | USB_DM | USB_OTG_FS | CDC (VCP) | 虚拟串口调参 (Rx/Tx 2048B) |
| PA12 | USB_DP | USB_OTG_FS | Device Mode | 固件升级 (DFU) |
| PB9 | USB_DETECT | GPIO_Input | USB_DET | USB 插入检测 (Active High, Pull-down) |
| **Actuators (Motors - TIM8)** |||||
| PC6 | MOTOR_1 | TIM8_CH1 | PWM 400Hz | 右前 (CCW) |
| PC7 | MOTOR_2 | TIM8_CH2 | PWM 400Hz | 左后 (CCW) |
| PC8 | MOTOR_3 | TIM8_CH3 | PWM 400Hz | 左前 (CW) |
| PC9 | MOTOR_4 | TIM8_CH4 | PWM 400Hz | 右后 (CW) |
| **RGB 灯光 (TIM1)** |||||
| PA8 | LED_RGB | TIM1_CH1 | PWM + DMA2_Stream1 | WS2812 驱动 (PSC=0, ARR=209) |
| **SPI 总线 (高速)** |||||
| PA5 | SPI1_SCK | SPI1 | 10.5 MBit/s | W25Q128 (Flash) + AT7456E (OSD) |
| PA6 | SPI1_MISO | SPI1 | DMA RX | 共享总线 |
| PA7 | SPI1_MOSI | SPI1 | DMA TX | 共享总线 |
| PA4 | FLASH_CS | GPIO_Output (High) | FLASH_CS | Flash 片选 (默认高) |
| PB12 | OSD_CS | GPIO_Output (High) | OSD_CS | OSD 片选 (默认高) |
| **ADC (多通道扫描)** |||||
| PA0 | ADC1_IN0 | ADC1 | - | 备用 ADC |
| PA2 | ADC1_IN2 | ADC1 | - | (与 USART2_TX 复用) |
| PA3 | ADC1_IN3 | ADC1 | - | (与 USART2_RX 复用) |
| PA4 | ADC1_IN4 | ADC1 | - | (与 FLASH_CS 复用) |
| ~~PB0~~ | ~~ADC1_IN8~~ | ~~ADC1~~ | - | ~~备用 ADC~~ (已改为 LED_GPS) |
| PC0 | ADC1_IN10 | ADC1 | VBAT_SENSE | 电池电压 |
| PC5 | ADC1_IN15 | ADC1 | CURR_SENSE | 电池电流 |
| **Debug** |||||
| PA13 | SWDIO | SYS | Serial Wire | 调试接口 |
| PA14 | SWCLK | SYS | Serial Wire | 调试接口 |
| **Crystal** |||||
| PH0 | RCC_OSC_IN | RCC | 8MHz HSE | 外部晶振输入 |
| PH1 | RCC_OSC_OUT | RCC | 8MHz HSE | 外部晶振输出 |

---

## 2. 时钟配置 (Clock Configuration) - STM32F405RGT6

### 2.1 时钟树配置

```
HSE (8MHz) ──┬── /M=4 ──> PLL Input (2MHz)
             │
             └── PLL ──┬── ×N=168 ──> VCO (336MHz)
                       │
                       ├── /P=2 ──> SYSCLK (168MHz)
                       │
                       └── /Q=7 ──> USB_OTG_FS (48MHz)

SYSCLK (168MHz) ──┬── AHB Prescaler /1 ──> HCLK (168MHz)
                  │
                  ├── APB1 Prescaler /4 ──> PCLK1 (42MHz) ──> APB1 Timer ×2 = 84MHz
                  │
                  └── APB2 Prescaler /2 ──> PCLK2 (84MHz) ──> APB2 Timer ×2 = 168MHz

PLLI2S ──┬── ×N=192 ──> VCO (384MHz)
         │
         └── /R=2 ──> I2S_CLK (192MHz)
```

### 2.2 时钟参数汇总

| 时钟源 | 频率 | 用途 |
|:---|:---|:---|
| HSE | 8 MHz | 外部晶振 |
| SYSCLK | 168 MHz | 系统主时钟 |
| HCLK | 168 MHz | AHB 总线 / Cortex 内核 |
| PCLK1 | 42 MHz | APB1 外设 (I2C, UART4/5, TIM2-7) |
| APB1 Timer | 84 MHz | TIM2-7, TIM12-14 |
| PCLK2 | 84 MHz | APB2 外设 (SPI1, USART1/2, TIM1/8) |
| APB2 Timer | 168 MHz | TIM1, TIM8, TIM9-11 |
| USB_OTG_FS | 48 MHz | USB 全速模式 |
| I2S_CLK | 192 MHz | 音频时钟 (未使用) |

---

## 3. DMA 架构配置 (DMA Architecture) - CubeMX 验证

**背景**: STM32F4 的 DMA 通道存在硬件绑定，需要合理分配避免冲突。
**状态**: ✅ 已通过 CubeMX 验证，无冲突。

### 3.1 DMA1 控制器配置 (APB1 外设)

| Stream | Channel | Request | Direction | Priority | Mode | 用途说明 |
|:---|:---|:---|:---|:---|:---|:---|
| **Stream 0** | Ch 1 | **I2C1_RX** | P→M | **Very High** | Normal | **MPU6050 批量读取** (核心传感器) |
| **Stream 1** | Ch 4 | **USART3_RX** | P→M | **Very High** | Circular | **CRSF/ELRS 信号接收** (420k baud) |
| **Stream 3** | Ch 4 | **USART3_TX** | M→P | Medium | Normal | CRSF 遥测回传 |
| **Stream 4** | Ch 4 | **UART4_TX** | M→P | Low | Normal | SmartAudio VTX 控制 |
| **Stream 5** | Ch 4 | **USART2_RX** | P→M | Medium | Circular | GPS NMEA 数据流 |
| **Stream 6** | Ch 1 | **I2C1_TX** | M→P | High | Normal | I2C1 寄存器地址写入 |
| **Stream 7** | Ch 4 | **UART5_TX** | M→P | Low | Normal | 光流模块发送 |

### 3.2 DMA2 控制器配置 (APB2 外设)

| Stream | Channel | Request | Direction | Priority | Mode | 用途说明 |
|:---|:---|:---|:---|:---|:---|:---|
| **Stream 0** | Ch 3 | **SPI1_RX** | P→M | Low | Normal | Flash/OSD 数据读取 |
| **Stream 1** | Ch 6 | **TIM1_CH1** | M→P | Low | Normal | **WS2812 RGB 灯驱动** |
| **Stream 2** | Ch 4 | **USART1_RX** | P→M | Medium | Circular | 树莓派指令接收 |
| **Stream 3** | Ch 3 | **SPI1_TX** | M→P | Low | Normal | OSD 显存刷新 / Flash 写入 |
| **Stream 4** | Ch 0 | **ADC1** | P→M | Low | Circular | **电池电压+电流双通道扫描** |
| **Stream 7** | Ch 4 | **USART1_TX** | M→P | Low | Normal | **RingBuffer 异步日志发送** |

---

## 4. FreeRTOS 任务配置 (RTOS Tasks)

| Task Name | Priority | Stack Size | Entry Function | 用途说明 |
|:---|:---|:---|:---|:---|
| **defaultTask** | osPriorityNormal | 256 Words | StartDefaultTask | LED 心跳 + 看门狗 (10Hz) |
| **imu_task** | osPriorityRealtime | 512 Words | StartTask02 | IMU DMA 读取 + Mahony 解算 (500Hz) |
| **ctl_task** | osPriorityHigh | 1024 Words | StartTask03 | SBUS 处理 + PID 控制 + 电机混控 (250Hz) |
| **log_task** | osPriorityBelowNormal | 1024 Words | StartLogTask | RingBuffer DMA 日志发送 (100Hz) |
| **comm_task** | osPriorityNormal | 512 Words | StartCommTask | OSD 刷新 + 上位机通信 (10Hz) |
| **gps_task** | osPriorityBelowNormal | 512 Words | StartTask06 | GPS 解析 + 导航计算 (10Hz) |

**FreeRTOS 配置参数**:
- Heap Size: 30720 Bytes (30KB)
- Tick Rate: 1000 Hz
- Minimal Stack Size: 128 Words
- Timer Task Stack Depth: 256 Words

---

## 5. NVIC 中断优先级配置 (Interrupt Priority)

| 中断源 | Preemption Priority | Sub Priority | FreeRTOS | 说明 |
|:---|:---|:---|:---|:---|
| Memory Management Fault | 0 | 0 | ❌ | 系统异常 |
| Bus Fault | 0 | 0 | ❌ | 系统异常 |
| Usage Fault | 0 | 0 | ❌ | 系统异常 |
| SVCall | 0 | 0 | ❌ | 系统调用 |
| PendSV | 15 | 0 | ✅ | RTOS 上下文切换 |
| SysTick | 15 | 0 | ✅ | RTOS 时基 |
| DMA1_Stream0 (I2C1_RX) | 5 | 0 | ✅ | IMU 数据接收 |
| DMA1_Stream1 (USART3_RX) | 5 | 0 | ✅ | CRSF/ELRS 接收 |
| DMA1_Stream5 (USART2_RX) | 5 | 0 | ✅ | GPS 接收 |
| DMA1_Stream6 (I2C1_TX) | 5 | 0 | ✅ | I2C 发送 |
| DMA2_Stream7 (USART1_TX) | 5 | 0 | ✅ | 日志发送 |
| I2C1 Event/Error | 5 | 0 | ✅ | I2C1 事件 |
| EXTI9_5 | 5 | 0 | ✅ | MPU6050 数据就绪 (PB5) |
| USART1 | 6 | 0 | ✅ | K210 上位机串口 |
| USART2 | 6 | 0 | ✅ | GPS 串口 |
| USART3 | 6 | 0 | ✅ | CRSF/ELRS 接收机 |
| UART4 | 6 | 0 | ✅ | SmartAudio VTX |
| UART5 | 6 | 0 | ✅ | 光流模块 |
| TIM4 (HAL Timebase) | 15 | 0 | ❌ | HAL 时基 (不使用 FreeRTOS) |
| USB_OTG_FS | 5 | 0 | ✅ | USB 中断 |
| ADC1/2/3 | 5 | 0 | ✅ | ADC 中断 |
| TIM1 CC | 5 | 0 | ✅ | WS2812 PWM |

**注意**: FreeRTOS 可管理的中断优先级范围为 5-15 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5)

---

## 6. USB 配置 (USB_OTG_FS)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Device Only | 仅设备模式 |
| Speed | Full Speed (12 Mbps) | USB 2.0 全速 |
| Class | CDC (Virtual Port Com) | 虚拟串口 |
| VID | 0x0483 | STMicroelectronics |
| PID | 0x5740 | Virtual COM Port |
| Rx Buffer Size | 2048 Bytes | 接收缓冲区 |
| Tx Buffer Size | 2048 Bytes | 发送缓冲区 |
| Max Interfaces | 1 | 单接口 |
| Self Powered | Enabled | 自供电 |

---

## 7. ADC 配置 (ADC1)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Independent | 独立模式 |
| Clock Prescaler | PCLK2 / 4 | 21 MHz |
| Resolution | 12 bits | 4096 级 |
| Data Alignment | Right | 右对齐 |
| Scan Conversion | Enabled | 扫描模式 |
| Continuous Conversion | Enabled | 连续转换 |
| DMA Continuous Requests | Enabled | DMA 连续请求 |
| Number of Conversion | 2 | 双通道 (电压+电流) |

**通道配置**:
| Rank | Channel | Sampling Time | 用途 |
|:---|:---|:---|:---|
| 1 | IN10 (PC0) | 480 Cycles | 电池电压 |
| 2 | IN15 (PC5) | 480 Cycles | 电池电流 |

---

## 7.1 定时器配置 (Timers)

### TIM1 - WS2812 RGB 灯驱动

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Clock Source | Internal | APB2 Timer = 168MHz |
| Channel1 | PWM Generation CH1 | PA8 输出 |
| Prescaler (PSC) | 0 | 不分频 |
| Counter Period (ARR) | 209 | 168MHz / 210 = 800kHz (WS2812 协议) |
| PWM Mode | Mode 1 | 向上计数 |
| Pulse (CCR1) | 0 | 初始占空比 |
| Output Compare Preload | Enable | 预装载使能 |
| DMA | TIM1_CH1 → DMA2_Stream1 | PWM 数据传输 |

**WS2812 时序计算**:
- 周期: 1/800kHz = 1.25us ✓
- T0H (0码高电平): 67/210 × 1.25us ≈ 0.4us ✓
- T1H (1码高电平): 134/210 × 1.25us ≈ 0.8us ✓

### TIM4 - HAL 时基 (Timebase Source)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| 用途 | HAL_IncTick() | 系统时基 |
| 优先级 | 15 (最低) | 不干扰 FreeRTOS |

### TIM8 - 电机 PWM (400Hz)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Clock Source | Internal | APB2 Timer = 168MHz |
| Channel1-4 | PWM Generation | PC6-PC9 输出 |
| Prescaler (PSC) | 83 | 168MHz / 84 = 2MHz |
| Counter Period (ARR) | 4999 | 2MHz / 5000 = 400Hz |
| PWM Mode | Mode 1 | 向上计数 |
| Pulse Range | 1000-2000 | 标准 PWM 范围 |

**电机 PWM 计算**:
- 频率: 168MHz / (83+1) / (4999+1) = 400Hz ✓
- 分辨率: 5000 级 (0.02% 精度)

---

## 7.2 I2C 配置

### I2C1 - 高速传感器总线 (MPU6050 + SPL06)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | I2C | 主机模式 |
| I2C Speed Mode | Fast Mode | 400kHz |
| I2C Clock Speed | 400000 Hz | 高速通信 |
| Fast Mode Duty Cycle | Tlow/Thigh = 2 | 标准占空比 |
| Clock No Stretch Mode | Disabled | 允许时钟拉伸 |
| Primary Address Length | 7-bit | 标准地址 |
| DMA | I2C1_RX → DMA1_Stream0, I2C1_TX → DMA1_Stream6 | 非阻塞传输 |

**连接设备**:
| 设备 | 地址 (7-bit) | 用途 |
|:---|:---|:---|
| MPU6050 | 0x68 | 6轴 IMU |
| SPL06-001 | 0x76 | 气压计 |

### [已移除] I2C2 - 磁力计专用总线

*   **变更**: v2.2.0 版本移除了 I2C2，QMC5883L 磁力计迁移至 I2C1 共享总线。
*   **原因**: 释放 PB10/PB11 引脚给 USART3 (ELRS 接收机)。
*   **注意**: 磁力计现与 IMU/气压计共享 I2C1 总线，需注意软件层面的访问调度。

---

## 7.3 SPI 配置

### SPI1 - 高速外设总线 (Flash + OSD)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Full-Duplex Master | 全双工主机 |
| Hardware NSS Signal | Disable | 软件片选 |
| Frame Format | Motorola | 标准 SPI |
| Data Size | 8 Bits | 字节传输 |
| First Bit | MSB First | 高位先发 |
| Prescaler | 8 | PCLK2/8 = 10.5 Mbps |
| Clock Polarity (CPOL) | Low | 空闲低电平 |
| Clock Phase (CPHA) | 1 Edge | 第一边沿采样 |
| CRC Calculation | Disabled | 无 CRC |
| NSS Signal Type | Software | 软件控制 |
| DMA | SPI1_RX → DMA2_Stream0, SPI1_TX → DMA2_Stream3 | 高速传输 |

**片选信号**:
| 设备 | CS 引脚 | 默认状态 |
|:---|:---|:---|
| W25Q128 Flash | PA4 (FLASH_CS) | High (未选中) |
| AT7456E OSD | PB12 (OSD_CS) | High (未选中) |

---

## 7.4 UART 配置

### USART1 - 树莓派/日志串口

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Asynchronous | 异步模式 |
| Baud Rate | 921600 Bits/s | 高速通信 |
| Word Length | 8 Bits | 标准字长 |
| Parity | None | 无校验 |
| Stop Bits | 1 | 1 停止位 |
| Hardware Flow Control | Disable | 无流控 |
| Over Sampling | 16 Samples | 标准采样 |
| DMA | USART1_TX → DMA2_Stream7, USART1_RX → DMA2_Stream2 | 异步传输 |

### USART2 - GPS 模块

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Asynchronous | 异步模式 |
| Baud Rate | 115200 Bits/s | GPS 标准波特率 |
| Word Length | 8 Bits | 标准字长 |
| Parity | None | 无校验 |
| Stop Bits | 1 | 1 停止位 |
| Hardware Flow Control | Disable | 无流控 |
| Over Sampling | 16 Samples | 标准采样 |
| DMA | USART2_RX → DMA1_Stream5 (Circular) | NMEA 连续接收 |

### USART3 - ELRS 接收机 (CRSF 协议)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Asynchronous | 异步模式 |
| Baud Rate | **420000 Bits/s** | CRSF 协议波特率 |
| Word Length | 8 Bits | 标准字长 |
| Parity | None | 无校验 |
| Stop Bits | 1 | 1 停止位 |
| Data Direction | Receive and Transmit | **双向 (支持遥测回传)** |
| Over Sampling | 16 Samples | 标准采样 |
| DMA | USART3_RX → DMA1_Stream1 (Circular, Very High) | CRSF 连续接收 |
| DMA | USART3_TX → DMA1_Stream3 (Normal, Medium) | 遥测发送 |
| GPIO | PB10 (TX) / PB11 (RX) | Pull-up 使能 |

**CRSF 协议优势** (相比 SBUS):
*   **更高波特率**: 420kbps vs 100kbps，更低延迟
*   **双向通信**: 支持遥测回传 (电池电压、RSSI、GPS 等)
*   **无需反相器**: 标准 UART 电平，简化硬件设计
*   **CRC 校验**: 更可靠的数据传输

### UART4 - SmartAudio VTX 控制

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Asynchronous | 异步模式 |
| Baud Rate | 4800 Bits/s | SmartAudio 协议波特率 |
| Word Length | 8 Bits | 标准字长 |
| Parity | None | 无校验 |
| Stop Bits | 2 | 2 停止位 |
| Data Direction | Receive and Transmit | 双向 (半双工) |
| Over Sampling | 16 Samples | 标准采样 |
| DMA | UART4_TX → DMA1_Stream4 (Normal, Low) | SmartAudio 命令发送 |

**注意**: SmartAudio 使用半双工通信，TX/RX 需要外部二极管合并为单线。

### UART5 - 光流模块

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Asynchronous | 异步模式 |
| Baud Rate | 115200 Bits/s | 标准波特率 |
| Word Length | 8 Bits | 标准字长 |
| Parity | None | 无校验 |
| Stop Bits | 1 | 1 停止位 |
| DMA | UART5_TX → DMA1_Stream7 | 异步发送 |

**接口**: J5 连接器

---

## 7.45 LED 指示灯系统 (7 颗专业级飞行仪表盘)

> **设计理念**: 将 7 颗 LED 按功能分组，形成"飞行仪表盘"，一眼即可判断飞控状态。

### 7.45.1 LED 功能分组

#### 第一组：核心系统 (Core System) - PC3, PC4

| 引脚 | 标签 | 颜色 | 功能定义 | 逻辑 (灌电流, 低电平亮) |
|:---|:---|:---|:---|:---|
| PC3 | LED_SYS | 蓝色 | 系统心跳/状态 | 闪烁(1Hz): 待机正常<br>常亮: 已解锁(Armed)<br>灭: 初始化中 |
| PC4 | LED_ERR | 红色 | 警告/错误 | 熄灭: 正常<br>慢闪: 低电压<br>快闪: 传感器故障<br>常亮: HardFault |

#### 第二组：导航与链路 (Nav & Link) - PB0, PB1

| 引脚 | 标签 | 颜色 | 功能定义 | 逻辑 (灌电流, 低电平亮) |
|:---|:---|:---|:---|:---|
| PB0 | LED_GPS | 绿色 | GPS 锁定状态 | 熄灭: 无GPS<br>闪烁: 搜星中<br>常亮: 3D Fix OK |
| PB1 | LED_RX | 黄色 | 遥控接收状态 | 常亮: 信号正常<br>闪烁: RSSI低<br>灭: Failsafe |

#### 第三组：飞行任务 (Flight Task) - PB13, PB14, PB15

| 引脚 | 标签 | 颜色 | 功能定义 | 逻辑 (灌电流, 低电平亮) |
|:---|:---|:---|:---|:---|
| PB13 | LED_MODE | 白色 | 飞行模式 | 灭: 自稳/半自稳<br>亮: 手动(Acro)<br>闪: 返航(RTH) |
| PB14 | LED_LOG | 橙色 | 黑匣子/日志 | 灭: 停止记录<br>闪烁: 正在写入Flash |
| PB15 | LED_VTX | 青色 | 图传状态 | 灭: 正常发射<br>亮: PIT Mode(低功率) |

### 7.45.2 硬件电路设计 (灌电流接法)

```
    3.3V ──> 限流电阻 ──> LED 正极 (+) ──> LED 负极 (-) ──> MCU 引脚
                │
                └── 低电平亮 (Sink Mode)
```

**电阻选型 (0603 封装)**:
*   **红色/黄色/橙色 LED**: 压降约 2V，使用 **1kΩ** 电阻
*   **蓝色/绿色/白色/青色 LED**: 压降约 3V，使用 **470Ω - 680Ω** 电阻

### 7.45.3 CubeMX GPIO 配置

| 引脚 | GPIO Output Level | GPIO Mode | Pull-up/Pull-down | Speed | User Label |
|:---|:---|:---|:---|:---|:---|
| PC3 | **High** | Push Pull | No pull | Low | LED_SYS |
| PC4 | **High** | Push Pull | No pull | Low | LED_ERR |
| PB0 | **High** | Push Pull | No pull | Low | LED_GPS |
| PB1 | **High** | Push Pull | No pull | Low | LED_RX |
| PB13 | **High** | Push Pull | No pull | Low | LED_MODE |
| PB14 | **High** | Push Pull | No pull | Low | LED_LOG |
| PB15 | **High** | Push Pull | No pull | Low | LED_VTX |

**配置说明**:
*   **Output Level = High**: 初始化为高电平 = 灯默认灭 (灌电流接法)
*   **Speed = Low**: LED 不需要高速翻转，减少对 GPS/接收机的高频干扰

---

## 7.5 USB 配置 (USB_OTG_FS)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| Mode | Device_Only | 仅设备模式 |
| Activate_SOF | Disabled | 不激活 SOF |
| Activate_VBUS | Disabled | 不检测 VBUS |
| Speed | Full Speed (12 Mbps) | USB 2.0 全速 |
| GPIO Mode | Alternate Function | PA11/PA12 |
| GPIO Pull-up | No pull-up | 无上拉 |
| Maximum Output Speed | Very High | 高速 GPIO |

---

## 8. 电源管理系统 (Power Management)

> **设计目标**: 稳定、可靠、宽电压输入，参考立创开源 F4 飞控 V2 方案优化

### 8.0 电源架构总览

```
                                    ┌─────────────────────────────────────────┐
                                    │           FLYAMASTER 电源架构            │
                                    └─────────────────────────────────────────┘
                                    
    ┌─────────┐                     ┌─────────────┐     ┌─────────────┐
    │ 电池    │────────────────────>│  DCDC 5V    │────>│  LDO 3.3V   │
    │ 2S-9S   │                     │  SY8303     │     │  RT9193     │
    │4.5-40V  │                     │  3A         │     │  500mA      │
    └─────────┘                     └─────────────┘     └─────────────┘
         │                               │                    │
         │                               │                    ├──> MCU (STM32F405)
         │                               │                    ├──> 传感器 (IMU/Baro/Mag)
         │                               │                    ├──> Flash (W25Q128)
         │                               │                    └──> OSD (AT7456E)
         │                               │
         │                               ├──> USB 5V (带反灌保护)
         │                               ├──> 蜂鸣器 (5V)
         │                               └──> 外设接口 (GPS/光流)
         │
         ├──> 电机 (直接电池供电)
         ├──> 图传 VTX (VBAT)
         └──> ADC 电压检测 (分压 1:11)
```

### 8.1 DCDC 降压芯片选型对比

| 参数 | **SY8303** (推荐) | TPS5450DDR | MP1584 |
|:---|:---|:---|:---|
| **输入电压** | **4.5-40V** (2S-9S) | 5.5-36V (2S-8S) | 4.5-28V (2S-6S) |
| **输出电流** | **3A** | 5A | 3A |
| **效率** | 95% | 90% | 92% |
| **封装** | **SOT-23-6** (小巧) | HSOP-8 (较大) | SOT-23-8 |
| **价格** | ~¥0.8 | ~¥3.5 | ~¥1.2 |
| **外围元件** | 少 | 多 | 中等 |
| **评价** | ✅ 性价比高，稳定 | 电流裕量大 | 输入范围窄 |

**选型结论**: 采用 **SY8303** 作为主 DCDC，支持 2S-9S 电池，3A 输出足够驱动所有外设。

### 8.2 [已移除] 软开关电路

*   **变更**: v2.0.0 版本移除了软开关电路，改为直接电池供电。
*   **原因**: 简化电路，提高可靠性，减少 PCB 空间占用。

### 8.3 USB 反灌保护电路 (USB Backfeed Protection)

> **问题**: 当电池供电时，5V 会通过 USB 线路反灌到电脑 USB 口，导致 MCU 误判 USB 已连接。

**解决方案**: 在 USB_Detect 线路上添加肖特基二极管隔离。

```
                    ┌─────────────────────────────────────┐
                    │         USB 反灌保护电路             │
                    └─────────────────────────────────────┘

    USB Type-C                                          MCU
    ┌─────────┐                                    ┌─────────┐
    │  VBUS   │──┬──[D3: SS14]──>──┬──> +5V_USB    │         │
    │         │  │                 │               │         │
    │  D+     │──┼─────────────────┼──────────────>│ PA12    │
    │  D-     │──┼─────────────────┼──────────────>│ PA11    │
    │  GND    │──┴─────────────────┴──────────────>│ GND     │
    └─────────┘                                    └─────────┘
                                   │
                                   ├──[R_detect: 10k]──> PB9 (USB_DETECT)
                                   │
                                   └──[R_pulldown: 100k]──> GND (可选)
                                   
    注意:
    1. 肖特基二极管 D3 放在 VBUS 之后，USB_DETECT 从 D3 阳极取信号
    2. 这样电池 5V 不会反灌到 USB_DETECT 线路
    3. R_pulldown 可选，如果使用软件内部下拉则可省略
```

**关键元件**:
*   **D3 (肖特基二极管)**: **SS14** (SMA, 1A, 40V) - 低压降 (~0.3V)
*   **R_detect (检测电阻)**: **10kΩ** (0603) - 限流保护
*   **R_pulldown (下拉电阻)**: **100kΩ** (0603) - 防止引脚浮空 (可选，软件可开启内部下拉)

**CubeMX 配置 (PB9)**:
| 参数 | 值 | 说明 |
|:---|:---|:---|
| GPIO Mode | Input | 输入模式 |
| Pull-up/Pull-down | **Pull-down** | 下拉 (USB 拔出时为低电平) |
| User Label | USB_DETECT | 用户标签 |

**软件使用**:
```c
// 检测 USB 是否插入
bool USB_IsConnected(void) {
    return HAL_GPIO_ReadPin(USB_DETECT_GPIO_Port, USB_DETECT_Pin) == GPIO_PIN_SET;
}
```

### 8.4 电源滤波与去耦 (Decoupling)

*   **MCU 电源**: 每个 VDD 引脚必须紧靠一个 **100nF (0.1uF)** 电容 (0603)。
*   **DCDC 输入/输出**:
    *   输入端: **22uF** (1206) + **100nF** (0603) - 陶瓷电容
    *   输出端: **22uF** (1206) + **100nF** (0603) - 陶瓷电容
    *   **注意**: 避免使用钽电容，MLCC 陶瓷电容更稳定且成本更低
*   **LDO 输入/输出**:
    *   输入端: **10uF** (0805) + **100nF** (0603)
    *   输出端: **22uF** (0805) + **100nF** (0603)
*   **电机电源**: 电调焊盘处并联大容量电解电容 (如 470uF/25V) 吸收反电动势。

### 8.5 总线上拉 (Pull-up Resistors)

*   **I2C 总线 (I2C1, I2C2)**: 必须添加上拉电阻。
    *   阻值: **4.7kΩ** 或 **2.2kΩ** (0603)。
    *   位置: 靠近 MCU 或传感器均可，每条总线一组即可。
*   **Boot0**: 下拉 **10kΩ** (0603) 确保正常启动。
*   **Reset**: 上拉 **10kΩ** (0603) + **100nF** 电容对地。

### 8.6 可选: 9V BEC 输出 (支持 DJI 数传)

> 如果需要支持 DJI 数字图传系统，可增加 9V BEC 输出。

**方案**: 使用 **SY8303** 或 **MP2315** 配置为 9V 输出。

| 参数 | 值 |
|:---|:---|
| 输入 | VBAT (4.5-40V) |
| 输出 | 9V / 2A |
| 芯片 | SY8303 (调整反馈电阻) |
| 用途 | DJI Air Unit / Vista |

**反馈电阻计算** (SY8303):
*   Vout = 0.6V × (1 + R1/R2)
*   9V = 0.6V × (1 + R1/R2) → R1/R2 = 14
*   推荐: R1 = 56kΩ, R2 = 4kΩ

---

## 9. PCB 布局与走线约束 (Layout & Routing)

### 9.1 布局 (Placement)
*   **传感器隔离**:
    *   **磁力计 (QMC5883L)**: 必须**远离**电机线、大电流回路和蜂鸣器。建议放置在板子边缘或伸出的“鸭嘴”区域。
    *   **气压计 (SPL06)**: 避免强光直射和气流直吹（需加海绵遮光罩）。
*   **晶振**: 尽量靠近 MCU，下方**禁止走线**，周围包地。
*   **电源**: DCDC 和 LDO 属于发热源，需适当散热铺铜。

### 9.2 走线 (Routing)
*   **电源线宽**:
    *   主电源 (VBAT): > **30mil** (或铺铜)
    *   3.3V / 5V: > **20mil**
    *   信号线: **6-8mil**
*   **差分线**: USB (D+/D-) 必须走差分线，阻抗控制 90Ω (若无阻抗板，保持平行、等长、包地)。
*   **过孔 (Via)**:
    *   信号过孔: 0.3mm/0.6mm (12/24mil)
    *   电源过孔: 多打几个或使用大过孔 0.5mm/1.0mm。
*   **晶振处理**:
    *   晶振下方**禁止走线**，所有层都要挖空 (Keepout)。
    *   晶振外壳建议接地 (如果是金属壳)。
    *   负载电容应尽可能靠近晶振引脚。

### 9.3 0603 封装建议
*   **电阻/电容**: 统一使用 **0603 (1608 Metric)** 封装，便于手工焊接且占用空间适中。
*   **大电容**: 10uF/22uF 可能需要 **0805** 或 **1206** 封装以保证耐压。

---

## 10. BOM 预选型 (Bill of Materials)

> **更新**: 参考立创开源 F4 飞控 V2 优化电源方案

### 10.1 核心器件

| 类型 | 规格 | 封装 | 用量 | 备注 |
|:---|:---|:---|:---|:---|
| **MCU** | **STM32F405RGT6** | LQFP-64 | 1 | 核心 (168MHz, 1MB Flash) |
| **IMU** | **MPU6050** | QFN-24 | 1 | 6轴 IMU (I2C1) |
| **气压计** | **SPL06-001** | LGA-8 | 1 | 气压高度 (I2C1) |
| **磁力计** | **QMC5883L** | LGA-16 | 1 | 电子罗盘 (I2C2) |
| **Flash** | **W25Q128JVSIQ** | SOP-8 | 1 | 黑匣子 (16MB, SPI1) |
| **OSD** | **AT7456E** | TSSOP-28 | 1 | 图传叠加 (SPI1) |

### 10.2 电源管理器件

| 类型 | 规格 | 封装 | 用量 | 备注 |
|:---|:---|:---|:---|:---|
| **DCDC** | **SY8303** | **SOT-23-6** | 1 | **5V/3A** (4.5-40V 输入, 2S-9S) |
| **LDO** | RT9193-33 | SOT-23-5 | 1 | 3.3V/500mA (数字电源) |
| **Diode** | **SS14** | SMA | 1 | USB 反灌保护 (肖特基) |
| **Diode** | 1N4148 | SOD-323 | 1 | 蜂鸣器续流 |

### 10.3 晶振与无源器件

| 类型 | 规格 | 封装 | 用量 | 备注 |
|:---|:---|:---|:---|:---|
| **Crystal** | **8MHz** | **SMD2016** | 1 | HSE 晶振 (小封装, 省空间) |
| **Crystal** | 27MHz | SMD2016 | 1 | OSD AT7456E 时钟 |
| **Inductor** | 4.7uH/3A | 4x4mm | 1 | DCDC 电感 (SY8303) |
| **Resistor** | 10k, 1k, 4.7k, 22R | 0603 | 若干 | 上拉/限流 |
| **Capacitor** | 100nF, 1uF | 0603 | 若干 | 滤波 |
| **Capacitor** | 22uF/25V | **1206** | 4 | DCDC 输入/输出 (MLCC) |
| **Capacitor** | 10uF | 0805 | 若干 | LDO 滤波 |

### 10.4 连接器与外设

| 类型 | 规格 | 封装 | 用量 | 备注 |
|:---|:---|:---|:---|:---|
| **Connector** | **SH1.0** (卧式贴片) | SMD | 若干 | 接口座子 (J2:GPS / J3:K210 / J5:光流 / J6:VTX) |
| **USB** | **Type-C** (16P) | SMD | 1 | 固件升级/调参 |
| **Buzzer** | 5V 有源蜂鸣器 | 9x5.5mm | 1 | 提示音 (PC2 驱动) |
| **LED** | 蓝色 0603 | 0603 | 1 | LED_SYS 系统心跳 (PC3) |
| **LED** | 红色 0603 | 0603 | 1 | LED_ERR 警告/错误 (PC4) |
| **LED** | 绿色 0603 | 0603 | 1 | LED_GPS GPS 锁定 (PB0) |
| **LED** | 黄色 0603 | 0603 | 1 | LED_RX 遥控接收 (PB1) |
| **LED** | 白色 0603 | 0603 | 1 | LED_MODE 飞行模式 (PB13) |
| **LED** | 橙色 0603 | 0603 | 1 | LED_LOG 黑匣子 (PB14) |
| **LED** | 青色 0603 | 0603 | 1 | LED_VTX 图传状态 (PB15) |
| **Resistor** | 1kΩ 0603 | 0603 | 4 | LED 限流 (红/黄/橙色) |
| **Resistor** | 470Ω-680Ω 0603 | 0603 | 3 | LED 限流 (蓝/绿/白/青色) |

### 10.5 可选器件 (9V BEC)

| 类型 | 规格 | 封装 | 用量 | 备注 |
|:---|:---|:---|:---|:---|
| **DCDC** | SY8303 | SOT-23-6 | 1 | 9V/2A (DJI 数传供电) |
| **Inductor** | 4.7uH/2A | 4x4mm | 1 | 9V DCDC 电感 |
| **Capacitor** | 22uF/25V | 1206 | 2 | 9V DCDC 滤波 |