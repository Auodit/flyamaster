# 🛠️ 硬件架构与PCB设计 (Hardware Design)

**版本**: v2.5.7 (STM32F405RGT6 - PCB Layout Final)
**最后更新**: 2026-01-25
**状态**: ✅ CubeMX 已生成

---

## 0. 硬件变更记录 (Hardware Revision Log)

| 版本 | 日期 | 变更内容 |
|:---|:---|:---|
| v2.0.0 | 2026-01-18 | 迁移至 STM32F405RGT6 (LQFP-64) |
| v2.2.0 | 2026-01-23 | 串口重新分配：USART3 用于 ELRS |
| v2.3.0 | 2026-01-23 | 新增 7 颗 LED |
| v2.4.0 | 2026-01-23 | 新增 USB 插入检测 (PB9) |
| v2.5.0 | 2026-01-24 | PCB 布局深度优化：按物理边沿重排引脚 |
| v2.5.2 | 2026-01-24 | 布局微调：顶部仅保留传感器，插座全部移至左/右/下侧 |
| v2.5.3 | 2026-01-24 | USART2/UART5 对调：UART5 用于 GPS，USART2 用于光流 |
| v2.5.4 | 2026-01-24 | 引脚调整：PC5 改为 USB_DETECT，电流检测移至 PC1 (ADC11) |
| v2.5.5 | 2026-01-24 | I2C1_SCL 重映射：PB6 → PB8 |
| v2.5.6 | 2026-01-24 | I2C1 完整重映射 (PB8/PB9)，USART1 重映射 (PB6/PB7) |
| **v2.5.7** | **2026-01-25** | **SPI 片选对调：PA4→OSD_CS，PC4→FLASH_CS** |

---

## 1. MCU 引脚分配表 (Pinout Map) - 按物理边沿排列

> **方向定义**: 以芯片丝印正向为准，Pin 1 为左上角。

### 1.1 顶部边沿 (Top Edge, Pins 49-64)
*布局目标: I2C 传感器 (PCB 顶部区域)*

| Pin | Label | Function | Net Name | Note |
|:---|:---|:---|:---|:---|
| PB9 | I2C1_SDA | I2C1 | I2C1_SDA | **传感器总线** (重映射) |
| PB8 | I2C1_SCL | I2C1 | I2C1_SCL | **传感器总线** (重映射) |
| BOOT0 | BOOT0 | System | BOOT0 | 启动模式选择 (背面开关) |
| PB7 | USART1_RX | USART1 | VTX_RX | **SmartAudio 图传** (重映射) |
| PB6 | USART1_TX | USART1 | VTX_TX | **SmartAudio 图传** (重映射) |
| PB5 | MPU_INT | GPIO_EXTI5 | MPU_INT | IMU 中断 |
| PB4 | GPIO_Output | GPIO_Output | - | 备用 |
| PB3 | GPIO_Output | GPIO_Output | - | 备用 |
| PD2 | UART5_RX | UART5 | **GPS_RX** | **GPS 模块** (引至左侧插座) |
| PC12 | UART5_TX | UART5 | **GPS_TX** | **GPS 模块** (引至左侧插座) |
| PC11 | UART4_RX | UART4 | LOG_RX | K210/调试日志 (引至左侧插座) |
| PC10 | UART4_TX | UART4 | LOG_TX | K210/调试日志 (引至左侧插座) |
| PA15 | - | - | - | 备用 |
| PA14 | SWCLK | SYS | SWCLK | SWD 调试接口 |

### 1.2 右侧边沿 (Right Edge, Pins 33-48)
*布局目标: USB, VTX, Motors, LEDs*

| Pin | Label | Function | Net Name | Note |
|:---|:---|:---|:---|:---|
| PA13 | SWDIO | SYS | SWDIO | SWD 调试接口 |
| PA12 | USB_DP | USB_OTG_FS | USB_DP | USB Type-C (背面) |
| PA11 | USB_DM | USB_OTG_FS | USB_DM | USB Type-C (背面) |
| PA10 | GPIO_Output | GPIO_Output | - | 备用 (原 USART1_RX) |
| PA9 | GPIO_Output | GPIO_Output | - | 备用 (原 USART1_TX) |
| PA8 | LED_RGB | TIM1_CH1 | LED_RGB | 4颗 WS2812 (位于PCB四角) |
| PC9 | MOTOR_4 | TIM8_CH4 | M4 | 电机 4 (CW) |
| PC8 | MOTOR_3 | TIM8_CH3 | M3 | 电机 3 (CCW) |
| PC7 | MOTOR_2 | TIM8_CH2 | M2 | 电机 2 (CCW) |
| PC6 | MOTOR_1 | TIM8_CH1 | M1 | 电机 1 (CW) |
| PB15 | LED_VTX | GPIO_Output | LED_VTX | 状态灯 4 (青色) - 图传 |
| PB14 | LED_LOG | GPIO_Output | LED_LOG | 状态灯 3 (橙色) - 日志 |
| PB13 | LED_MODE | GPIO_Output | LED_MODE | 状态灯 2 (白色) - 模式 |
| PB12 | LED_SYS | GPIO_Output | LED_SYS | 状态灯 1 (蓝色) - 系统 |

### 1.3 底部边沿 (Bottom Edge, Pins 17-32)
*布局目标: SPI (Flash/OSD), ELRS, 光流*

| Pin | Label | Function | Net Name | Note |
|:---|:---|:---|:---|:---|
| PB11 | USART3_RX | USART3 | ELRS_RX | **ELRS 接收机** (焊盘) |
| PB10 | USART3_TX | USART3 | ELRS_TX | **ELRS 接收机** (焊盘) |
| PB2 | GPIO_Output | GPIO_Output | - | 备用 (BOOT1) |
| PB1 | GPIO_Output | GPIO_Output | - | 备用 |
| PB0 | GPIO_Output | GPIO_Output | - | 备用 |
| PC5 | USB_DETECT | GPIO_Input | USB_DET | **USB 插入检测** |
| PC4 | FLASH_CS | GPIO_Output | FLASH_CS | **Flash 片选** (靠近 SPI) |
| PA7 | SPI1_MOSI | SPI1 | SPI1_MOSI | Flash + OSD |
| PA6 | SPI1_MISO | SPI1 | SPI1_MISO | Flash + OSD |
| PA5 | SPI1_SCK | SPI1 | SPI1_SCK | Flash + OSD |
| PA4 | OSD_CS | GPIO_Output | OSD_CS | **OSD 片选** |
| PA3 | USART2_RX | USART2 | **FLOW_RX** | **光流模块** (底部/左侧插座) |

### 1.4 左侧边沿 (Left Edge, Pins 1-16)
*布局目标: Power, Buzzer, 光流*

| Pin | Label | Function | Net Name | Note |
|:---|:---|:---|:---|:---|
| PA2 | USART2_TX | USART2 | **FLOW_TX** | **光流模块** (底部/左侧插座) |
| PA1 | - | - | - | 备用 |
| PA0 | Power_Key | GPIO_Input | POWER_KEY | 电源按键 (可选) |
| PC3 | - | - | - | 备用 |
| PC2 | BUZZER | GPIO_Output | BUZZER | 蜂鸣器 |
| PC1 | I_BAT | ADC1_IN11 | CURR_SENSE | **电流检测** |
| PC0 | V_BAT | ADC1_IN10 | VBAT_SENSE | 电压检测 |
| NRST | NRST | System | NRST | 复位 |
| PH1 | OSC_OUT | RCC | HSE_OUT | 8MHz 晶振 |
| PH0 | OSC_IN | RCC | HSE_IN | 8MHz 晶振 |

---

## 2. 串口分配汇总 (UART Assignment Summary)

| 串口 | 引脚 | 波特率 | 用途 | 接口类型 | 位置 |
|:---|:---|:---|:---|:---|:---|
| **USART1** | PB6/PB7 | 4800 | SmartAudio VTX | J6 插座 | 顶部 (重映射) |
| **USART2** | PA2/PA3 | 115200 | **光流模块** | J5 插座 | 底部/左侧 |
| **USART3** | PB10/PB11 | 420000 | ELRS (CRSF) | 焊盘 | 底部 |
| **UART4** | PC10/PC11 | 921600 | K210/日志 | J3 插座 | 顶部→左侧 |
| **UART5** | PC12/PD2 | 115200 | **GPS 模块** | J2 插座 | 顶部→左侧 |

---

## 3. 关键布局约束 (PCB Layout Constraints)

### 3.1 物理分区
*   **Top (上)**: **传感器专用区**。MPU6050, SPL06, QMC5883L 放置在 PCB 顶部中间，I2C 走线极短。**顶部边缘不放置任何插座**。
*   **Right (右)**: **接口区**。USB Type-C (背面), J6 (VTX), 电机焊盘。
*   **Bottom (下)**: **存储与 OSD 区**。Flash, AT7456E, ELRS 焊盘。SPI 走线紧凑。
*   **Left (左)**: **电源与外设区**。LDO, DCDC, ADC 检测。GPS (J2), 光流 (J5), 日志 (J3) 插座。

### 3.2 特殊走线要求
*   **USB (PA11/PA12)**: 差分走线，90Ω 阻抗控制，尽量短。
*   **OSD 视频线**: 模拟视频信号 (VIN/VOUT) 必须包地，远离 DCDC 电感和电机线。
*   **晶振 (PH0/PH1)**: 下方挖空，禁止走线，包地保护。
*   **电流检测 (PC5)**: 采样电阻出来的信号线需开尔文连接 (Kelvin Connection)。

---

## 4. LED 指示灯系统 (4 颗)

| 引脚 | 颜色 | 标签 | 功能定义 | 状态逻辑 (低电平亮) |
|:---|:---|:---|:---|:---|
| **PB12** | **蓝色** | **LED_SYS** | **系统状态** | **闪烁**: 待机正常 (心跳)<br>**常亮**: 已解锁 (Armed)<br>**灭**: 初始化中 |
| **PB13** | **白色** | **LED_MODE** | **飞行模式** | **灭**: 自稳/半自稳 (Angle/Horizon)<br>**亮**: 手动模式 (Acro)<br>**闪烁**: 返航/救援 (RTH/Rescue) |
| **PB14** | **绿色** | **LED_GPS** | **GPS 状态** | **灭**: 无 GPS 信号<br>**闪烁**: 搜星中<br>**常亮**: GPS 3D 锁定 (Ready) |
| **PB15** | **青色** | **LED_VTX** | **图传/警告** | **灭**: 正常<br>**亮**: 图传处于 PIT 模式 (低功率)<br>**快闪**: 电池低压/故障警告 |

---

## 5. DMA 通道分配

由于串口位置变更，DMA 通道需重新确认：

*   **USART1 (VTX)**: PB6/PB7 (重映射). DMA2_Stream7 (TX) / DMA2_Stream2 (RX). **OK**.
*   **USART2 (光流)**: PA2/PA3. DMA1_Stream6 (TX) / DMA1_Stream5 (RX). **OK**.
*   **USART3 (ELRS)**: PB10/PB11. DMA1_Stream3 (TX) / DMA1_Stream1 (RX). **OK**.
*   **UART4 (Log)**: PC10/PC11. DMA1_Stream4 (TX) / DMA1_Stream2 (RX). **OK**.
*   **UART5 (GPS)**: PC12/PD2. DMA1_Stream7 (TX) / DMA1_Stream0 (RX). **OK**.

---

## 6. 备用引脚汇总

| 引脚 | 位置 | 可用功能 | 建议用途 |
|:---|:---|:---|:---|
| PC13 | 左上 | GPIO (RTC 备份域) | 备用 LED 或使能信号 |
| PC14 | 左上 | GPIO (LSE) | 备用 (需禁用 LSE) |
| PC15 | 左上 | GPIO (LSE) | 备用 (需禁用 LSE) |
| PA0 | 左下 | GPIO/ADC | Power_Key 或备用 ADC |
| PA1 | 左下 | GPIO/ADC | 备用 ADC |
| PA9/PA10 | 右侧 | GPIO/USART1 | 备用 (原 USART1) |
| PA15 | 右上 | GPIO/TIM2_CH1 | 备用 PWM 或 GPIO |
| PB0/PB1/PB2 | 底部 | GPIO/ADC | 备用 |
| PB3/PB4 | 顶部 | GPIO/SPI3 | 备用 |
| PC3 | 左侧 | GPIO/ADC | 备用 |