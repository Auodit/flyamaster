# 🚁 FLYAMASTER 飞控项目

> ⚠️ **版权警告 / COPYRIGHT WARNING** ⚠️
>
> **本项目受版权保护，严格保留所有权利！**
>
> 🚫 **禁止** 未经授权的复制、修改、分发、商用
> 🚫 **禁止** Fork、Clone、Download 本仓库
> 🚫 **禁止** 转载、引用本项目任何内容
>
> 违者将依法追究法律责任！详见 [LICENSE](LICENSE)
>
> ---
>
> **This project is protected by copyright. All rights strictly reserved!**
>
> 🚫 **PROHIBITED**: Unauthorized copying, modification, distribution, commercial use
> 🚫 **PROHIBITED**: Fork, Clone, Download this repository
> 🚫 **PROHIBITED**: Repost or cite any content from this project
>
> Violators will be prosecuted! See [LICENSE](LICENSE)

---

**项目名称**: FLYAMASTER - F405 Mini 高性能穿越机飞控
**版本**: v2.4.0
**最后更新**: 2026-01-23
**状态**: 🟢 开发中 (软件完成，硬件设计完成，等待验证)

---

## 📖 项目简介

FLYAMASTER 是一款基于 **STM32F405RGT6** 的高性能、开源、易扩展的四轴飞行控制器。

### 🎯 多固件支持

本硬件设计兼容多种主流开源飞控固件：

| 固件 | 适用场景 | 支持状态 |
|------|----------|----------|
| **INAV** | 固定翼 / 长航时无人机 / GPS 导航 | ✅ 完全兼容 |
| **Betaflight** | 穿越机 / 竞速无人机 / 花飞 | ✅ 完全兼容 |
| **ArduPilot** | 专业航拍 / 测绘 / 自主任务 | ✅ 完全兼容 |
| **FLYAMASTER (自研)** | 学习研究 / 定制开发 | ✅ 原生支持 |

> 💡 **提示**: 硬件引脚分配遵循 Betaflight 标准，可直接使用 Betaflight Configurator 进行配置。
> 对于 INAV 和 ArduPilot，需要创建对应的 Target 定义文件。

### 核心特性

- **高性能 MCU**: STM32F405RGT6 (168MHz, 1MB Flash, 192KB RAM)
- **多固件兼容**: 支持 INAV / Betaflight / ArduPilot / 自研固件
- **实时操作系统**: FreeRTOS CMSIS_V2 (自研固件)
- **姿态解算**: Mahony 互补滤波 (500Hz)
- **控制算法**: 串级 PID (角度环 + 角速度环)
- **传感器融合**: IMU + 气压计 + 磁力计 + GPS + 光流
- **遥控接收**: ELRS/CRSF 协议 (420kbps, 双向遥测)
- **OSD 显示**: AT7456E 视频叠加
- **黑匣子**: W25Q128 Flash 日志记录 (16MB)
- **状态指示**: 7 颗专业级 LED 飞行仪表盘

### 设计哲学

1. **控制回路隔离**: 姿态解算与 PID 控制拥有最高优先级
2. **DMA 满载策略**: 所有高吞吐量接口强制开启 DMA
3. **资源冲突零容忍**: 针对 F405 的 DMA 通道冲突采用硬件重映射方案

---

## 🛠️ 技术栈

| 类别 | 技术选型 | 说明 |
|------|----------|------|
| **MCU** | STM32F405RGT6 | ARM Cortex-M4F @ 168MHz |
| **RTOS** | FreeRTOS | CMSIS_V2 接口 |
| **IDE** | Keil MDK-ARM | ARM Compiler 6 |
| **配置工具** | STM32CubeMX | HAL 库生成 |
| **EDA** | KiCad 9.0 | PCB 设计 |
| **调试** | 匿名上位机 V7 | 实时数据可视化 |

---

## 📐 硬件设计

### 原理图

📄 **原理图 PDF**: [Hardware/zero star/zero star.pdf](Hardware/zero%20star/zero%20star.pdf)

### 主要芯片

| 芯片 | 功能 | 接口 |
|------|------|------|
| STM32F405RGT6 | 主控 MCU | - |
| MPU6050 | 6轴 IMU | I2C1 (0x68) |
| SPL06-001 | 气压计 | I2C1 (0x76) |
| QMC5883L | 磁力计 | I2C1 (0x0D) |
| W25Q128 | 16MB Flash | SPI1 |
| AT7456E | OSD 视频叠加 | SPI1 |
| SY8303 | DCDC 5V/3A | - |
| RT9193-33 | LDO 3.3V/500mA | - |

### 接口定义

| 接口 | 引脚 | 功能 | 波特率 |
|------|------|------|--------|
| J1 | USB-C | 固件升级/调参 | CDC |
| J2 | PA2/PA3 | GPS 模块 | 115200 |
| J3 | PA9/PA10 | K210 上位机 | 921600 |
| J4 | PC6-PC9 | 4-in-1 ESC | PWM 400Hz |
| J5 | PC12/PD2 | 光流模块 | 115200 |
| J6 | PC10/PC11 | VTX SmartAudio | 4800 |
| J22 | PB10/PB11 | ELRS 接收机 | 420000 (CRSF) |

---

## 📚 文档索引

| 编号 | 文档名称 | 说明 | 链接 |
|------|----------|------|------|
| 00 | README.md | 项目总览与索引 (本文档) | - |
| 01 | 01_definitions.md | 命名规范与接口定义 | [查看](plans/01_definitions.md) |
| 02 | 02_project_roadmap.md | 项目路线图与待办事项 | [查看](plans/02_project_roadmap.md) |
| 03 | 03_issue_tracker.md | 问题追踪日志 (Bug/Risk) | [查看](plans/03_issue_tracker.md) |
| 04 | 04_algo_design.md | 算法设计与优化记录 | [查看](plans/04_algo_design.md) |
| 05 | 05_changelog.md | 版本更新日志 | [查看](plans/05_changelog.md) |
| 06 | 06_operation_guide.md | 操作指南与环境搭建 | [查看](plans/06_operation_guide.md) |
| 07 | 07_research_and_ideas.md | 调研分析与创意池 | [查看](plans/07_research_and_ideas.md) |
| 08 | 08_hardware_design.md | 硬件架构与 PCB 设计 | [查看](plans/08_hardware_design.md) |

---

## 🚀 快速开始

### 1. 环境准备

```bash
# 必需软件
- Keil MDK-ARM v5.38+
- STM32CubeMX v6.10+
- ST-Link 驱动
```

### 2. 编译项目

1. 打开 `FLYAMASTER/MDK-ARM/FLYAMASTER.uvprojx`
2. 点击 `Build` (F7) 编译
3. 点击 `Download` (F8) 烧录

### 3. 调试连接

- **SWD 接口**: PA13 (SWDIO), PA14 (SWCLK)
- **USB 虚拟串口**: PA11/PA12 (CDC)
- **匿名上位机**: USART1 (PA9/PA10) @ 921600 baud

---

## 📁 目录结构

```
FLYAMASTER/
├── 3D/                      # 3D 模型文件
├── FLYAMASTER/              # 固件源代码
│   ├── Core/
│   │   ├── Inc/             # 头文件
│   │   │   ├── mpu6050.h    # IMU 驱动
│   │   │   ├── mahony.h     # 姿态解算
│   │   │   ├── pid.h        # PID 控制器
│   │   │   ├── motor.h      # 电机混控
│   │   │   ├── sbus.h       # CRSF 接收
│   │   │   ├── gps.h        # GPS 解析
│   │   │   ├── at7456e.h    # OSD 驱动
│   │   │   └── ...
│   │   └── Src/             # 源文件
│   │       ├── freertos.c   # RTOS 任务
│   │       ├── mpu6050.c    # IMU 驱动实现
│   │       ├── mahony.c     # Mahony 滤波
│   │       ├── pid.c        # PID 算法
│   │       ├── motor.c      # 电机控制
│   │       └── ...
│   └── MDK-ARM/             # Keil 工程
│       ├── FLYAMASTER.uvprojx
│       └── ...
├── Hardware/                # 硬件设计
│   └── zero star/           # KiCad 工程
│       ├── zero star.kicad_sch    # 原理图
│       ├── zero star.kicad_pcb    # PCB
│       └── zero star.pdf          # 原理图 PDF
├── plans/                   # 项目文档
│   ├── README.md            # 文档索引
│   ├── 01_definitions.md    # 接口定义
│   ├── 02_project_roadmap.md # 路线图
│   ├── 03_issue_tracker.md  # 问题追踪
│   ├── 04_algo_design.md    # 算法设计
│   ├── 05_changelog.md      # 更新日志
│   ├── 06_operation_guide.md # 操作指南
│   ├── 07_research_and_ideas.md # 调研分析
│   ├── 08_hardware_design.md # 硬件设计
│   └── archive/             # 废弃文档归档
├── .gitignore               # Git 忽略规则
└── LICENSE                  # 版权声明
```

---

## 🎯 项目状态

### ✅ 已完成

- [x] 硬件驱动 (IMU, PWM, CRSF, GPS, OSD)
- [x] 姿态解算 (Mahony 互补滤波)
- [x] 姿态控制 (串级 PID)
- [x] 定高控制 (气压计 PID)
- [x] GPS 定点 (位置环 PID)
- [x] OSD 显示 (AT7456E)
- [x] 匿名上位机协议 (V7)
- [x] 硬件原理图设计 (KiCad)
- [x] 7 颗 LED 状态指示系统
- [x] USB 插入检测 (反灌保护)

### 🚧 进行中

- [ ] PCB 布局优化
- [ ] 打样验证

### 📋 计划中

- [ ] EKF 状态估计 (可选)
- [ ] 航点飞行
- [ ] 自动返航 (RTH)

---

## 👥 贡献者

- **Doro** - 项目负责人 & 开发者

---

## ⚖️ 版权声明

```
================================================================================
                    FLYAMASTER 飞控项目 - 版权声明
================================================================================

版权所有 (C) 2026 FLYAMASTER 项目组
Copyright (C) 2026 FLYAMASTER Project Team

                       严格保留所有权利
                  ALL RIGHTS STRICTLY RESERVED

未经版权所有者明确书面授权，严禁：
1. 复制、下载、存储本项目的任何部分
2. 修改、改编、翻译本项目的任何内容
3. 分发、传播、共享本项目的任何文件
4. 将本项目用于任何商业目的
5. 基于本项目创建衍生作品
6. 对本项目进行逆向工程
7. 在任何平台转载、引用本项目内容

任何未经授权的使用将被视为侵权，版权所有者保留追究法律责任的权利。

详见 LICENSE 文件
================================================================================
```

---

**文档版本**: v2.4.0
**最后更新**: 2026-01-23