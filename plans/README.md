# 🚁 FLYAMASTER 飞控项目

**项目名称**: FLYAMASTER - F405 Mini 高性能穿越机飞控  
**版本**: v2.0.0  
**最后更新**: 2026-01-22  
**状态**: 🟢 开发中 (软件完成，硬件设计中)

---

## 📖 项目简介

FLYAMASTER 是一款基于 **STM32F405RGT6** 的高性能、开源、易扩展的四轴飞行控制器。

### 核心特性

- **高性能 MCU**: STM32F405RGT6 (168MHz, 1MB Flash, 192KB RAM)
- **实时操作系统**: FreeRTOS CMSIS_V2
- **姿态解算**: Mahony 互补滤波 (500Hz)
- **控制算法**: 串级 PID (角度环 + 角速度环)
- **传感器融合**: IMU + 气压计 + 磁力计 + GPS + 光流
- **OSD 显示**: AT7456E 视频叠加
- **黑匣子**: W25Q128 Flash 日志记录

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
| **EDA** | 立创 EDA / KiCad | PCB 设计 |
| **调试** | 匿名上位机 V7 | 实时数据可视化 |

---

## 📚 文档索引

| 编号 | 文档名称 | 说明 | 链接 |
|------|----------|------|------|
| 00 | README.md | 项目总览与索引 (本文档) | - |
| 01 | 01_definitions.md | 命名规范与接口定义 | [查看](01_definitions.md) |
| 02 | 02_project_roadmap.md | 项目路线图与待办事项 | [查看](02_project_roadmap.md) |
| 03 | 03_issue_tracker.md | 问题追踪日志 (Bug/Risk) | [查看](03_issue_tracker.md) |
| 04 | 04_algo_design.md | 算法设计与优化记录 | [查看](04_algo_design.md) |
| 05 | 05_changelog.md | 版本更新日志 | [查看](05_changelog.md) |
| 06 | 06_operation_guide.md | 操作指南与环境搭建 | [查看](06_operation_guide.md) |
| 07 | 07_research_and_ideas.md | 调研分析与创意池 | [查看](07_research_and_ideas.md) |
| 08 | 08_hardware_design.md | 硬件架构与 PCB 设计 | [查看](08_hardware_design.md) |

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

1. 打开 `MDK-ARM/FLYAMASTER.uvprojx`
2. 点击 `Build` (F7) 编译
3. 点击 `Download` (F8) 烧录

### 3. 调试连接

- **SWD 接口**: PA13 (SWDIO), PA14 (SWCLK)
- **USB 虚拟串口**: PA11/PA12 (CDC)
- **匿名上位机**: UART5 (PC12/PD2) @ 115200 baud

---

## 📁 目录结构

```
FLYAMASTER/
├── Core/
│   ├── Inc/                 # 头文件
│   │   ├── mpu6050.h        # IMU 驱动
│   │   ├── mahony.h         # 姿态解算
│   │   ├── pid.h            # PID 控制器
│   │   ├── motor.h          # 电机混控
│   │   ├── sbus.h           # 遥控器接收
│   │   ├── gps.h            # GPS 解析
│   │   ├── at7456e.h        # OSD 驱动
│   │   └── ...
│   └── Src/                 # 源文件
│       ├── freertos.c       # RTOS 任务
│       ├── mpu6050.c        # IMU 驱动实现
│       ├── mahony.c         # Mahony 滤波
│       ├── pid.c            # PID 算法
│       ├── motor.c          # 电机控制
│       └── ...
├── MDK-ARM/                 # Keil 工程
│   ├── FLYAMASTER.uvprojx   # 工程文件
│   └── ...
└── plans/                   # 项目文档
    ├── README.md            # 本文档
    ├── 01_definitions.md    # 接口定义
    ├── 02_project_roadmap.md # 路线图
    ├── 03_issue_tracker.md  # 问题追踪
    ├── 04_algo_design.md    # 算法设计
    ├── 05_changelog.md      # 更新日志
    ├── 06_operation_guide.md # 操作指南
    ├── 07_research_and_ideas.md # 调研分析
    ├── 08_hardware_design.md # 硬件设计
    └── archive/             # 废弃文档归档
```

---

## 🎯 项目状态

### ✅ 已完成

- [x] 硬件驱动 (IMU, PWM, SBUS, GPS, OSD)
- [x] 姿态解算 (Mahony 互补滤波)
- [x] 姿态控制 (串级 PID)
- [x] 定高控制 (气压计 PID)
- [x] GPS 定点 (位置环 PID)
- [x] OSD 显示 (AT7456E)
- [x] 匿名上位机协议 (V7)

### 🚧 进行中

- [ ] 硬件原理图设计
- [ ] PCB 布局
- [ ] 打样验证

### 📋 计划中

- [ ] EKF 状态估计 (可选)
- [ ] 航点飞行
- [ ] 自动返航 (RTH)

---

## 👥 贡献者

- **Q版 Doro** - 项目负责人
- **Kilo Code** - AI 辅助开发

---

## 📄 许可证

本项目采用 MIT 许可证开源。

---

**文档版本**: v2.0.0  
**最后更新**: 2026-01-22