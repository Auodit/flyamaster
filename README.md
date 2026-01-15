# 🚁 FLYAMASTER - STM32F407 & Raspberry Pi Dual-Core Drone
# 基于 STM32F407 + 树莓派的双主控四轴飞行器

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![STM32](https://img.shields.io/badge/STM32-F407ZGT6-green.svg)
![RTOS](https://img.shields.io/badge/OS-FreeRTOS-orange.svg)

## 📖 项目简介 (Introduction)
本项目是一个高性能的双主控无人机飞控系统。
- **下位机 (STM32F407)**: 运行 FreeRTOS，负责 500Hz 姿态解算 (Mahony)、串级 PID 控制、传感器数据融合。
- **上位机 (Raspberry Pi)**: 负责高层指令发送、计算机视觉处理 (待开发)。

## ✨ 核心功能 (Features)
- [x] **姿态解算**: MPU6050 + Mahony 互补滤波 (500Hz)
- [x] **飞行控制**: 串级 PID (角度环 + 角速度环) + 混控算法
- [x] **定高定点**: SPL06 气压计 + 光流模块 (LC302) 融合
- [x] **安全机制**: 炸机检测、低压报警、失控保护、AirMode 怠速
- [x] **黑匣子**: W25Q128 Flash 参数存储与日志记录
- [x] **通信**: 自定义串口协议 + 兼容匿名上位机 V7

## 🛠️ 硬件清单 (Hardware)
| 模块 | 型号 | 接口 |
|---|---|---|
| 主控 | STM32F407ZGT6 | - |
| IMU | MPU6050 | I2C1 |
| 气压计 | SPL06-001 | I2C2 |
| 光流 | 优象 LC302 | UART4 |
| 接收机 | SBUS 协议 | USART3 |

## 📂 目录结构 (Directory)
- `Core/Src`: 飞控核心代码 (PID, Mahony, Drivers)
- `Docs/`: [详细开发文档与硬件连线图](./Docs/)

## 🚀 快速开始 (Getting Started)
1. 使用 Keil MDK v5 打开 `MDK-ARM/FLYAMASTER.uvprojx`
2. 编译并下载代码
3. 连接匿名上位机 V7 查看波形

## 📄 许可证 (License)
MIT License