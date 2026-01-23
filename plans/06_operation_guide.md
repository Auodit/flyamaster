# 📖 操作指南与项目总结 (Operation Guide)

> 项目: FLYAMASTER 飞控系统  
> 版本: v2.0.0  
> 更新日期: 2026-01-22

---

## 1. 环境搭建 (Environment Setup)

### 1.1 软件需求
- **IDE**: Keil MDK-ARM v5.30 或更高版本
- **编译器**: ARM Compiler 5 (AC5) 或 ARM Compiler 6 (AC6)
  - *注意*: 本项目针对 AC5 进行了优化，使用 AC6 可能需要修改部分汇编代码。
- **配置工具**: STM32CubeMX v6.0 或更高版本
- **驱动**: ST-Link USB Driver

### 1.2 Keil MDK 配置
1. 打开 `MDK-ARM/FLYAMASTER.uvprojx` 工程文件。
2. 点击 **Options for Target** (魔术棒图标)。
3. **C/C++ 选项卡**:
   - Define: `USE_HAL_DRIVER,STM32F405xx`
   - Optimization: `-O2` (推荐) 或 `-O3`
4. **Target 选项卡**:
   - Floating Point Hardware: **Single Precision** (必须开启 FPU)
   - Code Generation: **Thumb Mode**
5. **Debug 选项卡**:
   - 选择 **ST-Link Debugger**
   - Settings -> Flash Download -> 勾选 **Reset and Run**

### 1.3 CubeMX 配置注意事项
如果需要重新生成代码，请注意以下关键配置：
- **SYS**: Debug = Serial Wire, Timebase Source = **TIM4** (避免与 FreeRTOS 冲突)
- **FreeRTOS**: Interface = **CMSIS_V2**
- **USB_OTG_FS**: Mode = Device_Only, Class = CDC (VCP)
- **TIM1**: Channel 1 = PWM Generation (用于 WS2812)
- **TIM8**: Channel 1-4 = PWM Generation (用于电机)

---

## 2. 烧录与调试 (Flashing & Debugging)

### 2.1 硬件连接
- 使用 ST-Link V2 或 V3 连接飞控板的 SWD 接口：
  - **SWCLK** -> PA14
  - **SWDIO** -> PA13
  - **GND** -> GND
  - **3.3V** -> 3.3V (如果飞控板未通过电池供电)

### 2.2 烧录步骤
1. 编译工程 (F7)，确保无 Error。
2. 点击 **Download** (F8) 按钮进行烧录。
3. 观察 IDE 下方的 Build Output 窗口，确认 "Programming Done. Verify OK."。

### 2.3 串口调试
- 连接 USB Type-C 接口到电脑。
- 飞控会被识别为虚拟串口 (STMicroelectronics Virtual COM Port)。
- 使用串口助手 (如 PuTTY, SerialPlot) 打开对应 COM 口。
- 波特率: 任意 (USB 虚拟串口自适应)，推荐 115200。

---

## 3. 调试工具使用 (Debug Tools)

系统内置了强大的调试工具，可通过串口 CLI 或代码调用。

### 3.1 I2C 总线扫描
用于检测传感器是否正常连接。

**代码调用**:
```c
#include "debug_tools.h"
// 在初始化后调用
I2C_ScanAllBuses();
```

**预期输出**:
```
[I2C Scan] Scanning I2C1...
  Found device at 0x68 (MPU6050)
  Found device at 0x76 (SPL06)
[I2C Scan] Scanning I2C2...
  Found device at 0x0D (QMC5883L)
```

### 3.2 磁力计电流补偿标定
用于消除电机电流对磁力计的干扰。

**操作步骤**:
1. 将飞机固定在测试台上，确保周围无强磁干扰。
2. 连接串口助手，准备记录日志。
3. 在代码中调用 `MagCurrentCalib_Start()`。
4. **阶段 1 (低油门)**: 保持油门在 10%~20%，持续 5 秒。
5. **阶段 2 (高油门)**: 瞬间推油门到 60%~80%，持续 2 秒，然后收油。
6. 系统会自动计算补偿系数并打印。

**输出示例**:
```
[MagCalib] Calibration Done!
[MagCalib] K_x = 0.0012, K_y = -0.0005, K_z = 0.0034
[MagCalib] Copy these lines to qmc5883l.h:
#define MAG_COMP_K_X  0.0012f
#define MAG_COMP_K_Y -0.0005f
#define MAG_COMP_K_Z  0.0034f
```

---

## 4. 飞行模式说明 (Flight Modes)

通过遥控器 CH5 (AUX1) 通道切换飞行模式。

| 模式 | CH5 PWM | 说明 | 适用场景 |
|------|---------|------|----------|
| **自稳模式 (Stabilize)** | < 1300 | 摇杆控制角度，回中自动改平 | 新手入门，室内飞行 |
| **定高模式 (AltHold)** | 1300-1700 | 油门中点保持高度，摇杆控制升降速度 | 航拍，悬停 |
| **定点模式 (PosHold)** | > 1700 | 结合 GPS 和光流，自动保持位置 | 室外巡航，失控保护 |

**解锁/上锁 (Arming)**:
- **解锁**: 油门最低 + CH4 (AUX2) > 1500
- **上锁**: CH4 (AUX2) < 1500
- **注意**: 只有在传感器自检通过且 GPS 锁定 (定点模式下) 后才能解锁。

---

## 5. 常见问题排查 (Troubleshooting)

### 5.1 无法解锁
- **现象**: 拨动解锁开关，电机不转，蜂鸣器急促鸣叫。
- **原因**:
  1. 传感器自检失败 (检查 I2C 扫描结果)。
  2. 遥控器油门未归零。
  3. 电池电压过低。
  4. 倾斜角度过大 (> 25度)。
  5. 定点模式下 GPS 未锁定 (红灯闪烁)。

### 5.2 姿态漂移
- **现象**: 飞机起飞后自动向一个方向倾斜。
- **解决**:
  1. 执行加速度计校准 (放置水平地面，上电静止 5 秒)。
  2. 检查螺旋桨是否安装正确。
  3. 检查电机震动是否过大 (可能需要调整 DLPF 参数)。

### 5.3 航向不准 (画圈)
- **现象**: 定点模式下飞机绕圈飞行 (马桶效应)。
- **解决**:
  1. 执行磁力计校准 (8字校准)。
  2. 检查周围是否有磁干扰 (如蜂鸣器、电源线)。
  3. 执行磁力计电流补偿标定。

### 5.4 OSD 无显示
- **现象**: FPV 画面正常但无字符叠加。
- **解决**:
  1. 检查 OSD 芯片供电 (5V)。
  2. 检查 SPI1 通信是否正常。
  3. 确认摄像头制式 (PAL/NTSC) 与 OSD 设置一致。

---

## 6. CLI 指令集 (Command Line Interface)

通过 USB 虚拟串口发送指令 (波特率 115200, 换行符 `\n`)。

| 指令 | 参数 | 说明 |
|------|------|------|
| `help` | - | 显示帮助信息 |
| `status` | - | 显示系统状态 (CPU, 电池, 传感器) |
| `tasks` | - | 显示 FreeRTOS 任务列表和 CPU 占用率 |
| `scan` | - | 执行 I2C 总线扫描 |
| `calib_acc` | - | 执行加速度计校准 |
| `calib_mag` | - | 执行磁力计校准 |
| `save` | - | 保存参数到 Flash |
| `reboot` | - | 重启飞控 |

---

**维护人**: Q版 Doro  
**技术支持**: 请提交 Issue 到 `03_issue_tracker.md`