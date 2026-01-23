# 📜 版本更新日志 (Changelog)

**创建时间**: 2026-01-18  
**状态**: 活跃维护中  
**遵循规范**: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

---

## [Unreleased] - 2026-01-18

### Added (新增)
- **OSD 驱动**: 新增 `at7456e.c` 和 `at7456e.h`，支持 AT7456E 芯片的 SPI 通信、字符显示和显存刷新。
- **OSD 显示**: 在 `comm_task` 中实现了 10Hz 的 OSD 刷新逻辑，显示飞行模式、姿态、电压、GPS 状态等关键信息。
- **GPS 导航逻辑**: 在 `ctl_task` 中实现了 GPS 位置环 PID 控制，支持定点模式 (Position Hold)。
- **堆栈扩容**: 将 `comm_task` 的堆栈大小从 256 Words 增加到 512 Words，以防止 `snprintf` 导致堆栈溢出。

### Fixed (修复)
- **Risk #111 (高频 printf)**: 移除了 `imu_task` (500Hz) 和 `ctl_task` (250Hz) 中的所有阻塞式 `printf` 调用，消除了控制回路抖动风险。
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