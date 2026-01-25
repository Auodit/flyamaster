# 🚨 待处理问题 (2026-01-25)

> 用户起床后请查看以下问题

---

## Issue #020 [HW] - CubeMX 配置与设计文档不一致

### 问题描述
CubeMX 生成的代码中 SPI 片选引脚配置与设计文档 v2.5.7 不一致：

| 引脚 | CubeMX 生成 | 设计文档 v2.5.7 |
|------|-------------|-----------------|
| PA4 | FLASH_CS | **OSD_CS** |
| PC4 | (无标签) | **FLASH_CS** |

### 影响
- `w25qxx.c` 使用 `FLASH_CS_GPIO_Port/FLASH_CS_Pin` 宏
- `at7456e.c` 使用 `OSD_CS_GPIO_Port/OSD_CS_Pin` 宏
- 如果 PCB 按设计文档布线，代码将无法正确控制芯片

### 需要确认
**请确认你的 PCB 实际布线：**
1. PA4 连接的是 Flash 还是 OSD？
2. PC4 连接的是 Flash 还是 OSD？

### 解决方案

#### 方案 A：PCB 按设计文档布线 (PA4=OSD, PC4=FLASH)
在 CubeMX 中修改引脚标签：
1. PA4 → User Label: `OSD_CS`
2. PC4 → User Label: `FLASH_CS`
3. 重新生成代码

#### 方案 B：PCB 按 CubeMX 布线 (PA4=FLASH, PC4=OSD)
修改设计文档，将 v2.5.7 的变更回滚：
1. 修改 `08_hardware_design.md`
2. 修改 `w25qxx.h` 注释
3. 修改 `at7456e.h` 注释

### 临时处理
已在 `main.h` USER CODE 区域添加 `OSD_CS_Pin/OSD_CS_GPIO_Port` 定义 (PC4)，
但这与 CubeMX 生成的 `FLASH_CS` (PA4) 存在冲突。

---

## Issue #021 [HW] - USB-C 插口空间不足

### 问题描述
PCB 布局中 USB-C 插口没有合适的放置位置。

### 建议方案
1. **背面放置**: USB-C 放在 PCB 背面，通过开孔或边缘露出
2. **侧边放置**: 使用侧插式 USB-C 连接器 (如 TYPE-C-31-M-12)
3. **FPC 转接**: 使用 FPC 软排线将 USB-C 引出到机架其他位置
4. **减小尺寸**: 使用 USB-C 16P 而非 24P 版本 (仅支持 USB 2.0)

---

## 已完成的工作

### 文档更新
- [x] `08_hardware_design.md` 更新至 v2.5.7
- [x] `05_changelog.md` 添加 HW v2.5.7 记录
- [x] `w25qxx.h` 注释更新 (CS: PC4)
- [x] `at7456e.h` 注释保持 (CS: PA4)

### 代码修改
- [x] `main.h` 添加 OSD_CS 定义 (PC4) - 临时方案

### 待执行
- [ ] Git 提交到 Gitee 和 GitHub
- [ ] 确认 PCB 实际布线后修正 CubeMX 配置

---

## Git 仓库
- Gitee: https://gitee.com/micolifar/FLYAMASTER
- GitHub: https://github.com/Auodit/flyamaster