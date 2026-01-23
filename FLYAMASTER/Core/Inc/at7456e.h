/**
 * @file at7456e.h
 * @brief AT7456E OSD 芯片驱动模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * AT7456E 是一款视频叠加 OSD 芯片，通过 SPI 接口控制。
 * 
 * 硬件连接:
 * - SPI1 (共享总线)
 * - CS: PA4 (OSD_CS)
 * - 分辨率: 30 列 x 16 行 (NTSC) / 30 列 x 13 行 (PAL)
 */

#ifndef __AT7456E_H
#define __AT7456E_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* 显示分辨率 */
#define AT7456E_COLS            30      /**< 列数 */
#define AT7456E_ROWS_NTSC       16      /**< NTSC 行数 */
#define AT7456E_ROWS_PAL        13      /**< PAL 行数 */

/* 寄存器地址 */
#define AT7456E_REG_VM0         0x00    /**< 视频模式 0 */
#define AT7456E_REG_VM1         0x01    /**< 视频模式 1 */
#define AT7456E_REG_HOS         0x02    /**< 水平偏移 */
#define AT7456E_REG_VOS         0x03    /**< 垂直偏移 */
#define AT7456E_REG_DMM         0x04    /**< 显示内存模式 */
#define AT7456E_REG_DMAH        0x05    /**< 显示内存地址高位 */
#define AT7456E_REG_DMAL        0x06    /**< 显示内存地址低位 */
#define AT7456E_REG_DMDI        0x07    /**< 显示内存数据输入 */
#define AT7456E_REG_CMM         0x08    /**< 字符内存模式 */
#define AT7456E_REG_CMAH        0x09    /**< 字符内存地址高位 */
#define AT7456E_REG_CMAL        0x0A    /**< 字符内存地址低位 */
#define AT7456E_REG_CMDI        0x0B    /**< 字符内存数据输入 */
#define AT7456E_REG_OSDM        0x0C    /**< OSD 插入复用 */
#define AT7456E_REG_RB0         0x10    /**< 行亮度 0 */
#define AT7456E_REG_OSDBL       0x6C    /**< OSD 黑电平 */
#define AT7456E_REG_STAT        0xA0    /**< 状态寄存器 (只读) */

/* VM0 寄存器位 */
#define AT7456E_VM0_OSD_EN      0x08    /**< OSD 使能 */
#define AT7456E_VM0_SYNC_AUTO   0x00    /**< 自动同步 */
#define AT7456E_VM0_SYNC_EXT    0x20    /**< 外部同步 */
#define AT7456E_VM0_SYNC_INT    0x30    /**< 内部同步 */
#define AT7456E_VM0_PAL         0x40    /**< PAL 制式 */
#define AT7456E_VM0_NTSC        0x00    /**< NTSC 制式 */

/* DMM 寄存器位 */
#define AT7456E_DMM_CLEAR       0x04    /**< 清屏 */
#define AT7456E_DMM_VSYNC_CLR   0x02    /**< 垂直同步清除 */
#define AT7456E_DMM_AUTO_INC    0x01    /**< 自动递增 */

/* 特殊字符 */
#define AT7456E_CHAR_BLANK      0x00    /**< 空白 */
#define AT7456E_CHAR_BATTERY    0x90    /**< 电池图标 */
#define AT7456E_CHAR_RSSI       0x01    /**< 信号强度 */
#define AT7456E_CHAR_HOME       0x05    /**< 返航点 */
#define AT7456E_CHAR_GPS        0x1F    /**< GPS 图标 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 视频制式枚举
 */
typedef enum {
    AT7456E_MODE_NTSC = 0,
    AT7456E_MODE_PAL
} AT7456E_VideoMode_t;

/**
 * @brief OSD 状态结构体
 */
typedef struct {
    bool initialized;           /**< 初始化标志 */
    AT7456E_VideoMode_t mode;   /**< 视频制式 */
    uint8_t rows;               /**< 行数 */
    uint8_t cols;               /**< 列数 */
} AT7456E_State_t;

/* Exported variables --------------------------------------------------------*/
extern AT7456E_State_t g_osd_state;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 AT7456E
 * @return bool true=成功, false=失败
 */
bool AT7456E_Init(void);

/**
 * @brief 写寄存器
 * @param reg 寄存器地址
 * @param data 数据
 */
void AT7456E_WriteReg(uint8_t reg, uint8_t data);

/**
 * @brief 读寄存器
 * @param reg 寄存器地址
 * @return uint8_t 数据
 */
uint8_t AT7456E_ReadReg(uint8_t reg);

/**
 * @brief 使能 OSD 显示
 * @param enable true=使能, false=禁用
 */
void AT7456E_Enable(bool enable);

/**
 * @brief 清屏
 */
void AT7456E_ClearScreen(void);

/**
 * @brief 设置光标位置
 * @param col 列 (0-29)
 * @param row 行 (0-15 NTSC / 0-12 PAL)
 */
void AT7456E_SetCursor(uint8_t col, uint8_t row);

/**
 * @brief 写入单个字符
 * @param col 列
 * @param row 行
 * @param ch 字符 (ASCII 或自定义字符索引)
 */
void AT7456E_WriteChar(uint8_t col, uint8_t row, char ch);

/**
 * @brief 写入字符串
 * @param col 起始列
 * @param row 行
 * @param str 字符串
 */
void AT7456E_WriteString(uint8_t col, uint8_t row, const char *str);

/**
 * @brief 写入整数
 * @param col 起始列
 * @param row 行
 * @param value 整数值
 * @param digits 位数 (0=自动)
 */
void AT7456E_WriteInt(uint8_t col, uint8_t row, int32_t value, uint8_t digits);

/**
 * @brief 写入浮点数
 * @param col 起始列
 * @param row 行
 * @param value 浮点值
 * @param decimals 小数位数
 */
void AT7456E_WriteFloat(uint8_t col, uint8_t row, float value, uint8_t decimals);

/**
 * @brief 绘制电池图标
 * @param col 列
 * @param row 行
 * @param percentage 电量百分比 (0-100)
 */
void AT7456E_DrawBattery(uint8_t col, uint8_t row, uint8_t percentage);

/**
 * @brief 绘制信号强度图标
 * @param col 列
 * @param row 行
 * @param strength 信号强度 (0-4)
 */
void AT7456E_DrawRSSI(uint8_t col, uint8_t row, uint8_t strength);

/**
 * @brief 绘制人工地平线
 * @param roll 横滚角 (度)
 * @param pitch 俯仰角 (度)
 */
void AT7456E_DrawHorizon(float roll, float pitch);

/**
 * @brief 设置视频制式
 * @param mode 视频制式
 */
void AT7456E_SetVideoMode(AT7456E_VideoMode_t mode);

/**
 * @brief 设置水平偏移
 * @param offset 偏移值 (-32 到 31)
 */
void AT7456E_SetHorizontalOffset(int8_t offset);

/**
 * @brief 设置垂直偏移
 * @param offset 偏移值 (-16 到 15)
 */
void AT7456E_SetVerticalOffset(int8_t offset);

/**
 * @brief 检测视频信号
 * @return bool true=有信号, false=无信号
 */
bool AT7456E_DetectVideo(void);

/**
 * @brief SPI 片选控制
 * @param select true=选中, false=取消选中
 */
void AT7456E_CS(bool select);

#ifdef __cplusplus
}
#endif

#endif /* __AT7456E_H */