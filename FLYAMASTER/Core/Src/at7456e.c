/**
 * @file at7456e.c
 * @brief AT7456E OSD 芯片驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "at7456e.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define AT7456E_SPI         hspi1
#define AT7456E_CS_LOW()    HAL_GPIO_WritePin(OSD_CS_GPIO_Port, OSD_CS_Pin, GPIO_PIN_RESET)
#define AT7456E_CS_HIGH()   HAL_GPIO_WritePin(OSD_CS_GPIO_Port, OSD_CS_Pin, GPIO_PIN_SET)

/* AT7456E 寄存器地址 */
#define AT7456E_VM0         0x00    /* Video Mode 0 */
#define AT7456E_VM1         0x01    /* Video Mode 1 */
#define AT7456E_HOS         0x02    /* Horizontal Offset */
#define AT7456E_VOS         0x03    /* Vertical Offset */
#define AT7456E_DMM         0x04    /* Display Memory Mode */
#define AT7456E_DMAH        0x05    /* Display Memory Address High */
#define AT7456E_DMAL        0x06    /* Display Memory Address Low */
#define AT7456E_DMDI        0x07    /* Display Memory Data In */
#define AT7456E_CMM         0x08    /* Character Memory Mode */
#define AT7456E_CMAH        0x09    /* Character Memory Address High */
#define AT7456E_CMAL        0x0A    /* Character Memory Address Low */
#define AT7456E_CMDI        0x0B    /* Character Memory Data In */
#define AT7456E_OSDM        0x0C    /* OSD Insertion Mux */
#define AT7456E_RB0         0x10    /* Row 0 Brightness */
#define AT7456E_OSDBL       0x6C    /* OSD Black Level */
#define AT7456E_STAT        0xA0    /* Status Register (Read) */

/* VM0 寄存器位定义 */
#define VM0_VIDEO_BUF_EN    (1 << 0)
#define VM0_SOFT_RST        (1 << 1)
#define VM0_SYNC_SEL_AUTO   (1 << 2)
#define VM0_OSD_EN          (1 << 3)
#define VM0_SYNC_SEL_INT    (1 << 4)
#define VM0_PAL             (1 << 6)

/* DMM 寄存器位定义 */
#define DMM_CLEAR_DISPLAY   (1 << 2)
#define DMM_INV             (1 << 3)
#define DMM_BLK             (1 << 4)
#define DMM_LBC             (1 << 5)
#define DMM_AUTO_INC        (1 << 0)

/* Private variables ---------------------------------------------------------*/
static uint8_t at7456e_initialized = 0;
static uint8_t at7456e_video_mode = 0;  /* 0=NTSC, 1=PAL */

/* Private function prototypes -----------------------------------------------*/
/* Note: AT7456E_WriteReg and AT7456E_ReadReg are declared in header as public */

/* Exported functions --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
AT7456E_State_t g_osd_state = {0};

/**
 * @brief 初始化 AT7456E OSD 芯片
 * @return bool true=成功, false=失败
 */
bool AT7456E_Init(void)
{
    /* 确保 CS 为高 */
    AT7456E_CS_HIGH();
    HAL_Delay(1);
    
    /* 软复位 */
    AT7456E_WriteReg(AT7456E_VM0, VM0_SOFT_RST);
    HAL_Delay(100);
    
    /* 检测视频制式 (读取状态寄存器) */
    uint8_t stat = AT7456E_ReadReg(AT7456E_STAT);
    if (stat & 0x01) {
        at7456e_video_mode = 1;  /* PAL */
        g_osd_state.mode = AT7456E_MODE_PAL;
        g_osd_state.rows = AT7456E_ROWS_PAL;
    } else {
        at7456e_video_mode = 0;  /* NTSC */
        g_osd_state.mode = AT7456E_MODE_NTSC;
        g_osd_state.rows = AT7456E_ROWS_NTSC;
    }
    g_osd_state.cols = AT7456E_COLS;
    
    /* 配置 VM0: 使能 OSD, 自动同步 */
    uint8_t vm0 = VM0_OSD_EN | VM0_SYNC_SEL_AUTO;
    if (at7456e_video_mode) {
        vm0 |= VM0_PAL;
    }
    AT7456E_WriteReg(AT7456E_VM0, vm0);
    
    /* 配置 VM1: 默认设置 */
    AT7456E_WriteReg(AT7456E_VM1, 0x00);
    
    /* 设置水平/垂直偏移 */
    AT7456E_WriteReg(AT7456E_HOS, 0x20);  /* 居中 */
    AT7456E_WriteReg(AT7456E_VOS, 0x10);  /* 居中 */
    
    /* 设置 OSD 黑电平 */
    AT7456E_WriteReg(AT7456E_OSDBL, 0x00);
    
    /* 清屏 */
    AT7456E_ClearScreen();
    
    at7456e_initialized = 1;
    g_osd_state.initialized = true;
    
    return true;
}

/**
 * @brief 清除 OSD 显示
 */
void AT7456E_ClearScreen(void)
{
    AT7456E_WriteReg(AT7456E_DMM, DMM_CLEAR_DISPLAY);
    HAL_Delay(30);  /* 等待清屏完成 */
}

/**
 * @brief 清除 OSD 显示 (别名，兼容旧代码)
 */
void AT7456E_Clear(void)
{
    AT7456E_ClearScreen();
}

/**
 * @brief 在指定位置写入单个字符
 * @param col 列 (0-29)
 * @param row 行 (0-15 for PAL, 0-12 for NTSC)
 * @param ch 字符 (ASCII 或自定义字符索引)
 */
void AT7456E_WriteChar(uint8_t col, uint8_t row, char ch)
{
    uint16_t addr = row * AT7456E_COLS + col;
    
    /* 设置地址 */
    AT7456E_WriteReg(AT7456E_DMAH, (addr >> 8) & 0x01);
    AT7456E_WriteReg(AT7456E_DMAL, addr & 0xFF);
    
    /* 写入字符 */
    AT7456E_WriteReg(AT7456E_DMDI, (uint8_t)ch);
}

/**
 * @brief 在指定位置写入字符串
 * @param col 起始列
 * @param row 行
 * @param str 字符串
 */
void AT7456E_WriteString(uint8_t col, uint8_t row, const char *str)
{
    uint16_t addr = row * AT7456E_COLS + col;
    
    /* 设置起始地址 */
    AT7456E_WriteReg(AT7456E_DMAH, (addr >> 8) & 0x01);
    AT7456E_WriteReg(AT7456E_DMAL, addr & 0xFF);
    
    /* 使能自动递增模式 */
    AT7456E_WriteReg(AT7456E_DMM, DMM_AUTO_INC);
    
    /* 写入字符串 */
    while (*str && col < AT7456E_COLS) {
        AT7456E_WriteReg(AT7456E_DMDI, (uint8_t)*str++);
        col++;
    }
    
    /* 写入结束标记 (0xFF) */
    AT7456E_WriteReg(AT7456E_DMDI, 0xFF);
    
    /* 关闭自动递增 */
    AT7456E_WriteReg(AT7456E_DMM, 0x00);
}

/**
 * @brief 在指定位置写入整数
 * @param col 起始列
 * @param row 行
 * @param value 整数值
 * @param digits 显示位数 (0=自动)
 */
void AT7456E_WriteInt(uint8_t col, uint8_t row, int32_t value, uint8_t digits)
{
    char buf[12];
    
    if (digits > 0) {
        snprintf(buf, sizeof(buf), "%*ld", digits, (long)value);
    } else {
        snprintf(buf, sizeof(buf), "%ld", (long)value);
    }
    
    AT7456E_WriteString(col, row, buf);
}

/**
 * @brief 在指定位置写入浮点数
 * @param col 起始列
 * @param row 行
 * @param value 浮点值
 * @param decimals 小数位数
 */
void AT7456E_WriteFloat(uint8_t col, uint8_t row, float value, uint8_t decimals)
{
    char buf[16];
    
    /* 手动格式化浮点数 (避免使用 %f) */
    int32_t int_part = (int32_t)value;
    float frac = value - (float)int_part;
    if (frac < 0) frac = -frac;
    
    int32_t frac_part = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        frac_part *= 10;
    }
    int32_t frac_int = (int32_t)(frac * frac_part + 0.5f);
    
    if (value < 0 && int_part == 0) {
        snprintf(buf, sizeof(buf), "-%ld.%0*ld", (long)int_part, decimals, (long)frac_int);
    } else {
        snprintf(buf, sizeof(buf), "%ld.%0*ld", (long)int_part, decimals, (long)frac_int);
    }
    
    AT7456E_WriteString(col, row, buf);
}

/**
 * @brief 设置字符属性 (反显/闪烁)
 * @param col 列
 * @param row 行
 * @param attr 属性 (AT7456E_ATTR_xxx)
 */
void AT7456E_SetCharAttr(uint8_t col, uint8_t row, uint8_t attr)
{
    uint16_t addr = row * AT7456E_COLS + col;
    
    /* 设置地址 (属性存储在高位地址空间) */
    AT7456E_WriteReg(AT7456E_DMAH, ((addr >> 8) & 0x01) | 0x02);
    AT7456E_WriteReg(AT7456E_DMAL, addr & 0xFF);
    
    /* 写入属性 */
    AT7456E_WriteReg(AT7456E_DMDI, attr);
}

/**
 * @brief 使能/禁用 OSD 显示
 * @param enable true=使能, false=禁用
 */
void AT7456E_Enable(bool enable)
{
    uint8_t vm0 = AT7456E_ReadReg(AT7456E_VM0);
    
    if (enable) {
        vm0 |= VM0_OSD_EN;
    } else {
        vm0 &= ~VM0_OSD_EN;
    }
    
    AT7456E_WriteReg(AT7456E_VM0, vm0);
}

/**
 * @brief 设置行亮度
 * @param row 行号
 * @param brightness 亮度 (0-3)
 */
void AT7456E_SetRowBrightness(uint8_t row, uint8_t brightness)
{
    if (row > 15) return;
    AT7456E_WriteReg(AT7456E_RB0 + row, brightness & 0x03);
}

/**
 * @brief 设置光标位置
 * @param col 列 (0-29)
 * @param row 行 (0-15 NTSC / 0-12 PAL)
 */
void AT7456E_SetCursor(uint8_t col, uint8_t row)
{
    uint16_t addr = row * AT7456E_COLS + col;
    AT7456E_WriteReg(AT7456E_DMAH, (addr >> 8) & 0x01);
    AT7456E_WriteReg(AT7456E_DMAL, addr & 0xFF);
}

/**
 * @brief 绘制电池图标
 * @param col 列
 * @param row 行
 * @param percentage 电量百分比 (0-100)
 */
void AT7456E_DrawBattery(uint8_t col, uint8_t row, uint8_t percentage)
{
    uint8_t icon;
    if (percentage > 80) {
        icon = 0x90;  /* 满电 */
    } else if (percentage > 60) {
        icon = 0x91;
    } else if (percentage > 40) {
        icon = 0x92;
    } else if (percentage > 20) {
        icon = 0x93;
    } else {
        icon = 0x94;  /* 低电 */
    }
    AT7456E_WriteChar(col, row, icon);
}

/**
 * @brief 绘制信号强度图标
 * @param col 列
 * @param row 行
 * @param strength 信号强度 (0-4)
 */
void AT7456E_DrawRSSI(uint8_t col, uint8_t row, uint8_t strength)
{
    uint8_t icon = 0x01 + (strength > 4 ? 4 : strength);
    AT7456E_WriteChar(col, row, icon);
}

/**
 * @brief 绘制人工地平线
 * @param roll 横滚角 (度)
 * @param pitch 俯仰角 (度)
 */
void AT7456E_DrawHorizon(float roll, float pitch)
{
    /* 简化实现：在屏幕中央显示姿态指示 */
    uint8_t center_row = at7456e_video_mode ? 6 : 8;  /* PAL/NTSC 中心行 */
    uint8_t center_col = 15;
    
    /* 清除旧的地平线 */
    for (uint8_t i = 12; i < 18; i++) {
        AT7456E_WriteChar(i, center_row - 1, ' ');
        AT7456E_WriteChar(i, center_row, ' ');
        AT7456E_WriteChar(i, center_row + 1, ' ');
    }
    
    /* 根据俯仰角偏移行 */
    int8_t row_offset = (int8_t)(pitch / 10.0f);
    if (row_offset > 2) row_offset = 2;
    if (row_offset < -2) row_offset = -2;
    
    /* 绘制地平线 */
    AT7456E_WriteString(12, center_row + row_offset, "------");
    
    /* 绘制飞机符号 */
    AT7456E_WriteChar(center_col, center_row, '+');
}

/**
 * @brief 设置视频制式
 * @param mode 视频制式
 */
void AT7456E_SetVideoMode(AT7456E_VideoMode_t mode)
{
    uint8_t vm0 = AT7456E_ReadReg(AT7456E_VM0);
    
    if (mode == AT7456E_MODE_PAL) {
        vm0 |= VM0_PAL;
        at7456e_video_mode = 1;
        g_osd_state.mode = AT7456E_MODE_PAL;
        g_osd_state.rows = AT7456E_ROWS_PAL;
    } else {
        vm0 &= ~VM0_PAL;
        at7456e_video_mode = 0;
        g_osd_state.mode = AT7456E_MODE_NTSC;
        g_osd_state.rows = AT7456E_ROWS_NTSC;
    }
    
    AT7456E_WriteReg(AT7456E_VM0, vm0);
}

/**
 * @brief 设置水平偏移
 * @param offset 偏移值 (-32 到 31)
 */
void AT7456E_SetHorizontalOffset(int8_t offset)
{
    AT7456E_WriteReg(AT7456E_HOS, (uint8_t)(offset + 32));
}

/**
 * @brief 设置垂直偏移
 * @param offset 偏移值 (-16 到 15)
 */
void AT7456E_SetVerticalOffset(int8_t offset)
{
    AT7456E_WriteReg(AT7456E_VOS, (uint8_t)(offset + 16));
}

/**
 * @brief 检测视频信号
 * @return bool true=有信号, false=无信号
 */
bool AT7456E_DetectVideo(void)
{
    uint8_t stat = AT7456E_ReadReg(AT7456E_STAT);
    return (stat & 0x04) ? false : true;  /* LOS bit = 0 表示有信号 */
}

/**
 * @brief SPI 片选控制
 * @param select true=选中, false=取消选中
 */
void AT7456E_CS(bool select)
{
    if (select) {
        AT7456E_CS_LOW();
    } else {
        AT7456E_CS_HIGH();
    }
}

/**
 * @brief 获取视频制式
 * @return 0=NTSC, 1=PAL
 */
uint8_t AT7456E_GetVideoMode_Legacy(void)
{
    return at7456e_video_mode;
}

/**
 * @brief 检查是否已初始化
 * @return 1=已初始化, 0=未初始化
 */
uint8_t AT7456E_IsInitialized(void)
{
    return at7456e_initialized;
}

/**
 * @brief 显示飞行数据 (综合显示函数)
 * @param voltage 电压 (V)
 * @param current 电流 (A)
 * @param mah 已消耗电量 (mAh)
 * @param rssi RSSI 值 (0-100)
 * @param armed 解锁状态
 */
void AT7456E_ShowFlightData(float voltage, float current, uint32_t mah, 
                            uint8_t rssi, uint8_t armed)
{
    /* 第一行: 电压和电流 */
    AT7456E_WriteFloat(0, 0, voltage, 1);
    AT7456E_WriteString(5, 0, "V");
    AT7456E_WriteFloat(8, 0, current, 1);
    AT7456E_WriteString(13, 0, "A");
    
    /* 第二行: mAh 和 RSSI */
    AT7456E_WriteInt(0, 1, mah, 5);
    AT7456E_WriteString(5, 1, "mAh");
    AT7456E_WriteString(12, 1, "RSSI:");
    AT7456E_WriteInt(17, 1, rssi, 3);
    AT7456E_WriteString(20, 1, "%");
    
    /* 右上角: 解锁状态 */
    if (armed) {
        AT7456E_WriteString(25, 0, "ARM");
    } else {
        AT7456E_WriteString(25, 0, "DIS");
    }
}

/* Public register access functions ------------------------------------------*/

/**
 * @brief 写入寄存器
 * @param reg 寄存器地址
 * @param data 数据
 */
void AT7456E_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    
    AT7456E_CS_LOW();
    HAL_SPI_Transmit(&AT7456E_SPI, tx, 2, 10);
    AT7456E_CS_HIGH();
}

/**
 * @brief 读取寄存器
 * @param reg 寄存器地址
 * @return uint8_t 数据
 */
uint8_t AT7456E_ReadReg(uint8_t reg)
{
    uint8_t tx = reg | 0x80;  /* 读取标志 */
    uint8_t rx = 0;
    
    AT7456E_CS_LOW();
    HAL_SPI_Transmit(&AT7456E_SPI, &tx, 1, 10);
    HAL_SPI_Receive(&AT7456E_SPI, &rx, 1, 10);
    AT7456E_CS_HIGH();
    
    return rx;
}