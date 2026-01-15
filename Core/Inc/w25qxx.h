/**
  ******************************************************************************
  * @file           : w25qxx.h
  * @brief          : W25Q128 SPI Flash Driver Header
  * @author         : Doro (Flight Controller Team)
  * @date           : 2026-01-14
  ******************************************************************************
  * @attention
  * 
  * W25Q128 SPI Flash 驱动 - 用于参数存储和黑匣子功能
  * 
  * 硬件连接:
  *   CS:   PA4  (GPIO)
  *   SCK:  PA5  (SPI1_SCK)
  *   MISO: PA6  (SPI1_MISO)
  *   MOSI: PA7  (SPI1_MOSI)
  * 
  * 芯片规格:
  *   - 容量: 16MB (128Mbit)
  *   - 擦写寿命: 100,000 次
  *   - Sector 大小: 4KB
  *   - Page 大小: 256 Bytes
  *
  ******************************************************************************
  */

#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

// W25Q128 指令集
#define W25QXX_CMD_WRITE_ENABLE         0x06  // 写使能
#define W25QXX_CMD_WRITE_DISABLE        0x04  // 写禁止
#define W25QXX_CMD_READ_STATUS_REG1     0x05  // 读状态寄存器1
#define W25QXX_CMD_READ_STATUS_REG2     0x35  // 读状态寄存器2
#define W25QXX_CMD_WRITE_STATUS_REG     0x01  // 写状态寄存器
#define W25QXX_CMD_PAGE_PROGRAM         0x02  // 页编程
#define W25QXX_CMD_SECTOR_ERASE         0x20  // 扇区擦除 (4KB)
#define W25QXX_CMD_BLOCK_ERASE_32K      0x52  // 块擦除 (32KB)
#define W25QXX_CMD_BLOCK_ERASE_64K      0xD8  // 块擦除 (64KB)
#define W25QXX_CMD_CHIP_ERASE           0xC7  // 芯片擦除
#define W25QXX_CMD_READ_DATA            0x03  // 读数据
#define W25QXX_CMD_FAST_READ            0x0B  // 快速读
#define W25QXX_CMD_READ_JEDEC_ID        0x9F  // 读 JEDEC ID
#define W25QXX_CMD_POWER_DOWN           0xB9  // 掉电模式
#define W25QXX_CMD_RELEASE_POWER_DOWN   0xAB  // 释放掉电

// W25Q128 参数
#define W25QXX_PAGE_SIZE                256   // 页大小 (字节)
#define W25QXX_SECTOR_SIZE              4096  // 扇区大小 (字节)
#define W25QXX_BLOCK_SIZE_32K           32768 // 块大小 32KB
#define W25QXX_BLOCK_SIZE_64K           65536 // 块大小 64KB
#define W25QXX_CHIP_SIZE                16777216  // 芯片大小 16MB

// W25Q128 JEDEC ID
#define W25Q128_JEDEC_ID                0xEF4018  // W25Q128 的 ID

// 状态寄存器位定义
#define W25QXX_STATUS_BUSY              0x01  // Busy 标志位
#define W25QXX_STATUS_WEL               0x02  // Write Enable Latch

// 超时设置
#define W25QXX_TIMEOUT_PAGE_PROGRAM     5     // 页编程超时 (ms)
#define W25QXX_TIMEOUT_SECTOR_ERASE     400   // 扇区擦除超时 (ms)
#define W25QXX_TIMEOUT_BLOCK_ERASE      2000  // 块擦除超时 (ms)
#define W25QXX_TIMEOUT_CHIP_ERASE       100000 // 芯片擦除超时 (ms)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化 W25QXX Flash
  * @param  hspi: SPI 句柄指针
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_Init(SPI_HandleTypeDef *hspi);

/**
  * @brief  读取 JEDEC ID (用于识别芯片型号)
  * @param  None
  * @retval JEDEC ID (0xEF4018 for W25Q128)
  */
uint32_t W25QXX_ReadID(void);

/**
  * @brief  读取数据
  * @param  addr: 起始地址 (0x000000 ~ 0xFFFFFF)
  * @param  buf: 数据缓冲区指针
  * @param  len: 读取长度
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_Read(uint32_t addr, uint8_t *buf, uint16_t len);

/**
  * @brief  写入数据 (自动处理页边界)
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区指针
  * @param  len: 写入长度
  * @retval 0: 成功, 1: 失败
  * @note   此函数会自动擦除所需的 Sector
  */
uint8_t W25QXX_Write(uint32_t addr, uint8_t *buf, uint16_t len);

/**
  * @brief  擦除扇区 (4KB)
  * @param  sector_addr: 扇区地址 (必须是 4KB 对齐)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_EraseSector(uint32_t sector_addr);

/**
  * @brief  擦除块 (64KB)
  * @param  block_addr: 块地址 (必须是 64KB 对齐)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_EraseBlock(uint32_t block_addr);

/**
  * @brief  擦除整个芯片
  * @param  None
  * @retval 0: 成功, 1: 失败
  * @note   此操作需要很长时间 (~100秒)
  */
uint8_t W25QXX_EraseChip(void);

/**
  * @brief  进入掉电模式
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_PowerDown(void);

/**
  * @brief  唤醒芯片
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WakeUp(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  写使能并验证WEL位
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WriteEnable(void);

/**
  * @brief  写禁止
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WriteDisable(void);

/**
  * @brief  等待 Flash 空闲
  * @param  timeout_ms: 超时时间 (毫秒)
  * @retval 0: 成功, 1: 超时
  */
uint8_t W25QXX_WaitBusy(uint32_t timeout_ms);

/**
  * @brief  读取状态寄存器
  * @param  reg_num: 寄存器编号 (1 或 2)
  * @retval 状态寄存器值
  */
uint8_t W25QXX_ReadStatusReg(uint8_t reg_num);

/**
  * @brief  页编程 (最多 256 字节)
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区
  * @param  len: 长度 (1~256)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_PageProgram(uint32_t addr, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */


