/**
 * @file w25qxx.h
 * @brief W25Q128 SPI Flash 驱动模块
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * W25Q128 是一款 128Mbit (16MB) SPI Flash，用于黑匣子日志存储。
 * 
 * 硬件连接:
 * - SPI1 (PA5/PA6/PA7) - 与 OSD 共享总线
 * - CS: PB0 (FLASH_CS)
 * - 容量: 16MB (256 块 x 16 扇区 x 4KB)
 */

#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* Flash 容量定义 */
#define W25Q128_SIZE            (16 * 1024 * 1024)  /**< 16 MB */
#define W25Q128_SECTOR_SIZE     4096                /**< 4 KB 扇区 */
#define W25Q128_PAGE_SIZE       256                 /**< 256 字节页 */
#define W25Q128_BLOCK_SIZE      (64 * 1024)         /**< 64 KB 块 */
#define W25Q128_SECTOR_COUNT    4096                /**< 扇区数量 */
#define W25Q128_PAGE_COUNT      65536               /**< 页数量 */
#define W25Q128_BLOCK_COUNT     256                 /**< 块数量 */

/* 芯片 ID */
#define W25Q128_JEDEC_ID        0xEF4018            /**< W25Q128 JEDEC ID */
#define W25Q64_JEDEC_ID         0xEF4017            /**< W25Q64 JEDEC ID */
#define W25Q32_JEDEC_ID         0xEF4016            /**< W25Q32 JEDEC ID */

/* SPI Flash 命令 */
#define W25X_WriteEnable        0x06    /**< 写使能 */
#define W25X_WriteDisable       0x04    /**< 写禁止 */
#define W25X_ReadStatusReg1     0x05    /**< 读状态寄存器 1 */
#define W25X_ReadStatusReg2     0x35    /**< 读状态寄存器 2 */
#define W25X_ReadStatusReg3     0x15    /**< 读状态寄存器 3 */
#define W25X_WriteStatusReg1    0x01    /**< 写状态寄存器 1 */
#define W25X_WriteStatusReg2    0x31    /**< 写状态寄存器 2 */
#define W25X_WriteStatusReg3    0x11    /**< 写状态寄存器 3 */
#define W25X_ReadData           0x03    /**< 读数据 */
#define W25X_FastReadData       0x0B    /**< 快速读数据 */
#define W25X_FastReadDual       0x3B    /**< 双线快速读 */
#define W25X_PageProgram        0x02    /**< 页编程 */
#define W25X_SectorErase        0x20    /**< 扇区擦除 (4KB) */
#define W25X_BlockErase32       0x52    /**< 块擦除 (32KB) */
#define W25X_BlockErase64       0xD8    /**< 块擦除 (64KB) */
#define W25X_ChipErase          0xC7    /**< 全片擦除 */
#define W25X_PowerDown          0xB9    /**< 掉电模式 */
#define W25X_ReleasePowerDown   0xAB    /**< 释放掉电 */
#define W25X_DeviceID           0xAB    /**< 读设备 ID */
#define W25X_ManufactDeviceID   0x90    /**< 读制造商/设备 ID */
#define W25X_JedecDeviceID      0x9F    /**< 读 JEDEC ID */
#define W25X_EnableReset        0x66    /**< 使能复位 */
#define W25X_Reset              0x99    /**< 复位 */

/* 状态寄存器位 */
#define W25X_SR1_BUSY           0x01    /**< 忙标志 */
#define W25X_SR1_WEL            0x02    /**< 写使能锁存 */

/* 片选控制 */
#define W25QXX_CS_LOW()         HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH()        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Flash 芯片类型
 */
typedef enum {
    W25Q16 = 0,
    W25Q32,
    W25Q64,
    W25Q128,
    W25Q256,
    W25Q_UNKNOWN
} W25QXX_Type_t;

/**
 * @brief Flash 状态
 */
typedef enum {
    W25QXX_OK = 0,
    W25QXX_ERROR,
    W25QXX_BUSY,
    W25QXX_TIMEOUT
} W25QXX_Status_t;

/**
 * @brief Flash 数据结构体
 */
typedef struct {
    W25QXX_Type_t type;         /**< 芯片类型 */
    uint32_t jedec_id;          /**< JEDEC ID */
    uint32_t capacity;          /**< 容量 (字节) */
    uint32_t sector_count;      /**< 扇区数量 */
    uint32_t page_count;        /**< 页数量 */
    uint32_t block_count;       /**< 块数量 */
    bool initialized;           /**< 初始化标志 */
    
    /* 黑匣子日志管理 */
    uint32_t log_start_addr;    /**< 日志起始地址 */
    uint32_t log_end_addr;      /**< 日志结束地址 */
    uint32_t log_write_ptr;     /**< 当前写指针 */
    uint32_t log_read_ptr;      /**< 当前读指针 */
    bool log_wrapped;           /**< 日志是否已回绕 */
} W25QXX_Data_t;

/* Exported variables --------------------------------------------------------*/
extern W25QXX_Data_t g_w25qxx_data;     /**< 全局 Flash 数据 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 W25QXX
 * @return bool true=成功, false=失败
 */
bool W25QXX_Init(void);

/**
 * @brief 读取 JEDEC ID
 * @return uint32_t JEDEC ID
 */
uint32_t W25QXX_ReadJEDECID(void);

/**
 * @brief 读取设备 ID
 * @return uint16_t 设备 ID
 */
uint16_t W25QXX_ReadDeviceID(void);

/**
 * @brief 读取数据
 * @param addr 起始地址
 * @param data 数据缓冲区
 * @param len 长度
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_Read(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief 快速读取数据
 * @param addr 起始地址
 * @param data 数据缓冲区
 * @param len 长度
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_FastRead(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief 使用 DMA 读取数据
 * @param addr 起始地址
 * @param data 数据缓冲区
 * @param len 长度
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_ReadDMA(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief 写入数据 (自动处理跨页)
 * @param addr 起始地址
 * @param data 数据
 * @param len 长度
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_Write(uint32_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief 页编程 (单页，最多 256 字节)
 * @param addr 页起始地址
 * @param data 数据
 * @param len 长度 (最大 256)
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief 扇区擦除 (4KB)
 * @param addr 扇区地址
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_EraseSector(uint32_t addr);

/**
 * @brief 块擦除 (64KB)
 * @param addr 块地址
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_EraseBlock(uint32_t addr);

/**
 * @brief 全片擦除
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_EraseChip(void);

/**
 * @brief 等待忙标志清除
 * @param timeout 超时时间 (ms)
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_WaitBusy(uint32_t timeout);

/**
 * @brief 写使能
 */
void W25QXX_WriteEnable(void);

/**
 * @brief 写禁止
 */
void W25QXX_WriteDisable(void);

/**
 * @brief 读取状态寄存器 1
 * @return uint8_t 状态寄存器值
 */
uint8_t W25QXX_ReadStatusReg1(void);

/**
 * @brief 进入掉电模式
 */
void W25QXX_PowerDown(void);

/**
 * @brief 唤醒
 */
void W25QXX_WakeUp(void);

/* 黑匣子日志功能 */

/**
 * @brief 初始化日志系统
 * @param start_addr 日志区起始地址
 * @param size 日志区大小
 */
void W25QXX_LogInit(uint32_t start_addr, uint32_t size);

/**
 * @brief 写入日志数据
 * @param data 数据
 * @param len 长度
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_LogWrite(const uint8_t *data, uint32_t len);

/**
 * @brief 读取日志数据
 * @param data 数据缓冲区
 * @param len 长度
 * @return uint32_t 实际读取长度
 */
uint32_t W25QXX_LogRead(uint8_t *data, uint32_t len);

/**
 * @brief 清除所有日志
 * @return W25QXX_Status_t 状态
 */
W25QXX_Status_t W25QXX_LogClear(void);

/**
 * @brief 获取日志使用量
 * @return uint32_t 已使用字节数
 */
uint32_t W25QXX_LogGetUsed(void);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */