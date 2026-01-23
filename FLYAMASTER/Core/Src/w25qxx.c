/**
 * @file w25qxx.c
 * @brief W25Q128 SPI Flash 驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "w25qxx.h"
#include "spi.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define W25QXX_TIMEOUT          1000    /**< SPI 超时 (ms) */
#define W25QXX_SECTOR_ERASE_TIME 400    /**< 扇区擦除超时 (ms) */
#define W25QXX_BLOCK_ERASE_TIME  2000   /**< 块擦除超时 (ms) */
#define W25QXX_CHIP_ERASE_TIME   200000 /**< 全片擦除超时 (ms) */

/* Private variables ---------------------------------------------------------*/
W25QXX_Data_t g_w25qxx_data = {
    .type = W25Q_UNKNOWN,
    .jedec_id = 0,
    .capacity = 0,
    .sector_count = 0,
    .page_count = 0,
    .block_count = 0,
    .initialized = false,
    .log_start_addr = 0,
    .log_end_addr = 0,
    .log_write_ptr = 0,
    .log_read_ptr = 0,
    .log_wrapped = false
};

/* 用于 SPI 通信的临时缓冲区 */
static uint8_t spi_tx_buf[4];
/* static uint8_t spi_rx_buf[4]; */  /* 未使用，注释掉避免警告 */

/* Private function prototypes -----------------------------------------------*/
static uint8_t W25QXX_SPI_ReadWrite(uint8_t data);
static void W25QXX_SendAddress(uint32_t addr);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief SPI 读写一个字节
 */
static uint8_t W25QXX_SPI_ReadWrite(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, W25QXX_TIMEOUT);
    return rx;
}

/**
 * @brief 发送 24 位地址
 */
static void W25QXX_SendAddress(uint32_t addr)
{
    W25QXX_SPI_ReadWrite((addr >> 16) & 0xFF);
    W25QXX_SPI_ReadWrite((addr >> 8) & 0xFF);
    W25QXX_SPI_ReadWrite(addr & 0xFF);
}

/**
 * @brief 读取 JEDEC ID
 */
uint32_t W25QXX_ReadJEDECID(void)
{
    uint32_t id = 0;
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_JedecDeviceID);
    id = W25QXX_SPI_ReadWrite(0xFF) << 16;
    id |= W25QXX_SPI_ReadWrite(0xFF) << 8;
    id |= W25QXX_SPI_ReadWrite(0xFF);
    
    W25QXX_CS_HIGH();
    
    return id;
}

/**
 * @brief 读取设备 ID
 */
uint16_t W25QXX_ReadDeviceID(void)
{
    uint16_t id = 0;
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_ManufactDeviceID);
    W25QXX_SPI_ReadWrite(0x00);
    W25QXX_SPI_ReadWrite(0x00);
    W25QXX_SPI_ReadWrite(0x00);
    id = W25QXX_SPI_ReadWrite(0xFF) << 8;
    id |= W25QXX_SPI_ReadWrite(0xFF);
    
    W25QXX_CS_HIGH();
    
    return id;
}

/**
 * @brief 读取状态寄存器 1
 */
uint8_t W25QXX_ReadStatusReg1(void)
{
    uint8_t status;
    
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_ReadStatusReg1);
    status = W25QXX_SPI_ReadWrite(0xFF);
    W25QXX_CS_HIGH();
    
    return status;
}

/**
 * @brief 等待忙标志清除
 */
W25QXX_Status_t W25QXX_WaitBusy(uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    
    while (W25QXX_ReadStatusReg1() & W25X_SR1_BUSY) {
        if ((HAL_GetTick() - start) > timeout) {
            return W25QXX_TIMEOUT;
        }
    }
    
    return W25QXX_OK;
}

/**
 * @brief 写使能
 */
void W25QXX_WriteEnable(void)
{
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_WriteEnable);
    W25QXX_CS_HIGH();
}

/**
 * @brief 写禁止
 */
void W25QXX_WriteDisable(void)
{
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_WriteDisable);
    W25QXX_CS_HIGH();
}

/**
 * @brief 进入掉电模式
 */
void W25QXX_PowerDown(void)
{
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_PowerDown);
    W25QXX_CS_HIGH();
    HAL_Delay(1);  /* tDP = 3us */
}

/**
 * @brief 唤醒
 */
void W25QXX_WakeUp(void)
{
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_ReleasePowerDown);
    W25QXX_CS_HIGH();
    HAL_Delay(1);  /* tRES1 = 3us */
}

/**
 * @brief 初始化 W25QXX
 */
bool W25QXX_Init(void)
{
    uint32_t jedec_id;
    
    /* 清空数据结构 */
    memset(&g_w25qxx_data, 0, sizeof(g_w25qxx_data));
    
    /* 确保 CS 为高 */
    W25QXX_CS_HIGH();
    HAL_Delay(10);
    
    /* 唤醒芯片 */
    W25QXX_WakeUp();
    
    /* 读取 JEDEC ID */
    jedec_id = W25QXX_ReadJEDECID();
    g_w25qxx_data.jedec_id = jedec_id;
    
    /* 识别芯片类型 */
    switch (jedec_id) {
        case W25Q128_JEDEC_ID:
            g_w25qxx_data.type = W25Q128;
            g_w25qxx_data.capacity = 16 * 1024 * 1024;
            g_w25qxx_data.sector_count = 4096;
            g_w25qxx_data.page_count = 65536;
            g_w25qxx_data.block_count = 256;
            break;
            
        case W25Q64_JEDEC_ID:
            g_w25qxx_data.type = W25Q64;
            g_w25qxx_data.capacity = 8 * 1024 * 1024;
            g_w25qxx_data.sector_count = 2048;
            g_w25qxx_data.page_count = 32768;
            g_w25qxx_data.block_count = 128;
            break;
            
        case W25Q32_JEDEC_ID:
            g_w25qxx_data.type = W25Q32;
            g_w25qxx_data.capacity = 4 * 1024 * 1024;
            g_w25qxx_data.sector_count = 1024;
            g_w25qxx_data.page_count = 16384;
            g_w25qxx_data.block_count = 64;
            break;
            
        default:
            g_w25qxx_data.type = W25Q_UNKNOWN;
            return false;
    }
    
    g_w25qxx_data.initialized = true;
    
    return true;
}

/**
 * @brief 读取数据
 */
W25QXX_Status_t W25QXX_Read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!g_w25qxx_data.initialized || data == NULL || len == 0) {
        return W25QXX_ERROR;
    }
    
    if (addr + len > g_w25qxx_data.capacity) {
        return W25QXX_ERROR;
    }
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_ReadData);
    W25QXX_SendAddress(addr);
    
    for (uint32_t i = 0; i < len; i++) {
        data[i] = W25QXX_SPI_ReadWrite(0xFF);
    }
    
    W25QXX_CS_HIGH();
    
    return W25QXX_OK;
}

/**
 * @brief 快速读取数据
 */
W25QXX_Status_t W25QXX_FastRead(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!g_w25qxx_data.initialized || data == NULL || len == 0) {
        return W25QXX_ERROR;
    }
    
    if (addr + len > g_w25qxx_data.capacity) {
        return W25QXX_ERROR;
    }
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_FastReadData);
    W25QXX_SendAddress(addr);
    W25QXX_SPI_ReadWrite(0xFF);  /* Dummy byte */
    
    for (uint32_t i = 0; i < len; i++) {
        data[i] = W25QXX_SPI_ReadWrite(0xFF);
    }
    
    W25QXX_CS_HIGH();
    
    return W25QXX_OK;
}

/**
 * @brief 使用 DMA 读取数据
 */
W25QXX_Status_t W25QXX_ReadDMA(uint32_t addr, uint8_t *data, uint32_t len)
{
    HAL_StatusTypeDef status;
    
    if (!g_w25qxx_data.initialized || data == NULL || len == 0) {
        return W25QXX_ERROR;
    }
    
    if (addr + len > g_w25qxx_data.capacity) {
        return W25QXX_ERROR;
    }
    
    W25QXX_CS_LOW();
    
    /* 发送命令和地址 */
    spi_tx_buf[0] = W25X_FastReadData;
    spi_tx_buf[1] = (addr >> 16) & 0xFF;
    spi_tx_buf[2] = (addr >> 8) & 0xFF;
    spi_tx_buf[3] = addr & 0xFF;
    
    HAL_SPI_Transmit(&hspi1, spi_tx_buf, 4, W25QXX_TIMEOUT);
    
    /* Dummy byte */
    uint8_t dummy = 0xFF;
    HAL_SPI_Transmit(&hspi1, &dummy, 1, W25QXX_TIMEOUT);
    
    /* DMA 接收数据 */
    status = HAL_SPI_Receive_DMA(&hspi1, data, len);
    
    if (status != HAL_OK) {
        W25QXX_CS_HIGH();
        return W25QXX_ERROR;
    }
    
    /* 注意: CS 需要在 DMA 完成回调中拉高 */
    
    return W25QXX_OK;
}

/**
 * @brief 页编程
 */
W25QXX_Status_t W25QXX_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (!g_w25qxx_data.initialized || data == NULL || len == 0) {
        return W25QXX_ERROR;
    }
    
    if (len > W25Q128_PAGE_SIZE) {
        len = W25Q128_PAGE_SIZE;
    }
    
    /* 等待空闲 */
    {
        W25QXX_Status_t status = W25QXX_WaitBusy(W25QXX_TIMEOUT);
        if (status != W25QXX_OK) {
            return status;
        }
    }
    
    /* 写使能 */
    W25QXX_WriteEnable();
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_PageProgram);
    W25QXX_SendAddress(addr);
    
    {
        uint32_t i;
        for (i = 0; i < len; i++) {
            W25QXX_SPI_ReadWrite(data[i]);
        }
    }
    
    W25QXX_CS_HIGH();
    
    /* 等待编程完成 */
    return W25QXX_WaitBusy(W25QXX_TIMEOUT);
}

/**
 * @brief 写入数据 (自动处理跨页)
 */
W25QXX_Status_t W25QXX_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t page_remain;
    uint32_t offset;
    W25QXX_Status_t status;
    
    if (!g_w25qxx_data.initialized || data == NULL || len == 0) {
        return W25QXX_ERROR;
    }
    
    if (addr + len > g_w25qxx_data.capacity) {
        return W25QXX_ERROR;
    }
    
    offset = 0;
    
    while (len > 0) {
        /* 计算当前页剩余空间 */
        page_remain = W25Q128_PAGE_SIZE - (addr % W25Q128_PAGE_SIZE);
        
        if (len <= page_remain) {
            page_remain = len;
        }
        
        /* 页编程 */
        status = W25QXX_PageProgram(addr, data + offset, page_remain);
        if (status != W25QXX_OK) {
            return status;
        }
        
        addr += page_remain;
        offset += page_remain;
        len -= page_remain;
    }
    
    return W25QXX_OK;
}

/**
 * @brief 扇区擦除 (4KB)
 */
W25QXX_Status_t W25QXX_EraseSector(uint32_t addr)
{
    if (!g_w25qxx_data.initialized) {
        return W25QXX_ERROR;
    }
    
    /* 对齐到扇区边界 */
    addr = addr & ~(W25Q128_SECTOR_SIZE - 1);
    
    /* 等待空闲 */
    {
        W25QXX_Status_t status = W25QXX_WaitBusy(W25QXX_TIMEOUT);
        if (status != W25QXX_OK) {
            return status;
        }
    }
    
    /* 写使能 */
    W25QXX_WriteEnable();
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_SectorErase);
    W25QXX_SendAddress(addr);
    
    W25QXX_CS_HIGH();
    
    /* 等待擦除完成 */
    return W25QXX_WaitBusy(W25QXX_SECTOR_ERASE_TIME);
}

/**
 * @brief 块擦除 (64KB)
 */
W25QXX_Status_t W25QXX_EraseBlock(uint32_t addr)
{
    if (!g_w25qxx_data.initialized) {
        return W25QXX_ERROR;
    }
    
    /* 对齐到块边界 */
    addr = addr & ~(W25Q128_BLOCK_SIZE - 1);
    
    /* 等待空闲 */
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }
    
    /* 写使能 */
    W25QXX_WriteEnable();
    
    W25QXX_CS_LOW();
    
    W25QXX_SPI_ReadWrite(W25X_BlockErase64);
    W25QXX_SendAddress(addr);
    
    W25QXX_CS_HIGH();
    
    /* 等待擦除完成 */
    return W25QXX_WaitBusy(W25QXX_BLOCK_ERASE_TIME);
}

/**
 * @brief 全片擦除
 */
W25QXX_Status_t W25QXX_EraseChip(void)
{
    if (!g_w25qxx_data.initialized) {
        return W25QXX_ERROR;
    }
    
    /* 等待空闲 */
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT) != W25QXX_OK) {
        return W25QXX_TIMEOUT;
    }
    
    /* 写使能 */
    W25QXX_WriteEnable();
    
    W25QXX_CS_LOW();
    W25QXX_SPI_ReadWrite(W25X_ChipErase);
    W25QXX_CS_HIGH();
    
    /* 等待擦除完成 */
    return W25QXX_WaitBusy(W25QXX_CHIP_ERASE_TIME);
}

/* 黑匣子日志功能 */

/**
 * @brief 初始化日志系统
 */
void W25QXX_LogInit(uint32_t start_addr, uint32_t size)
{
    /* 对齐到扇区边界 */
    g_w25qxx_data.log_start_addr = start_addr & ~(W25Q128_SECTOR_SIZE - 1);
    g_w25qxx_data.log_end_addr = g_w25qxx_data.log_start_addr + size;
    
    /* 确保不超过芯片容量 */
    if (g_w25qxx_data.log_end_addr > g_w25qxx_data.capacity) {
        g_w25qxx_data.log_end_addr = g_w25qxx_data.capacity;
    }
    
    g_w25qxx_data.log_write_ptr = g_w25qxx_data.log_start_addr;
    g_w25qxx_data.log_read_ptr = g_w25qxx_data.log_start_addr;
    g_w25qxx_data.log_wrapped = false;
}

/**
 * @brief 写入日志数据
 */
W25QXX_Status_t W25QXX_LogWrite(const uint8_t *data, uint32_t len)
{
    W25QXX_Status_t status;
    uint32_t write_addr = g_w25qxx_data.log_write_ptr;
    uint32_t remaining = len;
    uint32_t offset = 0;
    
    while (remaining > 0) {
        /* 检查是否需要擦除新扇区 */
        if ((write_addr % W25Q128_SECTOR_SIZE) == 0) {
            status = W25QXX_EraseSector(write_addr);
            if (status != W25QXX_OK) {
                return status;
            }
        }
        
        /* 计算本次写入长度 */
        uint32_t sector_remain = W25Q128_SECTOR_SIZE - (write_addr % W25Q128_SECTOR_SIZE);
        uint32_t write_len = (remaining < sector_remain) ? remaining : sector_remain;
        
        /* 写入数据 */
        status = W25QXX_Write(write_addr, data + offset, write_len);
        if (status != W25QXX_OK) {
            return status;
        }
        
        write_addr += write_len;
        offset += write_len;
        remaining -= write_len;
        
        /* 检查是否回绕 */
        if (write_addr >= g_w25qxx_data.log_end_addr) {
            write_addr = g_w25qxx_data.log_start_addr;
            g_w25qxx_data.log_wrapped = true;
        }
    }
    
    g_w25qxx_data.log_write_ptr = write_addr;
    
    return W25QXX_OK;
}

/**
 * @brief 读取日志数据
 */
uint32_t W25QXX_LogRead(uint8_t *data, uint32_t len)
{
    uint32_t available = W25QXX_LogGetUsed();
    uint32_t read_len = (len < available) ? len : available;
    
    if (read_len == 0) {
        return 0;
    }
    
    uint32_t read_addr = g_w25qxx_data.log_read_ptr;
    uint32_t remaining = read_len;
    uint32_t offset = 0;
    
    while (remaining > 0) {
        uint32_t chunk = remaining;
        
        /* 检查是否需要回绕 */
        if (read_addr + chunk > g_w25qxx_data.log_end_addr) {
            chunk = g_w25qxx_data.log_end_addr - read_addr;
        }
        
        W25QXX_Read(read_addr, data + offset, chunk);
        
        read_addr += chunk;
        offset += chunk;
        remaining -= chunk;
        
        if (read_addr >= g_w25qxx_data.log_end_addr) {
            read_addr = g_w25qxx_data.log_start_addr;
        }
    }
    
    g_w25qxx_data.log_read_ptr = read_addr;
    
    return read_len;
}

/**
 * @brief 清除所有日志
 */
W25QXX_Status_t W25QXX_LogClear(void)
{
    W25QXX_Status_t status;
    uint32_t addr = g_w25qxx_data.log_start_addr;
    
    while (addr < g_w25qxx_data.log_end_addr) {
        status = W25QXX_EraseSector(addr);
        if (status != W25QXX_OK) {
            return status;
        }
        addr += W25Q128_SECTOR_SIZE;
    }
    
    g_w25qxx_data.log_write_ptr = g_w25qxx_data.log_start_addr;
    g_w25qxx_data.log_read_ptr = g_w25qxx_data.log_start_addr;
    g_w25qxx_data.log_wrapped = false;
    
    return W25QXX_OK;
}

/**
 * @brief 获取日志使用量
 */
uint32_t W25QXX_LogGetUsed(void)
{
    if (g_w25qxx_data.log_wrapped) {
        return g_w25qxx_data.log_end_addr - g_w25qxx_data.log_start_addr;
    } else {
        return g_w25qxx_data.log_write_ptr - g_w25qxx_data.log_start_addr;
    }
}
