/**
  ******************************************************************************
  * @file           : w25qxx.c
  * @brief          : W25Q128 SPI Flash Driver Implementation
  * @author         : Doro (Flight Controller Team)
  * @date           : 2026-01-14
  ******************************************************************************
  * @attention
  * 
  * W25Q128 SPI Flash 驱动实现
  * 
  * 功能:
  *   - 基本读写操作 (Read/Write)
  *   - 扇区/块擦除 (Erase)
  *   - ID 读取 (ReadID)
  *   - 低功耗模式 (PowerDown)
  * 
  * 注意事项:
  *   1. 写入前必须先擦除 (Flash 特性: 只能从 1 改成 0)
  *   2. Sector 擦除时间约 30ms, 需调用 W25QXX_WaitBusy()
  *   3. 禁止在高频任务中调用擦除操作
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "w25qxx.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

// CS 片选控制宏
#define W25QXX_CS_LOW()   HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET)
#define W25QXX_CS_HIGH()  HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *w25qxx_hspi = NULL;  // SPI 句柄

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 W25QXX Flash
  * @param  hspi: SPI 句柄指针
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_Init(SPI_HandleTypeDef *hspi)
{
    uint32_t chip_id;
    
    // 参数检查
    if (hspi == NULL) {
        return 1;
    }
    
    w25qxx_hspi = hspi;
    
    // 片选默认拉高
    W25QXX_CS_HIGH();
    HAL_Delay(10);
    
    // 唤醒芯片 (可能处于掉电模式)
    if (W25QXX_WakeUp() != 0) {
        return 1;
    }
    HAL_Delay(1);
    
    // 读取 ID 验证芯片
    chip_id = W25QXX_ReadID();
    
    if (chip_id == W25Q128_JEDEC_ID) {
        return 0;  // 初始化成功
    } else {
        return 1;  // ID 不匹配
    }
}

/**
  * @brief  读取 JEDEC ID
  * @param  None
  * @retval JEDEC ID (0xEF4018 for W25Q128), 失败返回 0
  */
uint32_t W25QXX_ReadID(void)
{
    uint8_t cmd = W25QXX_CMD_READ_JEDEC_ID;
    uint8_t data[3] = {0};
    uint32_t id;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 0;
    }
    if (HAL_SPI_Receive(w25qxx_hspi, data, 3, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 0;
    }
    W25QXX_CS_HIGH();
    
    id = (data[0] << 16) | (data[1] << 8) | data[2];
    return id;
}

/**
  * @brief  读取数据
  * @param  addr: 起始地址 (0x000000 ~ 0xFFFFFF)
  * @param  buf: 数据缓冲区指针
  * @param  len: 读取长度
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_Read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4];
    
    // 参数检查
    if (buf == NULL || len == 0) {
        return 1;
    }
    
    // 地址范围检查
    if (addr + len > W25QXX_CHIP_SIZE) {
        return 1;
    }
    
    cmd[0] = W25QXX_CMD_READ_DATA;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, cmd, 4, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    if (HAL_SPI_Receive(w25qxx_hspi, buf, len, 1000) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    return 0;
}

/**
  * @brief  页编程 (最多 256 字节)
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区
  * @param  len: 长度 (1~256)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_PageProgram(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4];
    
    // 参数检查
    if (buf == NULL || len == 0 || len > W25QXX_PAGE_SIZE) {
        return 1;
    }
    
    // 写使能并验证WEL位
    if (W25QXX_WriteEnable() != 0) {
        return 1;
    }
    
    cmd[0] = W25QXX_CMD_PAGE_PROGRAM;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, cmd, 4, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    if (HAL_SPI_Transmit(w25qxx_hspi, buf, len, 1000) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    // 等待写入完成
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT_PAGE_PROGRAM) != 0) {
        return 1;
    }
    
    return 0;
}

/**
  * @brief  写入数据 (自动处理页边界)
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区指针
  * @param  len: 写入长度
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_Write(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t page_remain;
    uint16_t write_len;
    uint16_t offset = 0;
    
    // 参数检查
    if (buf == NULL || len == 0) {
        return 1;
    }
    
    // 地址范围检查
    if (addr + len > W25QXX_CHIP_SIZE) {
        return 1;
    }
    
    while (len > 0) {
        // 计算当前页剩余空间
        page_remain = W25QXX_PAGE_SIZE - (addr % W25QXX_PAGE_SIZE);
        
        // 确定本次写入长度
        write_len = (len > page_remain) ? page_remain : len;
        
        // 页编程
        if (W25QXX_PageProgram(addr, buf + offset, write_len) != 0) {
            return 1;
        }
        
        addr += write_len;
        offset += write_len;
        len -= write_len;
    }
    
    return 0;
}

/**
  * @brief  擦除扇区 (4KB)
  * @param  sector_addr: 扇区地址 (必须是 4KB 对齐)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_EraseSector(uint32_t sector_addr)
{
    uint8_t cmd[4];
    
    // 地址 4KB 对齐检查
    if (sector_addr % W25QXX_SECTOR_SIZE != 0) {
        return 1;
    }
    
    // 写使能并验证WEL位
    if (W25QXX_WriteEnable() != 0) {
        return 1;
    }
    
    cmd[0] = W25QXX_CMD_SECTOR_ERASE;
    cmd[1] = (sector_addr >> 16) & 0xFF;
    cmd[2] = (sector_addr >> 8) & 0xFF;
    cmd[3] = sector_addr & 0xFF;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, cmd, 4, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    // 等待擦除完成
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT_SECTOR_ERASE) != 0) {
        return 1;
    }
    
    return 0;
}

/**
  * @brief  擦除块 (64KB)
  * @param  block_addr: 块地址 (必须是 64KB 对齐)
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_EraseBlock(uint32_t block_addr)
{
    uint8_t cmd[4];
    
    // 地址 64KB 对齐检查
    if (block_addr % W25QXX_BLOCK_SIZE_64K != 0) {
        return 1;
    }
    
    // 写使能并验证WEL位
    if (W25QXX_WriteEnable() != 0) {
        return 1;
    }
    
    cmd[0] = W25QXX_CMD_BLOCK_ERASE_64K;
    cmd[1] = (block_addr >> 16) & 0xFF;
    cmd[2] = (block_addr >> 8) & 0xFF;
    cmd[3] = block_addr & 0xFF;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, cmd, 4, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    // 等待擦除完成
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT_BLOCK_ERASE) != 0) {
        return 1;
    }
    
    return 0;
}

/**
  * @brief  擦除整个芯片
  * @param  None
  * @retval 0: 成功, 1: 失败
  * @note   此操作需要很长时间 (~100秒)
  * @warning ⚠️ 严禁在FreeRTOS任务中调用！会导致系统阻塞100秒，触发看门狗复位
  * @warning ⚠️ 仅在系统初始化阶段（任务启动前）或特殊维护模式下使用
  * @warning ⚠️ Risk #037: 超时时间100秒，必须在非实时环境调用
  */
uint8_t W25QXX_EraseChip(void)
{
    uint8_t cmd = W25QXX_CMD_CHIP_ERASE;
    
    // 写使能并验证WEL位
    if (W25QXX_WriteEnable() != 0) {
        return 1;
    }
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    // 等待擦除完成
    if (W25QXX_WaitBusy(W25QXX_TIMEOUT_CHIP_ERASE) != 0) {
        return 1;
    }
    
    return 0;
}

/**
  * @brief  进入掉电模式
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_PowerDown(void)
{
    uint8_t cmd = W25QXX_CMD_POWER_DOWN;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    return 0;
}

/**
  * @brief  唤醒芯片
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WakeUp(void)
{
    uint8_t cmd = W25QXX_CMD_RELEASE_POWER_DOWN;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    return 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  写使能并验证WEL位
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WriteEnable(void)
{
    uint8_t cmd = W25QXX_CMD_WRITE_ENABLE;
    uint8_t status;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    
    // 验证WEL位是否成功设置 (Risk #039修复)
    status = W25QXX_ReadStatusReg(1);
    if ((status & W25QXX_STATUS_WEL) == 0) {
        return 1;  // WEL位未设置，写使能失败
    }
    
    return 0;
}

/**
  * @brief  写禁止
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t W25QXX_WriteDisable(void)
{
    uint8_t cmd = W25QXX_CMD_WRITE_DISABLE;
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 1;
    }
    W25QXX_CS_HIGH();
    return 0;
}

/**
  * @brief  读取状态寄存器
  * @param  reg_num: 寄存器编号 (1 或 2)
  * @retval 状态寄存器值，失败返回 0
  */
uint8_t W25QXX_ReadStatusReg(uint8_t reg_num)
{
    uint8_t cmd, status = 0;
    
    if (reg_num == 1) {
        cmd = W25QXX_CMD_READ_STATUS_REG1;
    } else if (reg_num == 2) {
        cmd = W25QXX_CMD_READ_STATUS_REG2;
    } else {
        return 0;
    }
    
    W25QXX_CS_LOW();
    if (HAL_SPI_Transmit(w25qxx_hspi, &cmd, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 0;
    }
    if (HAL_SPI_Receive(w25qxx_hspi, &status, 1, 100) != HAL_OK) {
        W25QXX_CS_HIGH();
        return 0;
    }
    W25QXX_CS_HIGH();
    
    return status;
}

/**
  * @brief  等待 Flash 空闲
  * @param  timeout_ms: 超时时间 (毫秒)
  * @retval 0: 成功, 1: 超时
  */
uint8_t W25QXX_WaitBusy(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t status;
    
    do {
        status = W25QXX_ReadStatusReg(1);
        
        // 检查 Busy 位
        if ((status & W25QXX_STATUS_BUSY) == 0) {
            return 0;  // 空闲
        }
        
        // 超时检查
        if (HAL_GetTick() - start_tick > timeout_ms) {
            return 1;  // 超时
        }
        
    } while (1);
}
