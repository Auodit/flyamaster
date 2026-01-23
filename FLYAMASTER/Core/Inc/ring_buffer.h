/**
 * @file ring_buffer.h
 * @brief 环形缓冲区模块 - 用于异步日志系统
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 实现一个线程安全的环形缓冲区，用于：
 * 1. 异步日志发送 (USART1 DMA)
 * 2. 解耦高优先级任务与耗时的 sprintf 操作
 * 
 * 设计原则：
 * - 写入操作 < 10us (仅 memcpy)
 * - 支持 DMA 发送
 * - 线程安全 (使用临界区)
 */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported defines ----------------------------------------------------------*/
#define LOG_BUFFER_SIZE     2048    /**< 日志缓冲区大小 (字节) */
#define LOG_MAX_MSG_LEN     256     /**< 单条日志最大长度 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 环形缓冲区结构体
 */
typedef struct {
    uint8_t buffer[LOG_BUFFER_SIZE];    /**< 数据缓冲区 */
    volatile uint16_t head;              /**< 写入指针 */
    volatile uint16_t tail;              /**< 读取指针 */
    volatile bool dma_busy;              /**< DMA 发送忙标志 */
} RingBuffer_t;

/**
 * @brief 缓冲区状态枚举
 */
typedef enum {
    RING_BUFFER_OK = 0,         /**< 操作成功 */
    RING_BUFFER_FULL,           /**< 缓冲区满 */
    RING_BUFFER_EMPTY,          /**< 缓冲区空 */
    RING_BUFFER_ERROR           /**< 操作错误 */
} RingBuffer_Status_t;

/* Exported variables --------------------------------------------------------*/
extern RingBuffer_t g_log_buffer;   /**< 全局日志缓冲区 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化环形缓冲区
 * @param rb 缓冲区指针
 */
void RingBuffer_Init(RingBuffer_t *rb);

/**
 * @brief 写入数据到缓冲区
 * @param rb 缓冲区指针
 * @param data 数据指针
 * @param len 数据长度
 * @return RingBuffer_Status_t 操作状态
 * @note 此函数耗时 < 10us
 */
RingBuffer_Status_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t len);

/**
 * @brief 从缓冲区读取数据 (用于 DMA 发送)
 * @param rb 缓冲区指针
 * @param data 输出数据指针
 * @param max_len 最大读取长度
 * @return uint16_t 实际读取长度
 */
uint16_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint16_t max_len);

/**
 * @brief 获取缓冲区可用数据长度
 * @param rb 缓冲区指针
 * @return uint16_t 可用数据长度
 */
uint16_t RingBuffer_Available(RingBuffer_t *rb);

/**
 * @brief 获取缓冲区剩余空间
 * @param rb 缓冲区指针
 * @return uint16_t 剩余空间
 */
uint16_t RingBuffer_FreeSpace(RingBuffer_t *rb);

/**
 * @brief 检查缓冲区是否为空
 * @param rb 缓冲区指针
 * @return bool true=空, false=非空
 */
bool RingBuffer_IsEmpty(RingBuffer_t *rb);

/**
 * @brief 检查缓冲区是否已满
 * @param rb 缓冲区指针
 * @return bool true=满, false=未满
 */
bool RingBuffer_IsFull(RingBuffer_t *rb);

/**
 * @brief 清空缓冲区
 * @param rb 缓冲区指针
 */
void RingBuffer_Clear(RingBuffer_t *rb);

/**
 * @brief 获取连续可读数据块 (用于 DMA 零拷贝发送)
 * @param rb 缓冲区指针
 * @param data_ptr 输出数据指针的指针
 * @return uint16_t 连续可读长度
 */
uint16_t RingBuffer_GetContiguousReadBlock(RingBuffer_t *rb, uint8_t **data_ptr);

/**
 * @brief 确认 DMA 发送完成，更新 tail 指针
 * @param rb 缓冲区指针
 * @param len 已发送长度
 */
void RingBuffer_ConfirmRead(RingBuffer_t *rb, uint16_t len);

/* Log Printf API ------------------------------------------------------------*/

/**
 * @brief 初始化日志系统
 * @note 在 log_task 启动时调用
 */
void Log_Init(void);

/**
 * @brief 格式化日志输出 (类似 printf)
 * @param format 格式字符串
 * @param ... 可变参数
 * @return int 写入的字节数，-1 表示失败
 * @note 此函数会将格式化后的字符串写入 g_log_buffer
 */
int Log_Printf(const char *format, ...);

/**
 * @brief 启动 DMA 发送 (由 log_task 调用)
 * @return bool true=启动成功, false=无数据或DMA忙
 */
bool Log_StartDMATransmit(void);

/**
 * @brief DMA 发送完成回调 (在 HAL_UART_TxCpltCallback 中调用)
 */
void Log_DMATransmitComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H */