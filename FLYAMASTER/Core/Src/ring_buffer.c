/**
 * @file ring_buffer.c
 * @brief 环形缓冲区模块实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "ring_buffer.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>

/* Private defines -----------------------------------------------------------*/
#define ENTER_CRITICAL()    taskENTER_CRITICAL()
#define EXIT_CRITICAL()     taskEXIT_CRITICAL()

/* Private variables ---------------------------------------------------------*/
RingBuffer_t g_log_buffer;                          /**< 全局日志缓冲区 */
static uint8_t s_dma_tx_buffer[LOG_MAX_MSG_LEN];    /**< DMA 发送临时缓冲区 */
static volatile uint16_t s_dma_tx_len = 0;          /**< 当前 DMA 发送长度 */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化环形缓冲区
 */
void RingBuffer_Init(RingBuffer_t *rb)
{
    if (rb == NULL) return;
    
    ENTER_CRITICAL();
    rb->head = 0;
    rb->tail = 0;
    rb->dma_busy = false;
    memset(rb->buffer, 0, LOG_BUFFER_SIZE);
    EXIT_CRITICAL();
}

/**
 * @brief 写入数据到缓冲区
 * @note 此函数设计为快速执行 (< 10us)
 */
RingBuffer_Status_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
    if (rb == NULL || data == NULL || len == 0) {
        return RING_BUFFER_ERROR;
    }
    
    uint16_t free_space = RingBuffer_FreeSpace(rb);
    if (len > free_space) {
        return RING_BUFFER_FULL;
    }
    
    ENTER_CRITICAL();
    
    uint16_t head = rb->head;
    
    /* 计算到缓冲区末尾的空间 */
    uint16_t space_to_end = LOG_BUFFER_SIZE - head;
    
    if (len <= space_to_end) {
        /* 数据可以连续写入 */
        memcpy(&rb->buffer[head], data, len);
    } else {
        /* 数据需要分两段写入 (环绕) */
        memcpy(&rb->buffer[head], data, space_to_end);
        memcpy(&rb->buffer[0], data + space_to_end, len - space_to_end);
    }
    
    /* 更新 head 指针 */
    rb->head = (head + len) % LOG_BUFFER_SIZE;
    
    EXIT_CRITICAL();
    
    return RING_BUFFER_OK;
}

/**
 * @brief 从缓冲区读取数据
 */
uint16_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint16_t max_len)
{
    if (rb == NULL || data == NULL || max_len == 0) {
        return 0;
    }
    
    uint16_t available = RingBuffer_Available(rb);
    if (available == 0) {
        return 0;
    }
    
    uint16_t read_len = (available < max_len) ? available : max_len;
    
    ENTER_CRITICAL();
    
    uint16_t tail = rb->tail;
    uint16_t space_to_end = LOG_BUFFER_SIZE - tail;
    
    if (read_len <= space_to_end) {
        memcpy(data, &rb->buffer[tail], read_len);
    } else {
        memcpy(data, &rb->buffer[tail], space_to_end);
        memcpy(data + space_to_end, &rb->buffer[0], read_len - space_to_end);
    }
    
    rb->tail = (tail + read_len) % LOG_BUFFER_SIZE;
    
    EXIT_CRITICAL();
    
    return read_len;
}

/**
 * @brief 获取缓冲区可用数据长度
 */
uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
    if (rb == NULL) return 0;
    
    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return LOG_BUFFER_SIZE - tail + head;
    }
}

/**
 * @brief 获取缓冲区剩余空间
 */
uint16_t RingBuffer_FreeSpace(RingBuffer_t *rb)
{
    if (rb == NULL) return 0;
    
    /* 保留 1 字节用于区分满/空状态 */
    return LOG_BUFFER_SIZE - RingBuffer_Available(rb) - 1;
}

/**
 * @brief 检查缓冲区是否为空
 */
bool RingBuffer_IsEmpty(RingBuffer_t *rb)
{
    if (rb == NULL) return true;
    return (rb->head == rb->tail);
}

/**
 * @brief 检查缓冲区是否已满
 */
bool RingBuffer_IsFull(RingBuffer_t *rb)
{
    if (rb == NULL) return true;
    return (RingBuffer_FreeSpace(rb) == 0);
}

/**
 * @brief 清空缓冲区
 */
void RingBuffer_Clear(RingBuffer_t *rb)
{
    if (rb == NULL) return;
    
    ENTER_CRITICAL();
    rb->head = 0;
    rb->tail = 0;
    EXIT_CRITICAL();
}

/**
 * @brief 获取连续可读数据块 (用于 DMA 零拷贝发送)
 */
uint16_t RingBuffer_GetContiguousReadBlock(RingBuffer_t *rb, uint8_t **data_ptr)
{
    if (rb == NULL || data_ptr == NULL) return 0;
    
    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    
    if (head == tail) {
        *data_ptr = NULL;
        return 0;
    }
    
    *data_ptr = &rb->buffer[tail];
    
    if (head > tail) {
        /* 数据连续 */
        return head - tail;
    } else {
        /* 数据环绕，只返回到末尾的部分 */
        return LOG_BUFFER_SIZE - tail;
    }
}

/**
 * @brief 确认 DMA 发送完成，更新 tail 指针
 */
void RingBuffer_ConfirmRead(RingBuffer_t *rb, uint16_t len)
{
    if (rb == NULL || len == 0) return;
    
    ENTER_CRITICAL();
    rb->tail = (rb->tail + len) % LOG_BUFFER_SIZE;
    EXIT_CRITICAL();
}

/* Log Printf API ------------------------------------------------------------*/

/**
 * @brief 初始化日志系统
 */
void Log_Init(void)
{
    RingBuffer_Init(&g_log_buffer);
}

/**
 * @brief 格式化日志输出
 */
int Log_Printf(const char *format, ...)
{
    if (format == NULL) return -1;
    
    char temp_buffer[LOG_MAX_MSG_LEN];
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(temp_buffer, LOG_MAX_MSG_LEN, format, args);
    va_end(args);
    
    if (len < 0) return -1;
    if (len > LOG_MAX_MSG_LEN - 1) {
        len = LOG_MAX_MSG_LEN - 1;
    }
    
    RingBuffer_Status_t status = RingBuffer_Write(&g_log_buffer, 
                                                   (uint8_t *)temp_buffer, 
                                                   (uint16_t)len);
    
    return (status == RING_BUFFER_OK) ? len : -1;
}

/**
 * @brief 启动 DMA 发送
 * @note 由 log_task 周期性调用
 */
bool Log_StartDMATransmit(void)
{
    /* 检查 DMA 是否忙 */
    if (g_log_buffer.dma_busy) {
        return false;
    }
    
    /* 检查是否有数据 */
    if (RingBuffer_IsEmpty(&g_log_buffer)) {
        return false;
    }
    
    /* 获取连续可读数据块 */
    uint8_t *data_ptr;
    uint16_t len = RingBuffer_GetContiguousReadBlock(&g_log_buffer, &data_ptr);
    
    if (len == 0 || data_ptr == NULL) {
        return false;
    }
    
    /* 限制单次发送长度 */
    if (len > LOG_MAX_MSG_LEN) {
        len = LOG_MAX_MSG_LEN;
    }
    
    /* 复制到 DMA 发送缓冲区 (避免数据被覆盖) */
    memcpy(s_dma_tx_buffer, data_ptr, len);
    s_dma_tx_len = len;
    
    /* 标记 DMA 忙 */
    g_log_buffer.dma_busy = true;
    
    /* 启动 DMA 发送 (USART1) */
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, s_dma_tx_buffer, len);
    
    if (status != HAL_OK) {
        g_log_buffer.dma_busy = false;
        return false;
    }
    
    /* 更新 tail 指针 */
    RingBuffer_ConfirmRead(&g_log_buffer, len);
    
    return true;
}

/**
 * @brief DMA 发送完成回调
 * @note 在 stm32f4xx_it.c 的 HAL_UART_TxCpltCallback 中调用
 */
void Log_DMATransmitComplete(void)
{
    g_log_buffer.dma_busy = false;
    s_dma_tx_len = 0;
}

/* Callback for HAL UART TX Complete -----------------------------------------*/
/* 
 * 注意: 需要在 stm32f4xx_it.c 中添加以下代码:
 * 
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == USART1) {
 *         Log_DMATransmitComplete();
 *     }
 * }
 */