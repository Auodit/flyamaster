#include "protocol.h"
#include <string.h>

// 回调函数指针
Protocol_PID_Callback g_pid_callback = NULL;
Protocol_Arm_Callback g_arm_callback = NULL;

/**
 * @brief 注册 PID 参数设置回调
 */
void Protocol_RegisterPIDCallback(Protocol_PID_Callback cb) {
    g_pid_callback = cb;
}

/**
 * @brief 注册解锁/上锁回调
 */
void Protocol_RegisterArmCallback(Protocol_Arm_Callback cb) {
    g_arm_callback = cb;
}

/**
 * @brief 计算校验和 (简单异或)
 */
static uint8_t Protocol_CalcChecksum(uint8_t *buf, uint16_t len) {
    // Risk #055 修复：添加指针空检查
    if (buf == NULL) return 0;
    
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum ^= buf[i];
    }
    return sum;
}

/**
 * @brief 发送状态数据给上位机
 * 帧格式: [AA] [CMD] [DATA...] [CHECKSUM] [55]
 */
void Protocol_SendStatus(UART_HandleTypeDef *huart, Status_Packet_t *status) {
    // Risk #055 修复：添加指针空检查
    if (huart == NULL || status == NULL) return;
    
    uint8_t tx_buf[32];
    uint8_t idx = 0;

    tx_buf[idx++] = FRAME_HEADER;
    tx_buf[idx++] = CMD_STATUS;
    
    memcpy(&tx_buf[idx], &status->roll, 4); idx += 4;
    memcpy(&tx_buf[idx], &status->pitch, 4); idx += 4;
    memcpy(&tx_buf[idx], &status->yaw, 4); idx += 4;
    memcpy(&tx_buf[idx], &status->voltage, 4); idx += 4;

    // 计算校验和 (从 CMD 开始到数据结束)
    tx_buf[idx] = Protocol_CalcChecksum(&tx_buf[1], idx - 1);
    idx++;
    
    tx_buf[idx++] = FRAME_TAIL;

    // Risk #056 修复：虽然是DMA发送，但检查返回值可提高可靠性
    // 注：DMA发送失败时数据会丢失，但不会阻塞系统
    if (HAL_UART_Transmit_DMA(huart, tx_buf, idx) != HAL_OK) {
        // DMA发送失败，数据丢失（非阻塞设计，不重试）
    }
}

/**
 * @brief 解析上位机指令
 * 帧格式: [AA] [CMD] [DATA...] [CHECKSUM] [55]
 */
void Protocol_Parse(uint8_t *buf, uint16_t len) {
    // Risk #055 修复：添加指针空检查
    if (buf == NULL) return;
    
    PID_Packet_t pid_pkt;
    
    // 遍历查找帧头
    for (uint16_t i = 0; i < len - 3; i++) {
        // 查找帧头
        if (buf[i] != FRAME_HEADER) {
            continue;
        }
        
        uint8_t cmd = buf[i + 1];
        uint16_t data_start = i + 2;
        
        switch (cmd) {
            case CMD_PID_SET:
                // PID 设置帧: [AA][10][PID_ID][Kp(4)][Ki(4)][Kd(4)][CHK][55]
                if (i + 17 < len && buf[i + 16] == FRAME_TAIL) {
                    // 校验
                    uint8_t chk = Protocol_CalcChecksum(&buf[i + 1], 14);
                    if (chk == buf[i + 15]) {
                        pid_pkt.pid_id = buf[data_start];
                        memcpy(&pid_pkt.kp, &buf[data_start + 1], 4);
                        memcpy(&pid_pkt.ki, &buf[data_start + 5], 4);
                        memcpy(&pid_pkt.kd, &buf[data_start + 9], 4);
                        
                        if (g_pid_callback) {
                            g_pid_callback(&pid_pkt);
                            
                            // ========== 标记参数已修改，需要保存到 Flash ==========
                            extern void FlashParams_MarkDirty(void);
                            FlashParams_MarkDirty();
                        }
                    }
                }
                break;
                
            case CMD_ARM:
                // 解锁帧: [AA][40][CHK][55]
                if (i + 4 <= len && buf[i + 3] == FRAME_TAIL) {
                    if (g_arm_callback) {
                        g_arm_callback(1);  // 1 = 解锁
                    }
                }
                break;
                
            case CMD_DISARM:
                // 上锁帧: [AA][41][CHK][55]
                if (i + 4 <= len && buf[i + 3] == FRAME_TAIL) {
                    if (g_arm_callback) {
                        g_arm_callback(0);  // 0 = 上锁
                    }
                }
                break;
                
            case CMD_MOTOR_TEST:
                // 电机测试帧: [AA][20][MOTOR_ID][PWM_H][PWM_L][CHK][55]
                // TODO: 实现电机单独测试
                break;
                
            case CMD_CALIBRATE:
                // 校准帧: [AA][30][CAL_TYPE][CHK][55]
                // TODO: 实现传感器校准
                break;
                
            default:
                break;
        }
    }
}
