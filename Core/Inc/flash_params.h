/**
  ******************************************************************************
  * @file           : flash_params.h
  * @brief          : Flash 参数管理模块 (带掉电保护)
  * @author         : Doro (Flight Controller Team)
  * @date           : 2026-01-14
  ******************************************************************************
  * @attention
  * 
  * 功能:
  *   1. PID 参数掉电保存/加载
  *   2. 陀螺仪/加速度计/磁力计/气压计校准数据
  *   3. 飞行配置 (悬停油门、通道映射等)
  *   4. 双缓冲区 + 版本号机制防止掉电损坏
  *   5. CRC16 校验保证数据完整性
  * 
  * 使用方法:
  *   - 开机时调用 FlashParams_Load() 加载参数
  *   - 修改参数后调用 FlashParams_Save() 保存
  *   - 恢复出厂设置调用 FlashParams_RestoreDefaults()
  * 
  * 存储方案:
  *   - W25Q128 SPI Flash
  *   - Block A: 0x000000 (512 字节) - 主数据区
  *   - Block B: 0x000200 (512 字节) - 备份数据区
  *
  ******************************************************************************
  */

#ifndef __FLASH_PARAMS_H
#define __FLASH_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  PID 参数索引枚举
  */
typedef enum {
    PID_ROLL_ANGLE = 0,
    PID_ROLL_RATE,
    PID_PITCH_ANGLE,
    PID_PITCH_RATE,
    PID_YAW_ANGLE,
    PID_YAW_RATE,
    PID_POS_X,
    PID_POS_Y,
    PID_ALT_POS,
    PID_ALT_VEL,
    PID_COUNT
} PID_Index_t;

/**
  * @brief  PID 参数结构体
  */
typedef struct __attribute__((packed)) {
    float kp, ki, kd;
} PID_Params_t;

/**
  * @brief  Flash 参数总结构体
  */
typedef struct __attribute__((packed)) {
    // ========== Header (16 字节) ==========
    uint32_t magic;       // 魔数: 0x464C4159 ("FLAY")
    uint16_t version;     // 版本号 (用于双缓冲区选择)
    uint16_t crc16;       // CRC16 校验值
    uint32_t total_size;  // 数据总大小 (用于校验)
    uint32_t reserved1;   // 保留 (Risk #045修复：添加packed属性防止编译器插入padding)
    
    // ========== PID 参数 (120 字节) ==========
    PID_Params_t pid_roll_angle;    // Roll 角度环
    PID_Params_t pid_roll_rate;     // Roll 角速度环
    PID_Params_t pid_pitch_angle;   // Pitch 角度环
    PID_Params_t pid_pitch_rate;    // Pitch 角速度环
    PID_Params_t pid_yaw_angle;     // Yaw 角度环 (航向锁定)
    PID_Params_t pid_yaw_rate;      // Yaw 角速度环
    PID_Params_t pid_pos_x;         // X 位置环 (定点)
    PID_Params_t pid_pos_y;         // Y 位置环 (定点)
    PID_Params_t pid_alt_pos;       // 高度位置环
    PID_Params_t pid_alt_vel;       // 高度速度环
    
    // ========== 校准数据 (76 字节) ==========
    float gyro_offset[3];       // 陀螺仪零偏 (x, y, z)
    float accel_offset[3];      // 加速度计偏移 (x, y, z)
    float mag_calib[12];        // 磁力计校准矩阵 (硬磁/软磁)
    float pressure_ref;         // 参考气压 (Pa)
    
    // ========== 飞行配置 (24 字节) ==========
    float hover_throttle;       // 悬停油门估计值 (0.0~1.0)
    uint8_t channel_map[16];    // 遥控器通道映射
    uint8_t flight_mode_memory; // 上次飞行模式
    uint8_t reserved2[7];       // 对齐到 4 字节
    
    // ========== 系统信息 (16 字节) ==========
    uint32_t total_flight_time; // 累计飞行时间 (秒)
    uint8_t fw_version[4];      // 固件版本 [major, minor, patch, build]
    uint32_t save_count;        // 保存次数 (用于监控擦写寿命)
    uint32_t reserved3;         // 保留
    
} FlashParams_t;

/* Exported constants --------------------------------------------------------*/

// Flash 地址分配
#define FLASH_PARAMS_ADDR_A    0x000000  // Block A 地址
#define FLASH_PARAMS_ADDR_B    0x000200  // Block B 地址 (512 字节偏移)

// 魔数和版本
#define FLASH_PARAMS_MAGIC     0x464C4159  // "FLAY"
#define FLASH_PARAMS_VERSION   1           // 当前版本号

// 保存延迟保护 (防止频繁擦写)
#define FLASH_SAVE_DELAY_MS    2000        // 修改后延迟 2 秒保存

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern FlashParams_t g_flash_params;  // 全局参数实例
extern uint8_t g_params_dirty;        // 参数脏标志 (需要保存)

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化 Flash 参数模块
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t FlashParams_Init(void);

/**
  * @brief  从 Flash 加载参数
  * @param  None
  * @retval 0: 成功, 1: 失败 (使用默认值)
  */
uint8_t FlashParams_Load(void);

/**
  * @brief  保存参数到 Flash
  * @param  None
  * @retval 0: 成功, 1: 失败
  * @note   使用双缓冲区机制，写入到版本号较小的 Block
  */
uint8_t FlashParams_Save(void);

/**
  * @brief  恢复出厂设置
  * @param  None
  * @retval None
  * @note   此函数会擦除 Flash 并重置为默认参数
  */
void FlashParams_RestoreDefaults(void);

/**
  * @brief  获取指定 PID 参数指针
  * @param  pid_id: PID 索引 (PID_ROLL_ANGLE ~ PID_ALT_VEL)
  * @retval PID 参数指针，失败返回 NULL
  */
PID_Params_t* FlashParams_GetPID(PID_Index_t pid_id);

/**
  * @brief  应用参数到 PID 控制器
  * @param  None
  * @retval None
  * @note   在 FlashParams_Load() 后调用，更新所有 PID 控制器
  */
void FlashParams_ApplyToPID(void);

/**
  * @brief  从 PID 控制器读取当前参数
  * @param  None
  * @retval None
  * @note   在保存前调用，确保参数同步
  */
void FlashParams_ReadFromPID(void);

/**
  * @brief  检查参数是否需要保存
  * @param  None
  * @retval 1: 需要保存, 0: 不需要
  */
uint8_t FlashParams_IsDirty(void);

/**
  * @brief  标记参数已修改 (需要保存)
  * @param  None
  * @retval None
  */
void FlashParams_MarkDirty(void);

/**
  * @brief  周期性保存任务 (在 FreeRTOS 任务中调用)
  * @param  None
  * @retval None
  * @note   检查脏标志，延迟 2 秒后自动保存
  */
void FlashParams_PeriodicSave(void);

/**
  * @brief  计算 CRC16 校验值
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval CRC16 值
  */
uint16_t FlashParams_CalcCRC16(uint8_t *data, uint32_t len);

/**
  * @brief  继续计算 CRC16 校验值（从已有CRC值继续）
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @param  prev_crc: 之前计算的CRC值
  * @retval CRC16 值
  */
uint16_t FlashParams_CalcCRC16_Continue(uint8_t *data, uint32_t len, uint16_t prev_crc);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_PARAMS_H */


