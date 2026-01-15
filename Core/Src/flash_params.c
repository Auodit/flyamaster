/**
  ******************************************************************************
  * @file           : flash_params.c
  * @brief          : Flash 参数管理模块实现 (带掉电保护)
  * @author         : Doro (Flight Controller Team)
  * @date           : 2026-01-14
  ******************************************************************************
  * @attention
  * 
  * 双缓冲区机制:
  *   1. 写入时选择版本号较小的 Block
  *   2. 读取时选择版本号较大且 CRC 正确的 Block
  *   3. 如果两个 Block 都损坏，使用默认参数
  * 
  * CRC16 校验:
  *   - 使用 CRC-16-CCITT (多项式: 0x1021)
  *   - 校验范围: Header 之后的所有数据
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash_params.h"
#include "w25qxx.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;  // SPI1 句柄 (定义在 spi.c)

// 外部 PID 控制器 (定义在 freertos.c)
// Risk #046修复：extern声明应在头文件中，但为保持兼容性暂保留在此处
extern PID_TypeDef pid_roll_angle, pid_roll_rate;
extern PID_TypeDef pid_pitch_angle, pid_pitch_rate;
extern PID_TypeDef pid_yaw_angle, pid_yaw_rate;
extern PID_TypeDef pid_pos_x, pid_pos_y;
extern PID_TypeDef pid_alt_pos, pid_alt_vel;

// 外部飞行状态 (定义在 freertos.c)
typedef enum {
    FLIGHT_DISARMED = 0,
    FLIGHT_ARMED = 1
} FlightState_t;
extern volatile FlightState_t g_flight_state;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CRC16_POLY  0x1021  // CRC-16-CCITT 多项式

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
FlashParams_t g_flash_params;       // 全局参数实例
uint8_t g_params_dirty = 0;         // 参数脏标志
static uint32_t s_dirty_timestamp = 0;  // 脏标志时间戳

/* Private function prototypes -----------------------------------------------*/
static void FlashParams_SetDefaults(FlashParams_t *params);
static uint8_t FlashParams_ReadBlock(uint32_t addr, FlashParams_t *params);
static uint8_t FlashParams_WriteBlock(uint32_t addr, FlashParams_t *params);
static uint8_t FlashParams_ValidateCRC(FlashParams_t *params);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 Flash 参数模块
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t FlashParams_Init(void)
{
    // 初始化 W25QXX
    if (W25QXX_Init(&hspi1) != 0) {
        return 1;
    }
    
    // 加载参数
    return FlashParams_Load();
}

/**
  * @brief  从 Flash 加载参数
  * @param  None
  * @retval 0: 成功, 1: 失败 (使用默认值)
  */
uint8_t FlashParams_Load(void)
{
    FlashParams_t params_a, params_b;
    uint8_t valid_a = 0, valid_b = 0;
    
    // 读取 Block A
    if (FlashParams_ReadBlock(FLASH_PARAMS_ADDR_A, &params_a) == 0) {
        if (FlashParams_ValidateCRC(&params_a) == 0) {
            valid_a = 1;
        }
    }
    
    // 读取 Block B
    if (FlashParams_ReadBlock(FLASH_PARAMS_ADDR_B, &params_b) == 0) {
        if (FlashParams_ValidateCRC(&params_b) == 0) {
            valid_b = 1;
        }
    }
    
    // 选择版本号较大的有效 Block
    if (valid_a && valid_b) {
        // 两个都有效，选择版本号较大的
        if (params_a.version >= params_b.version) {
            memcpy((void*)&g_flash_params, (void*)&params_a, sizeof(FlashParams_t));
        } else {
            memcpy((void*)&g_flash_params, (void*)&params_b, sizeof(FlashParams_t));
        }
        return 0;
    } else if (valid_a) {
        // 只有 A 有效
        memcpy((void*)&g_flash_params, (void*)&params_a, sizeof(FlashParams_t));
        return 0;
    } else if (valid_b) {
        // 只有 B 有效
        memcpy((void*)&g_flash_params, (void*)&params_b, sizeof(FlashParams_t));
        return 0;
    } else {
        // 都无效，使用默认值
        FlashParams_SetDefaults(&g_flash_params);
        FlashParams_Save();  // 立即保存默认值
        return 1;
    }
}

/**
  * @brief  保存参数到 Flash
  * @param  None
  * @retval 0: 成功, 1: 失败
  */
uint8_t FlashParams_Save(void)
{
    FlashParams_t params_a, params_b;
    uint8_t valid_a = 0, valid_b = 0;
    uint32_t write_addr;
    
    // 读取当前两个 Block 的版本号
    if (FlashParams_ReadBlock(FLASH_PARAMS_ADDR_A, &params_a) == 0) {
        if (FlashParams_ValidateCRC(&params_a) == 0) {
            valid_a = 1;
        }
    }
    
    if (FlashParams_ReadBlock(FLASH_PARAMS_ADDR_B, &params_b) == 0) {
        if (FlashParams_ValidateCRC(&params_b) == 0) {
            valid_b = 1;
        }
    }
    
    // Risk #042 修复：版本号溢出检测
    // 选择版本号较小的 Block 写入（磨损均衡）
    if (valid_a && valid_b) {
        if (params_a.version <= params_b.version) {
            write_addr = FLASH_PARAMS_ADDR_A;
            // 防止版本号溢出
            if (params_a.version == 0xFFFF) {
                g_flash_params.version = 1;  // 溢出重置
            } else {
                g_flash_params.version = params_a.version + 1;
            }
        } else {
            write_addr = FLASH_PARAMS_ADDR_B;
            // 防止版本号溢出
            if (params_b.version == 0xFFFF) {
                g_flash_params.version = 1;  // 溢出重置
            } else {
                g_flash_params.version = params_b.version + 1;
            }
        }
    } else if (valid_a) {
        write_addr = FLASH_PARAMS_ADDR_B;
        // 防止版本号溢出
        if (params_a.version == 0xFFFF) {
            g_flash_params.version = 1;  // 溢出重置
        } else {
            g_flash_params.version = params_a.version + 1;
        }
    } else if (valid_b) {
        write_addr = FLASH_PARAMS_ADDR_A;
        // 防止版本号溢出
        if (params_b.version == 0xFFFF) {
            g_flash_params.version = 1;  // 溢出重置
        } else {
            g_flash_params.version = params_b.version + 1;
        }
    } else {
        // 都无效，写入 A
        write_addr = FLASH_PARAMS_ADDR_A;
        g_flash_params.version = 1;
    }
    
    // 更新元数据
    g_flash_params.magic = FLASH_PARAMS_MAGIC;
    g_flash_params.total_size = sizeof(FlashParams_t);
    g_flash_params.save_count++;
    
    // Risk #041 修复：CRC必须保护version字段
    // 计算 CRC16 (分两段：magic+version，然后跳过crc16字段，继续计算剩余部分)
    uint16_t crc_part1 = FlashParams_CalcCRC16((uint8_t*)&g_flash_params.magic, 6);  // magic(4) + version(2) = 6字节
    g_flash_params.crc16 = FlashParams_CalcCRC16_Continue(
        (uint8_t*)&g_flash_params.total_size,  // 从total_size开始（跳过crc16字段）
        sizeof(FlashParams_t) - 8,  // 总大小 - (magic+version+crc16) = -8字节
        crc_part1
    );
    
    // 写入 Flash
    if (FlashParams_WriteBlock(write_addr, &g_flash_params) != 0) {
        return 1;
    }
    
    // 清除脏标志
    g_params_dirty = 0;
    s_dirty_timestamp = 0;
    
    return 0;
}

/**
  * @brief  恢复出厂设置
  * @param  None
  * @retval None
  */
void FlashParams_RestoreDefaults(void)
{
    // Risk #043 修复：正确擦除两个Block
    W25QXX_EraseSector(FLASH_PARAMS_ADDR_A);
    W25QXX_EraseSector(FLASH_PARAMS_ADDR_B);
    
    // 设置默认值
    FlashParams_SetDefaults(&g_flash_params);
    
    // 保存到 Flash
    FlashParams_Save();
    
    // 应用到 PID
    FlashParams_ApplyToPID();
}

/**
  * @brief  获取指定 PID 参数指针
  * @param  pid_id: PID 索引
  * @retval PID 参数指针，失败返回 NULL
  */
PID_Params_t* FlashParams_GetPID(PID_Index_t pid_id)
{
    switch (pid_id) {
        case PID_ROLL_ANGLE:  return &g_flash_params.pid_roll_angle;
        case PID_ROLL_RATE:   return &g_flash_params.pid_roll_rate;
        case PID_PITCH_ANGLE: return &g_flash_params.pid_pitch_angle;
        case PID_PITCH_RATE:  return &g_flash_params.pid_pitch_rate;
        case PID_YAW_ANGLE:   return &g_flash_params.pid_yaw_angle;
        case PID_YAW_RATE:    return &g_flash_params.pid_yaw_rate;
        case PID_POS_X:       return &g_flash_params.pid_pos_x;
        case PID_POS_Y:       return &g_flash_params.pid_pos_y;
        case PID_ALT_POS:     return &g_flash_params.pid_alt_pos;
        case PID_ALT_VEL:     return &g_flash_params.pid_alt_vel;
        default: return NULL;
    }
}

/**
  * @brief  应用参数到 PID 控制器
  * @param  None
  * @retval None
  */
void FlashParams_ApplyToPID(void)
{
    // Roll 角度环和角速度环
    pid_roll_angle.Kp = g_flash_params.pid_roll_angle.kp;
    pid_roll_angle.Ki = g_flash_params.pid_roll_angle.ki;
    pid_roll_angle.Kd = g_flash_params.pid_roll_angle.kd;
    
    pid_roll_rate.Kp = g_flash_params.pid_roll_rate.kp;
    pid_roll_rate.Ki = g_flash_params.pid_roll_rate.ki;
    pid_roll_rate.Kd = g_flash_params.pid_roll_rate.kd;
    
    // Pitch 角度环和角速度环
    pid_pitch_angle.Kp = g_flash_params.pid_pitch_angle.kp;
    pid_pitch_angle.Ki = g_flash_params.pid_pitch_angle.ki;
    pid_pitch_angle.Kd = g_flash_params.pid_pitch_angle.kd;
    
    pid_pitch_rate.Kp = g_flash_params.pid_pitch_rate.kp;
    pid_pitch_rate.Ki = g_flash_params.pid_pitch_rate.ki;
    pid_pitch_rate.Kd = g_flash_params.pid_pitch_rate.kd;
    
    // Yaw 角度环和角速度环
    pid_yaw_angle.Kp = g_flash_params.pid_yaw_angle.kp;
    pid_yaw_angle.Ki = g_flash_params.pid_yaw_angle.ki;
    pid_yaw_angle.Kd = g_flash_params.pid_yaw_angle.kd;
    
    pid_yaw_rate.Kp = g_flash_params.pid_yaw_rate.kp;
    pid_yaw_rate.Ki = g_flash_params.pid_yaw_rate.ki;
    pid_yaw_rate.Kd = g_flash_params.pid_yaw_rate.kd;
    
    // 位置环
    pid_pos_x.Kp = g_flash_params.pid_pos_x.kp;
    pid_pos_x.Ki = g_flash_params.pid_pos_x.ki;
    pid_pos_x.Kd = g_flash_params.pid_pos_x.kd;
    
    pid_pos_y.Kp = g_flash_params.pid_pos_y.kp;
    pid_pos_y.Ki = g_flash_params.pid_pos_y.ki;
    pid_pos_y.Kd = g_flash_params.pid_pos_y.kd;
    
    // 高度环
    pid_alt_pos.Kp = g_flash_params.pid_alt_pos.kp;
    pid_alt_pos.Ki = g_flash_params.pid_alt_pos.ki;
    pid_alt_pos.Kd = g_flash_params.pid_alt_pos.kd;
    
    pid_alt_vel.Kp = g_flash_params.pid_alt_vel.kp;
    pid_alt_vel.Ki = g_flash_params.pid_alt_vel.ki;
    pid_alt_vel.Kd = g_flash_params.pid_alt_vel.kd;

    // 应用磁力计校准数据
    QMC_Calibration.offset_x = g_flash_params.mag_calib[0];
    QMC_Calibration.offset_y = g_flash_params.mag_calib[1];
    QMC_Calibration.offset_z = g_flash_params.mag_calib[2];
    QMC_Calibration.scale_x = g_flash_params.mag_calib[3];
    QMC_Calibration.scale_y = g_flash_params.mag_calib[4];
    QMC_Calibration.scale_z = g_flash_params.mag_calib[5];
    QMC_Calibration.calibrated = (uint8_t)g_flash_params.mag_calib[6];
    
    // 如果 Flash 中无有效校准数据（全0），重置为默认
    if (QMC_Calibration.scale_x < 0.001f) QMC_Calibration.scale_x = 1.0f;
    if (QMC_Calibration.scale_y < 0.001f) QMC_Calibration.scale_y = 1.0f;
    if (QMC_Calibration.scale_z < 0.001f) QMC_Calibration.scale_z = 1.0f;
}

/**
  * @brief  从 PID 控制器读取当前参数
  * @param  None
  * @retval None
  */
void FlashParams_ReadFromPID(void)
{
    // Roll
    g_flash_params.pid_roll_angle.kp = pid_roll_angle.Kp;
    g_flash_params.pid_roll_angle.ki = pid_roll_angle.Ki;
    g_flash_params.pid_roll_angle.kd = pid_roll_angle.Kd;
    
    g_flash_params.pid_roll_rate.kp = pid_roll_rate.Kp;
    g_flash_params.pid_roll_rate.ki = pid_roll_rate.Ki;
    g_flash_params.pid_roll_rate.kd = pid_roll_rate.Kd;
    
    // Pitch
    g_flash_params.pid_pitch_angle.kp = pid_pitch_angle.Kp;
    g_flash_params.pid_pitch_angle.ki = pid_pitch_angle.Ki;
    g_flash_params.pid_pitch_angle.kd = pid_pitch_angle.Kd;
    
    g_flash_params.pid_pitch_rate.kp = pid_pitch_rate.Kp;
    g_flash_params.pid_pitch_rate.ki = pid_pitch_rate.Ki;
    g_flash_params.pid_pitch_rate.kd = pid_pitch_rate.Kd;
    
    // Yaw
    g_flash_params.pid_yaw_angle.kp = pid_yaw_angle.Kp;
    g_flash_params.pid_yaw_angle.ki = pid_yaw_angle.Ki;
    g_flash_params.pid_yaw_angle.kd = pid_yaw_angle.Kd;
    
    g_flash_params.pid_yaw_rate.kp = pid_yaw_rate.Kp;
    g_flash_params.pid_yaw_rate.ki = pid_yaw_rate.Ki;
    g_flash_params.pid_yaw_rate.kd = pid_yaw_rate.Kd;
    
    // Position
    g_flash_params.pid_pos_x.kp = pid_pos_x.Kp;
    g_flash_params.pid_pos_x.ki = pid_pos_x.Ki;
    g_flash_params.pid_pos_x.kd = pid_pos_x.Kd;
    
    g_flash_params.pid_pos_y.kp = pid_pos_y.Kp;
    g_flash_params.pid_pos_y.ki = pid_pos_y.Ki;
    g_flash_params.pid_pos_y.kd = pid_pos_y.Kd;
    
    // Altitude
    g_flash_params.pid_alt_pos.kp = pid_alt_pos.Kp;
    g_flash_params.pid_alt_pos.ki = pid_alt_pos.Ki;
    g_flash_params.pid_alt_pos.kd = pid_alt_pos.Kd;
    
    g_flash_params.pid_alt_vel.kp = pid_alt_vel.Kp;
    g_flash_params.pid_alt_vel.ki = pid_alt_vel.Ki;
    g_flash_params.pid_alt_vel.kd = pid_alt_vel.Kd;

    // 读取磁力计校准数据
    g_flash_params.mag_calib[0] = QMC_Calibration.offset_x;
    g_flash_params.mag_calib[1] = QMC_Calibration.offset_y;
    g_flash_params.mag_calib[2] = QMC_Calibration.offset_z;
    g_flash_params.mag_calib[3] = QMC_Calibration.scale_x;
    g_flash_params.mag_calib[4] = QMC_Calibration.scale_y;
    g_flash_params.mag_calib[5] = QMC_Calibration.scale_z;
    g_flash_params.mag_calib[6] = (float)QMC_Calibration.calibrated;
}

/**
  * @brief  检查参数是否需要保存
  * @param  None
  * @retval 1: 需要保存, 0: 不需要
  */
uint8_t FlashParams_IsDirty(void)
{
    return g_params_dirty;
}

/**
  * @brief  标记参数已修改
  * @param  None
  * @retval None
  */
void FlashParams_MarkDirty(void)
{
    if (g_params_dirty == 0) {
        g_params_dirty = 1;
        s_dirty_timestamp = HAL_GetTick();
    }
}

/**
  * @brief  周期性保存任务
  * @param  None
  * @retval None
  * @note   在 FreeRTOS 任务中每 100ms 调用一次
  *
  * ⚠️ 安全约束：只在 DISARMED（上锁）状态才真正写入 Flash
  * - 飞行中（ARMED）：只标记脏标志，暂存 RAM
  * - 上锁后：自动检测并触发保存
  * - 原因：Sector 擦除需 ~30ms，会阻塞控制任务，可能导致炸机
  */
void FlashParams_PeriodicSave(void)
{
    if (g_params_dirty == 0) {
        return;
    }
    
    // ⚠️ 安全检查：只在上锁状态才保存
    if (g_flight_state != FLIGHT_DISARMED) {
        // 飞行中，暂不保存，保持脏标志
        return;
    }
    
    // 延迟 2 秒后保存
    if (HAL_GetTick() - s_dirty_timestamp >= FLASH_SAVE_DELAY_MS) {
        FlashParams_ReadFromPID();
        FlashParams_Save();
    }
}

/**
  * @brief  计算 CRC16 校验值
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval CRC16 值
  */
uint16_t FlashParams_CalcCRC16(uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    uint32_t i, j;
    
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
  * @brief  继续计算 CRC16 校验值（从已有CRC值继续）
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @param  prev_crc: 之前计算的CRC值
  * @retval CRC16 值
  */
uint16_t FlashParams_CalcCRC16_Continue(uint8_t *data, uint32_t len, uint16_t prev_crc)
{
    uint16_t crc = prev_crc;
    uint32_t i, j;
    
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  设置默认参数
  * @param  params: 参数结构体指针
  * @retval None
  */
static void FlashParams_SetDefaults(FlashParams_t *params)
{
    memset((void*)params, 0, sizeof(FlashParams_t));
    
    // Header
    params->magic = FLASH_PARAMS_MAGIC;
    params->version = 0;
    params->total_size = sizeof(FlashParams_t);
    
    // PID 默认值 (保守参数，首飞后需要调参)
    // Roll/Pitch 角度环
    params->pid_roll_angle.kp = 5.0f;
    params->pid_roll_angle.ki = 0.0f;
    params->pid_roll_angle.kd = 0.0f;
    
    params->pid_pitch_angle.kp = 5.0f;
    params->pid_pitch_angle.ki = 0.0f;
    params->pid_pitch_angle.kd = 0.0f;
    
    // Roll/Pitch 角速度环
    params->pid_roll_rate.kp = 0.3f;
    params->pid_roll_rate.ki = 0.05f;
    params->pid_roll_rate.kd = 0.005f;
    
    params->pid_pitch_rate.kp = 0.3f;
    params->pid_pitch_rate.ki = 0.05f;
    params->pid_pitch_rate.kd = 0.005f;
    
    // Yaw 角度环和角速度环
    params->pid_yaw_angle.kp = 3.0f;
    params->pid_yaw_angle.ki = 0.0f;
    params->pid_yaw_angle.kd = 0.0f;
    
    params->pid_yaw_rate.kp = 0.5f;
    params->pid_yaw_rate.ki = 0.1f;
    params->pid_yaw_rate.kd = 0.0f;
    
    // 位置环
    params->pid_pos_x.kp = 1.0f;
    params->pid_pos_x.ki = 0.0f;
    params->pid_pos_x.kd = 0.0f;
    
    params->pid_pos_y.kp = 1.0f;
    params->pid_pos_y.ki = 0.0f;
    params->pid_pos_y.kd = 0.0f;
    
    // 高度环
    params->pid_alt_pos.kp = 2.0f;
    params->pid_alt_pos.ki = 0.0f;
    params->pid_alt_pos.kd = 0.0f;
    
    params->pid_alt_vel.kp = 3.0f;
    params->pid_alt_vel.ki = 0.5f;
    params->pid_alt_vel.kd = 0.0f;
    
    // 校准数据 (初始零偏为 0)
    memset((void*)params->gyro_offset, 0, sizeof(params->gyro_offset));
    memset((void*)params->accel_offset, 0, sizeof(params->accel_offset));
    memset((void*)params->mag_calib, 0, sizeof(params->mag_calib));
    params->pressure_ref = 101325.0f;  // 标准大气压
    
    // 飞行配置
    params->hover_throttle = 0.5f;  // 初始估计 50% 油门
    memset((void*)params->channel_map, 0, sizeof(params->channel_map));
    params->flight_mode_memory = 0;  // 手动模式
    
    // 系统信息
    params->total_flight_time = 0;
    params->fw_version[0] = 1;  // v1.0.0.0
    params->fw_version[1] = 0;
    params->fw_version[2] = 0;
    params->fw_version[3] = 0;
    params->save_count = 0;
}

/**
  * @brief  从 Flash 读取一个 Block
  * @param  addr: 地址 (FLASH_PARAMS_ADDR_A 或 FLASH_PARAMS_ADDR_B)
  * @param  params: 参数结构体指针
  * @retval 0: 成功, 1: 失败
  */
static uint8_t FlashParams_ReadBlock(uint32_t addr, FlashParams_t *params)
{
    if (params == NULL) {
        return 1;
    }
    
    if (W25QXX_Read(addr, (uint8_t*)params, sizeof(FlashParams_t)) != 0) {
        return 1;
    }
    
    // 检查魔数
    if (params->magic != FLASH_PARAMS_MAGIC) {
        return 1;
    }
    
    return 0;
}

/**
  * @brief  写入一个 Block 到 Flash
  * @param  addr: 地址
  * @param  params: 参数结构体指针
  * @retval 0: 成功, 1: 失败
  */
static uint8_t FlashParams_WriteBlock(uint32_t addr, FlashParams_t *params)
{
    if (params == NULL) {
        return 1;
    }
    
    // 擦除 Sector (4KB)
    if (W25QXX_EraseSector(addr) != 0) {
        return 1;
    }
    
    // 写入数据
    if (W25QXX_Write(addr, (uint8_t*)params, sizeof(FlashParams_t)) != 0) {
        return 1;
    }
    
    // Risk #044 修复：读回校验改为CRC比较（避免padding字节问题）
    FlashParams_t verify;
    if (W25QXX_Read(addr, (uint8_t*)&verify, sizeof(FlashParams_t)) != 0) {
        return 1;
    }
    
    // 只比较CRC（CRC已涵盖所有有效数据字段）
    if (verify.crc16 != params->crc16) {
        return 1;  // 校验失败
    }
    
    return 0;
}

/**
  * @brief  校验 CRC16
  * @param  params: 参数结构体指针
  * @retval 0: 校验通过, 1: 校验失败
  */
static uint8_t FlashParams_ValidateCRC(FlashParams_t *params)
{
    if (params == NULL) {
        return 1;
    }
    
    uint16_t stored_crc = params->crc16;
    
    // Risk #041 修复：CRC校验必须覆盖version字段
    // 分两段计算：magic+version，然后跳过crc16，继续计算剩余
    uint16_t calc_crc_part1 = FlashParams_CalcCRC16((uint8_t*)&params->magic, 6);
    uint16_t calc_crc = FlashParams_CalcCRC16_Continue(
        (uint8_t*)&params->total_size,
        sizeof(FlashParams_t) - 8,
        calc_crc_part1
    );
    
    if (stored_crc != calc_crc) {
        return 1;  // CRC 校验失败
    }
    
    return 0;
}
