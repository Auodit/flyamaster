/**
 * @file battery.h
 * @brief 电池监控模块 (电压 + 电流 + mAh 计算)
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 使用 ADC1 双通道 DMA 循环采样：
 * - PC0 (ADC1_IN10): 电池电压 (分压 1:11)
 * - PC5 (ADC1_IN15): 电流传感器 (来自 PDB/电调 CURR 脚)
 * 
 * 功能：
 * 1. 实时电压/电流监测
 * 2. mAh 消耗积分计算
 * 3. 低电压报警
 * 4. 电流异常检测 (堵转预警)
 * 
 * 硬件接线：
 * - PDB/电调 CURR 脚 -> PC5 (无需分压，0-3.3V)
 * - 电池电压 -> 分压电阻 -> PC0
 */

#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* ADC 通道索引 (DMA 缓冲区顺序) */
#define BATTERY_ADC_VOLTAGE_IDX     0       /**< 电压通道索引 */
#define BATTERY_ADC_CURRENT_IDX     1       /**< 电流通道索引 */
#define BATTERY_ADC_CHANNELS        2       /**< ADC 通道数 */

/* 电压标定参数 */
#define BATTERY_VBAT_SCALE          11.0f   /**< 分压比 (10k:1k = 11:1) */
#define BATTERY_VREF                3.3f    /**< ADC 参考电压 */
#define BATTERY_ADC_MAX             4095    /**< 12位 ADC 最大值 */

/* 电流标定参数 (根据电调说明书调整) */
#define BATTERY_CURRENT_SCALE       33.3f   /**< 电流比例 (A/V), 3.3V = 100A */
#define BATTERY_CURRENT_OFFSET      0.0f    /**< 零点偏移 (V) */

/* 电池参数 (3S LiPo) */
#define BATTERY_CELLS               3       /**< 电池节数 */
#define BATTERY_CELL_MIN            3.5f    /**< 单节最低电压 (V) */
#define BATTERY_CELL_WARNING        3.6f    /**< 单节警告电压 (V) */
#define BATTERY_CELL_FULL           4.2f    /**< 单节满电电压 (V) */

/* 报警阈值 */
#define BATTERY_VOLTAGE_WARNING     (BATTERY_CELLS * BATTERY_CELL_WARNING)  /**< 10.8V */
#define BATTERY_VOLTAGE_CRITICAL    (BATTERY_CELLS * BATTERY_CELL_MIN)      /**< 10.5V */
#define BATTERY_CURRENT_MAX         80.0f   /**< 最大电流报警 (A) */

/* 滤波参数 */
#define BATTERY_VOLTAGE_LPF_ALPHA   0.05f   /**< 电压滤波系数 (越小越平滑) */
#define BATTERY_CURRENT_LPF_ALPHA   0.10f   /**< 电流滤波系数 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电池状态枚举
 */
typedef enum {
    BATTERY_OK = 0,         /**< 正常 */
    BATTERY_WARNING,        /**< 低电压警告 */
    BATTERY_CRITICAL,       /**< 电压临界 */
    BATTERY_OVERCURRENT     /**< 过流 */
} Battery_Status_t;

/**
 * @brief 电池数据结构体
 */
typedef struct {
    /* 原始数据 */
    float voltage_raw;          /**< 原始电压 (V) */
    float current_raw;          /**< 原始电流 (A) */
    
    /* 滤波后数据 */
    float voltage;              /**< 滤波后电压 (V) */
    float current;              /**< 滤波后电流 (A) */
    
    /* 计算值 */
    float cell_voltage;         /**< 单节电压 (V) */
    uint8_t cell_count;         /**< 电池节数 */
    uint8_t percentage;         /**< 电量百分比 (0-100%) */
    
    /* 消耗统计 */
    float mah_drawn;            /**< 已消耗电量 (mAh) */
    float mwh_drawn;            /**< 已消耗能量 (mWh) */
    float watt;                 /**< 瞬时功率 (W) */
    
    /* 状态标志 */
    Battery_Status_t status;    /**< 电池状态 */
    bool low_voltage_warning;   /**< 低电压警告 */
    bool low_voltage_critical;  /**< 电压临界 */
    bool overcurrent;           /**< 过流标志 */
    
    /* 更新计数 */
    uint32_t update_count;      /**< 更新次数 */
    uint32_t last_update_tick;  /**< 最后更新时间 */
} Battery_t;

/* Exported variables --------------------------------------------------------*/
extern Battery_t g_battery;                             /**< 全局电池数据 */
extern uint16_t g_adc_dma_buffer[BATTERY_ADC_CHANNELS]; /**< ADC DMA 缓冲区 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电池监控
 * @note 启动 ADC DMA 循环采样
 */
void Battery_Init(void);

/**
 * @brief 更新电池数据
 * @param dt 调用间隔 (秒)
 * @note 在定时任务中周期性调用 (建议 10Hz)
 */
void Battery_Update(float dt);

/**
 * @brief 获取电池电压
 * @return float 电压 (V)
 */
float Battery_GetVoltage(void);

/**
 * @brief 获取电池电流
 * @return float 电流 (A)
 */
float Battery_GetCurrent(void);

/**
 * @brief 获取已消耗电量
 * @return float mAh
 */
float Battery_GetMAhDrawn(void);

/**
 * @brief 获取电量百分比
 * @return uint8_t 百分比 (0-100)
 */
uint8_t Battery_GetPercentage(void);

/**
 * @brief 获取电池状态
 * @return Battery_Status_t 状态
 */
Battery_Status_t Battery_GetStatus(void);

/**
 * @brief 检查是否低电压
 * @return bool true=低电压
 */
bool Battery_IsLowVoltage(void);

/**
 * @brief 检查是否过流
 * @return bool true=过流
 */
bool Battery_IsOvercurrent(void);

/**
 * @brief 重置 mAh 计数器
 */
void Battery_ResetMAh(void);

/**
 * @brief 设置电流标定参数
 * @param scale 电流比例 (A/V)
 * @param offset 零点偏移 (V)
 */
void Battery_SetCurrentCalibration(float scale, float offset);

/**
 * @brief 设置电压标定参数
 * @param scale 分压比
 */
void Battery_SetVoltageCalibration(float scale);

/**
 * @brief 自动检测电池节数
 * @return uint8_t 检测到的节数 (2-6S)
 */
uint8_t Battery_AutoDetectCells(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */