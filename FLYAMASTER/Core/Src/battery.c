/**
 * @file battery.c
 * @brief 电池监控模块实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "battery.h"
#include "adc.h"

/* Private defines -----------------------------------------------------------*/

/* mAh 积分系数: A * s -> mAh */
/* 1 mAh = 1 mA * 1 h = 0.001 A * 3600 s */
/* mAh = A * dt * 1000 / 3600 = A * dt * 0.277778 */
#define MAH_CONVERSION_FACTOR   0.277778f

/* Private variables ---------------------------------------------------------*/
Battery_t g_battery = {0};
uint16_t g_adc_dma_buffer[BATTERY_ADC_CHANNELS] = {0};

/* 标定参数 (可运行时修改) */
static float s_voltage_scale = BATTERY_VBAT_SCALE;
static float s_current_scale = BATTERY_CURRENT_SCALE;
static float s_current_offset = BATTERY_CURRENT_OFFSET;

/* Private function prototypes -----------------------------------------------*/
static float Battery_ApplyLPF(float old_val, float new_val, float alpha);
static float Battery_ADCToVoltage(uint16_t adc_value);
static float Battery_ADCToCurrent(uint16_t adc_value);
static uint8_t Battery_CalculatePercentage(float voltage, uint8_t cells);
static void Battery_UpdateStatus(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电池监控
 */
void Battery_Init(void)
{
    /* 清零数据结构 */
    g_battery.voltage_raw = 0.0f;
    g_battery.current_raw = 0.0f;
    g_battery.voltage = 0.0f;
    g_battery.current = 0.0f;
    g_battery.cell_voltage = 0.0f;
    g_battery.cell_count = BATTERY_CELLS;
    g_battery.percentage = 0;
    g_battery.mah_drawn = 0.0f;
    g_battery.mwh_drawn = 0.0f;
    g_battery.watt = 0.0f;
    g_battery.status = BATTERY_OK;
    g_battery.low_voltage_warning = false;
    g_battery.low_voltage_critical = false;
    g_battery.overcurrent = false;
    g_battery.update_count = 0;
    g_battery.last_update_tick = 0;
    
    /* 启动 ADC DMA 循环采样 */
    /* 注意: hadc1 由 CubeMX 生成 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc_dma_buffer, BATTERY_ADC_CHANNELS);
}

/**
 * @brief 更新电池数据
 * @param dt 调用间隔 (秒)
 */
void Battery_Update(float dt)
{
    /* 1. 读取 ADC 原始值并转换 */
    g_battery.voltage_raw = Battery_ADCToVoltage(g_adc_dma_buffer[BATTERY_ADC_VOLTAGE_IDX]);
    g_battery.current_raw = Battery_ADCToCurrent(g_adc_dma_buffer[BATTERY_ADC_CURRENT_IDX]);
    
    /* 2. 低通滤波 */
    g_battery.voltage = Battery_ApplyLPF(g_battery.voltage, g_battery.voltage_raw, 
                                          BATTERY_VOLTAGE_LPF_ALPHA);
    g_battery.current = Battery_ApplyLPF(g_battery.current, g_battery.current_raw, 
                                          BATTERY_CURRENT_LPF_ALPHA);
    
    /* 过滤负电流 */
    if (g_battery.current < 0.0f) {
        g_battery.current = 0.0f;
    }
    
    /* 3. 计算单节电压 */
    if (g_battery.cell_count > 0) {
        g_battery.cell_voltage = g_battery.voltage / (float)g_battery.cell_count;
    }
    
    /* 4. 计算电量百分比 */
    g_battery.percentage = Battery_CalculatePercentage(g_battery.voltage, g_battery.cell_count);
    
    /* 5. 计算瞬时功率 */
    g_battery.watt = g_battery.voltage * g_battery.current;
    
    /* 6. 积分计算 mAh 消耗 */
    /* mAh = A * dt * 0.277778 */
    float mah_step = g_battery.current * dt * MAH_CONVERSION_FACTOR;
    g_battery.mah_drawn += mah_step;
    
    /* 7. 积分计算 mWh 消耗 */
    /* mWh = W * dt * 0.277778 */
    float mwh_step = g_battery.watt * dt * MAH_CONVERSION_FACTOR;
    g_battery.mwh_drawn += mwh_step;
    
    /* 8. 更新状态标志 */
    Battery_UpdateStatus();
    
    /* 9. 更新计数器 */
    g_battery.update_count++;
    g_battery.last_update_tick = HAL_GetTick();
}

/**
 * @brief 获取电池电压
 */
float Battery_GetVoltage(void)
{
    return g_battery.voltage;
}

/**
 * @brief 获取电池电流
 */
float Battery_GetCurrent(void)
{
    return g_battery.current;
}

/**
 * @brief 获取已消耗电量
 */
float Battery_GetMAhDrawn(void)
{
    return g_battery.mah_drawn;
}

/**
 * @brief 获取电量百分比
 */
uint8_t Battery_GetPercentage(void)
{
    return g_battery.percentage;
}

/**
 * @brief 获取电池状态
 */
Battery_Status_t Battery_GetStatus(void)
{
    return g_battery.status;
}

/**
 * @brief 检查是否低电压
 */
bool Battery_IsLowVoltage(void)
{
    return g_battery.low_voltage_warning || g_battery.low_voltage_critical;
}

/**
 * @brief 检查是否过流
 */
bool Battery_IsOvercurrent(void)
{
    return g_battery.overcurrent;
}

/**
 * @brief 重置 mAh 计数器
 */
void Battery_ResetMAh(void)
{
    g_battery.mah_drawn = 0.0f;
    g_battery.mwh_drawn = 0.0f;
}

/**
 * @brief 设置电流标定参数
 */
void Battery_SetCurrentCalibration(float scale, float offset)
{
    s_current_scale = scale;
    s_current_offset = offset;
}

/**
 * @brief 设置电压标定参数
 */
void Battery_SetVoltageCalibration(float scale)
{
    s_voltage_scale = scale;
}

/**
 * @brief 自动检测电池节数
 * @return 检测到的节数 (2-6S)
 */
uint8_t Battery_AutoDetectCells(void)
{
    float voltage = g_battery.voltage;
    uint8_t cells = 0;
    
    /* 根据电压范围判断节数 */
    /* 假设每节电压范围 3.0V - 4.35V */
    if (voltage > 21.0f) {
        cells = 6;  /* 6S: 21.0V - 26.1V */
    } else if (voltage > 17.5f) {
        cells = 5;  /* 5S: 17.5V - 21.75V */
    } else if (voltage > 14.0f) {
        cells = 4;  /* 4S: 14.0V - 17.4V */
    } else if (voltage > 10.5f) {
        cells = 3;  /* 3S: 10.5V - 13.05V */
    } else if (voltage > 7.0f) {
        cells = 2;  /* 2S: 7.0V - 8.7V */
    } else {
        cells = 1;  /* 1S 或未接电池 */
    }
    
    g_battery.cell_count = cells;
    return cells;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 低通滤波器
 */
static float Battery_ApplyLPF(float old_val, float new_val, float alpha)
{
    return old_val * (1.0f - alpha) + new_val * alpha;
}

/**
 * @brief ADC 值转电压
 */
static float Battery_ADCToVoltage(uint16_t adc_value)
{
    /* ADC 值 -> 引脚电压 -> 实际电压 */
    float pin_voltage = (float)adc_value * (BATTERY_VREF / (float)BATTERY_ADC_MAX);
    return pin_voltage * s_voltage_scale;
}

/**
 * @brief ADC 值转电流
 */
static float Battery_ADCToCurrent(uint16_t adc_value)
{
    /* ADC 值 -> 引脚电压 -> 实际电流 */
    float pin_voltage = (float)adc_value * (BATTERY_VREF / (float)BATTERY_ADC_MAX);
    return (pin_voltage - s_current_offset) * s_current_scale;
}

/**
 * @brief 计算电量百分比
 * @note 基于电压的简单线性估算
 */
static uint8_t Battery_CalculatePercentage(float voltage, uint8_t cells)
{
    if (cells == 0) return 0;
    
    float cell_voltage = voltage / (float)cells;
    
    /* 线性映射: 3.5V = 0%, 4.2V = 100% */
    float min_v = BATTERY_CELL_MIN;
    float max_v = BATTERY_CELL_FULL;
    
    if (cell_voltage <= min_v) {
        return 0;
    } else if (cell_voltage >= max_v) {
        return 100;
    } else {
        float percent = (cell_voltage - min_v) / (max_v - min_v) * 100.0f;
        return (uint8_t)percent;
    }
}

/**
 * @brief 更新电池状态
 * @note 使用动态阈值，根据检测到的电池节数计算
 */
static void Battery_UpdateStatus(void)
{
    /* 检查过流 */
    g_battery.overcurrent = (g_battery.current > BATTERY_CURRENT_MAX);
    
    /* 动态计算电压阈值 (根据实际电池节数) */
    float warning_voltage, critical_voltage;
    if (g_battery.cell_count > 0) {
        warning_voltage = (float)g_battery.cell_count * BATTERY_CELL_WARNING;
        critical_voltage = (float)g_battery.cell_count * BATTERY_CELL_MIN;
    } else {
        /* 未检测到电池，使用默认 3S 阈值 */
        warning_voltage = BATTERY_VOLTAGE_WARNING;
        critical_voltage = BATTERY_VOLTAGE_CRITICAL;
    }
    
    /* 检查低电压 (使用动态阈值) */
    g_battery.low_voltage_warning = (g_battery.voltage < warning_voltage);
    g_battery.low_voltage_critical = (g_battery.voltage < critical_voltage);
    
    /* 确定状态优先级: 过流 > 临界 > 警告 > 正常 */
    if (g_battery.overcurrent) {
        g_battery.status = BATTERY_OVERCURRENT;
    } else if (g_battery.low_voltage_critical) {
        g_battery.status = BATTERY_CRITICAL;
    } else if (g_battery.low_voltage_warning) {
        g_battery.status = BATTERY_WARNING;
    } else {
        g_battery.status = BATTERY_OK;
    }
}