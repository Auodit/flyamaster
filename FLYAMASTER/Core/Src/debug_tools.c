/**
 * @file debug_tools.c
 * @brief 调试工具模块实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "debug_tools.h"
#include "i2c.h"
#include "ring_buffer.h"
#include "qmc5883l.h"
#include "sbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portable.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define I2C_SCAN_TIMEOUT        10      /**< I2C 扫描超时 (ms) */
#define MAG_CALIB_SAMPLES       50      /**< 标定采样次数 */

/* Private variables ---------------------------------------------------------*/
MagCurrentCalib_t g_mag_current_calib = {0};

/* 标定过程中的累积值 */
static float calib_low_x = 0, calib_low_y = 0, calib_low_z = 0;
static float calib_high_x = 0, calib_high_y = 0, calib_high_z = 0;
static float calib_low_throttle = 0, calib_high_throttle = 0;
static uint16_t calib_low_count = 0, calib_high_count = 0;

/* 已知设备列表 */
static const I2C_DeviceInfo_t known_devices[] = {
    {I2C_ADDR_MPU6050,      "MPU6050",      false},
    {I2C_ADDR_MPU6050_ALT,  "MPU6050 (ALT)", false},
    {I2C_ADDR_QMC5883L,     "QMC5883L",     false},
    {I2C_ADDR_HMC5883L,     "HMC5883L",     false},
    {I2C_ADDR_SPL06,        "SPL06",        false},
    {I2C_ADDR_SPL06_ALT,    "SPL06 (ALT)",  false},
    {I2C_ADDR_BMP280,       "BMP280",       false},
    {I2C_ADDR_MS5611,       "MS5611",       false},
    {I2C_ADDR_EEPROM,       "EEPROM",       false},
};

#define KNOWN_DEVICE_COUNT  (sizeof(known_devices) / sizeof(known_devices[0]))

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 获取设备名称
 */
const char* I2C_GetDeviceName(uint8_t address)
{
    for (uint8_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
        if (known_devices[i].address == address) {
            return known_devices[i].name;
        }
    }
    return "Unknown";
}

/**
 * @brief 扫描 I2C 总线
 */
uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c)
{
    uint8_t found_count = 0;
    HAL_StatusTypeDef status;
    
    Log_Printf("\r\n=== I2C Bus Scan ===\r\n");
    Log_Printf("Scanning address range 0x08 ~ 0x77...\r\n");
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        /* 尝试与设备通信 */
        status = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, I2C_SCAN_TIMEOUT);
        
        if (status == HAL_OK) {
            const char *name = I2C_GetDeviceName(addr);
            Log_Printf("  Found: 0x%02X - %s\r\n", addr, name);
            found_count++;
        }
    }
    
    Log_Printf("Scan complete. Found %d device(s).\r\n", found_count);
    
    return found_count;
}

/**
 * @brief 扫描所有 I2C 总线
 */
void I2C_ScanAllBuses(void)
{
    Log_Printf("\r\n========================================\r\n");
    Log_Printf("       I2C Bus Scanner\r\n");
    Log_Printf("========================================\r\n");
    
    /* 扫描 I2C1 */
    Log_Printf("\r\n[I2C1] (MPU6050, SPL06)\r\n");
    I2C_ScanBus(&hi2c1);
    
    /* 扫描 I2C2 */
    Log_Printf("\r\n[I2C2] (QMC5883L)\r\n");
    I2C_ScanBus(&hi2c2);
    
    Log_Printf("\r\n========================================\r\n");
}

/**
 * @brief 开始磁力计电流补偿标定
 */
void MagCurrentCalib_Start(void)
{
    /* 清空累积值 */
    calib_low_x = calib_low_y = calib_low_z = 0;
    calib_high_x = calib_high_y = calib_high_z = 0;
    calib_low_throttle = calib_high_throttle = 0;
    calib_low_count = calib_high_count = 0;
    
    g_mag_current_calib.calibrated = false;
    
    Log_Printf("\r\n=== Magnetometer Current Calibration ===\r\n");
    Log_Printf("Step 1: Keep throttle at MINIMUM and wait...\r\n");
}

/**
 * @brief 采集低油门数据
 */
void MagCurrentCalib_SampleLow(void)
{
    /* 从磁力计和遥控器获取当前值 */
    float mag_x = g_qmc5883l_data.mag_x;
    float mag_y = g_qmc5883l_data.mag_y;
    float mag_z = g_qmc5883l_data.mag_z;
    float throttle = g_rc_data.throttle;
    
    calib_low_x += mag_x;
    calib_low_y += mag_y;
    calib_low_z += mag_z;
    calib_low_throttle += throttle;
    calib_low_count++;
    
    if (calib_low_count >= MAG_CALIB_SAMPLES) {
        /* 计算平均值 */
        calib_low_x /= calib_low_count;
        calib_low_y /= calib_low_count;
        calib_low_z /= calib_low_count;
        calib_low_throttle /= calib_low_count;
        
        Log_Printf("Low throttle data collected.\r\n");
        Log_Printf("  Mag: X=%.2f, Y=%.2f, Z=%.2f\r\n", 
                   calib_low_x, calib_low_y, calib_low_z);
        Log_Printf("  Throttle: %.2f\r\n", calib_low_throttle);
        Log_Printf("\r\nStep 2: Increase throttle to MAXIMUM and wait...\r\n");
    }
}

/**
 * @brief 采集高油门数据
 */
void MagCurrentCalib_SampleHigh(void)
{
    /* 从磁力计和遥控器获取当前值 */
    float mag_x = g_qmc5883l_data.mag_x;
    float mag_y = g_qmc5883l_data.mag_y;
    float mag_z = g_qmc5883l_data.mag_z;
    float throttle = g_rc_data.throttle;
    
    calib_high_x += mag_x;
    calib_high_y += mag_y;
    calib_high_z += mag_z;
    calib_high_throttle += throttle;
    calib_high_count++;
    
    if (calib_high_count >= MAG_CALIB_SAMPLES) {
        /* 计算平均值 */
        calib_high_x /= calib_high_count;
        calib_high_y /= calib_high_count;
        calib_high_z /= calib_high_count;
        calib_high_throttle /= calib_high_count;
        
        Log_Printf("High throttle data collected.\r\n");
        Log_Printf("  Mag: X=%.2f, Y=%.2f, Z=%.2f\r\n", 
                   calib_high_x, calib_high_y, calib_high_z);
        Log_Printf("  Throttle: %.2f\r\n", calib_high_throttle);
        
        /* 自动完成标定 */
        MagCurrentCalib_Finish();
    }
}

/**
 * @brief 完成标定并计算系数
 */
void MagCurrentCalib_Finish(void)
{
    float delta_throttle = calib_high_throttle - calib_low_throttle;
    
    if (delta_throttle < 0.1f) {
        Log_Printf("\r\nERROR: Throttle difference too small!\r\n");
        return;
    }
    
    /* 计算补偿系数: K = (Mag_high - Mag_low) / (Throttle_high - Throttle_low) */
    g_mag_current_calib.k_x = (calib_high_x - calib_low_x) / delta_throttle;
    g_mag_current_calib.k_y = (calib_high_y - calib_low_y) / delta_throttle;
    g_mag_current_calib.k_z = (calib_high_z - calib_low_z) / delta_throttle;
    g_mag_current_calib.calibrated = true;
    
    MagCurrentCalib_PrintResult();
}

/**
 * @brief 打印标定结果
 */
void MagCurrentCalib_PrintResult(void)
{
    Log_Printf("\r\n=== Calibration Complete ===\r\n");
    Log_Printf("Compensation coefficients:\r\n");
    Log_Printf("  K_x = %.4f\r\n", g_mag_current_calib.k_x);
    Log_Printf("  K_y = %.4f\r\n", g_mag_current_calib.k_y);
    Log_Printf("  K_z = %.4f\r\n", g_mag_current_calib.k_z);
    Log_Printf("\r\nCopy these to your code:\r\n");
    Log_Printf("#define MAG_CURRENT_COMP_KX  %.4ff\r\n", g_mag_current_calib.k_x);
    Log_Printf("#define MAG_CURRENT_COMP_KY  %.4ff\r\n", g_mag_current_calib.k_y);
    Log_Printf("#define MAG_CURRENT_COMP_KZ  %.4ff\r\n", g_mag_current_calib.k_z);
}

/**
 * @brief 应用电流补偿
 */
void MagCurrentCalib_Apply(float *mag_x, float *mag_y, float *mag_z, float throttle)
{
    if (!g_mag_current_calib.calibrated) {
        return;
    }
    
    /* Mag_corrected = Mag_raw - Throttle × K */
    *mag_x -= throttle * g_mag_current_calib.k_x;
    *mag_y -= throttle * g_mag_current_calib.k_y;
    *mag_z -= throttle * g_mag_current_calib.k_z;
}

/**
 * @brief 打印系统状态
 */
void Debug_PrintSystemStatus(void)
{
    Log_Printf("\r\n=== System Status ===\r\n");
    Log_Printf("MCU: STM32F405RGT6 @ 168MHz\r\n");
    Log_Printf("Tick: %lu ms\r\n", HAL_GetTick());
}

/**
 * @brief 打印任务状态
 */
void Debug_PrintTaskStatus(void)
{
#if configUSE_TRACE_FACILITY == 1
    char buffer[512];
    Log_Printf("\r\n=== Task Status ===\r\n");
    vTaskList(buffer);
    Log_Printf("%s\r\n", buffer);
#else
    Log_Printf("Task tracing not enabled.\r\n");
#endif
}

/**
 * @brief 打印内存使用情况
 */
void Debug_PrintMemoryUsage(void)
{
    Log_Printf("\r\n=== Memory Usage ===\r\n");
    Log_Printf("FreeRTOS Heap Free: %u bytes\r\n", (unsigned int)xPortGetFreeHeapSize());
    Log_Printf("FreeRTOS Heap Min:  %u bytes\r\n", (unsigned int)xPortGetMinimumEverFreeHeapSize());
}
