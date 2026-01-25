/**
 * @file gps.h
 * @brief GPS 模块驱动 (NMEA 协议解析)
 * @version 1.0.0
 * @date 2026-01-18
 * 
 * @details
 * 支持 NMEA-0183 协议解析：
 * - GNGGA: 定位信息
 * - GNRMC: 推荐最小定位信息
 * - GNVTG: 地面速度
 * 
 * 硬件连接:
 * - UART5 (PC12/PD2)
 * - 波特率: 115200 (可配置)
 * - DMA 循环接收
 * - DMA: DMA1_Stream0 (RX)
 */

#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define GPS_RX_BUFFER_SIZE      512     /**< 接收缓冲区大小 */
#define GPS_SENTENCE_MAX_LEN    128     /**< 最大语句长度 */

/* 地球半径 (米) */
#define GPS_EARTH_RADIUS        6371000.0f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief GPS 定位质量枚举
 */
typedef enum {
    GPS_FIX_NONE = 0,       /**< 无定位 */
    GPS_FIX_GPS,            /**< GPS 定位 */
    GPS_FIX_DGPS,           /**< 差分 GPS */
    GPS_FIX_PPS,            /**< PPS 定位 */
    GPS_FIX_RTK,            /**< RTK 固定解 */
    GPS_FIX_RTK_FLOAT,      /**< RTK 浮点解 */
    GPS_FIX_ESTIMATED,      /**< 估算定位 */
    GPS_FIX_MANUAL,         /**< 手动输入 */
    GPS_FIX_SIMULATION      /**< 模拟模式 */
} GPS_FixQuality_t;

/**
 * @brief GPS 时间结构体
 */
typedef struct {
    uint8_t hour;           /**< 小时 (0-23) */
    uint8_t minute;         /**< 分钟 (0-59) */
    uint8_t second;         /**< 秒 (0-59) */
    uint16_t millisecond;   /**< 毫秒 (0-999) */
} GPS_Time_t;

/**
 * @brief GPS 日期结构体
 */
typedef struct {
    uint8_t day;            /**< 日 (1-31) */
    uint8_t month;          /**< 月 (1-12) */
    uint16_t year;          /**< 年 (2000-2099) */
} GPS_Date_t;

/**
 * @brief GPS 数据结构体
 */
typedef struct {
    /* 定位状态 */
    bool valid;                     /**< 数据有效 */
    GPS_FixQuality_t fix_quality;   /**< 定位质量 */
    uint8_t satellites;             /**< 卫星数量 */
    
    /* 位置 */
    double latitude;                /**< 纬度 (度) */
    double longitude;               /**< 经度 (度) */
    float altitude;                 /**< 海拔高度 (米) */
    
    /* 速度和航向 */
    float speed_knots;              /**< 速度 (节) */
    float speed_kmh;                /**< 速度 (km/h) */
    float course;                   /**< 航向 (度) */
    
    /* 精度 */
    float hdop;                     /**< 水平精度因子 */
    float vdop;                     /**< 垂直精度因子 */
    float pdop;                     /**< 位置精度因子 */
    
    /* 时间日期 */
    GPS_Time_t time;                /**< UTC 时间 */
    GPS_Date_t date;                /**< 日期 */
    
    /* 相对位置 (相对于起飞点) */
    float home_distance;            /**< 距离起飞点 (米) */
    float home_bearing;             /**< 起飞点方位 (度) */
    float rel_x;                    /**< 相对 X 位置 (米, 东向为正) */
    float rel_y;                    /**< 相对 Y 位置 (米, 北向为正) */
    
    /* 起飞点 */
    bool home_set;                  /**< 起飞点已设置 */
    double home_lat;                /**< 起飞点纬度 */
    double home_lon;                /**< 起飞点经度 */
    float home_alt;                 /**< 起飞点海拔 */
    
    /* 更新计数 */
    uint32_t update_count;          /**< 更新次数 */
    uint32_t last_update_tick;      /**< 最后更新时间 */
} GPS_Data_t;

/* Exported variables --------------------------------------------------------*/
extern GPS_Data_t g_gps_data;       /**< 全局 GPS 数据 */
extern volatile bool g_gps_new_data; /**< 新数据标志 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 GPS 模块
 */
void GPS_Init(void);

/**
 * @brief 处理 GPS 数据
 * @note 在 gps_task 中周期性调用
 */
void GPS_Process(void);

/**
 * @brief 解析 NMEA 语句
 * @param sentence NMEA 语句
 * @return bool true=解析成功
 */
bool GPS_ParseSentence(const char *sentence);

/**
 * @brief 解析 GNGGA 语句
 * @param sentence GNGGA 语句
 * @return bool true=解析成功
 */
bool GPS_ParseGGA(const char *sentence);

/**
 * @brief 解析 GNRMC 语句
 * @param sentence GNRMC 语句
 * @return bool true=解析成功
 */
bool GPS_ParseRMC(const char *sentence);

/**
 * @brief 解析 GNVTG 语句
 * @param sentence GNVTG 语句
 * @return bool true=解析成功
 */
bool GPS_ParseVTG(const char *sentence);

/**
 * @brief 设置起飞点
 */
void GPS_SetHome(void);

/**
 * @brief 清除起飞点
 */
void GPS_ClearHome(void);

/**
 * @brief 计算相对位置
 */
void GPS_CalculateRelativePosition(void);

/**
 * @brief 计算两点间距离
 * @param lat1 点1纬度
 * @param lon1 点1经度
 * @param lat2 点2纬度
 * @param lon2 点2经度
 * @return float 距离 (米)
 */
float GPS_CalculateDistance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief 计算两点间方位角
 * @param lat1 点1纬度
 * @param lon1 点1经度
 * @param lat2 点2纬度
 * @param lon2 点2经度
 * @return float 方位角 (度, 0-360)
 */
float GPS_CalculateBearing(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief 检查 GPS 是否有效
 * @return bool true=有效
 */
bool GPS_IsValid(void);

/**
 * @brief 检查是否有 3D 定位
 * @return bool true=3D 定位
 */
bool GPS_Has3DFix(void);

/**
 * @brief 获取卫星数量
 * @return uint8_t 卫星数量
 */
uint8_t GPS_GetSatellites(void);

/**
 * @brief UART 空闲中断回调
 */
void GPS_UART_IdleCallback(void);

/**
 * @brief DMA 接收完成回调
 */
void GPS_DMA_RxCpltCallback(void);

/**
 * @brief 验证 NMEA 校验和
 * @param sentence NMEA 语句
 * @return bool true=校验通过
 */
bool GPS_VerifyChecksum(const char *sentence);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */