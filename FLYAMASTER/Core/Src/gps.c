/**
 * @file gps.c
 * @brief GPS 模块驱动实现
 * @version 1.0.0
 * @date 2026-01-18
 */

/* Includes ------------------------------------------------------------------*/
#include "gps.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define GPS_DEG_TO_RAD      (3.14159265358979f / 180.0f)
#define GPS_RAD_TO_DEG      (180.0f / 3.14159265358979f)

/* Exported variables --------------------------------------------------------*/
GPS_Data_t g_gps_data = {0};
volatile bool g_gps_new_data = false;

/* Private variables ---------------------------------------------------------*/
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static char gps_sentence_buffer[GPS_SENTENCE_MAX_LEN];
static uint16_t gps_sentence_index = 0;

/* Private function prototypes -----------------------------------------------*/
static double GPS_ParseCoordinate(const char *str, char direction);
static float GPS_ParseFloat(const char *str);
static int GPS_ParseInt(const char *str);
static char* GPS_GetField(const char *sentence, uint8_t field_num, char *buffer, uint8_t buf_size);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 GPS 模块
 */
void GPS_Init(void)
{
    /* 清空数据 */
    memset(&g_gps_data, 0, sizeof(g_gps_data));
    memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
    
    g_gps_data.valid = false;
    g_gps_data.home_set = false;
    g_gps_new_data = false;
    
    /* 启用 UART5 空闲中断 */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    
    /* 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(&huart5, gps_rx_buffer, GPS_RX_BUFFER_SIZE);
}

/**
 * @brief 处理 GPS 数据
 */
void GPS_Process(void)
{
    /* 检查超时 */
    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - g_gps_data.last_update_tick) > 2000) {
        g_gps_data.valid = false;
    }
    
    /* 如果有新数据，计算相对位置 */
    if (g_gps_new_data && g_gps_data.valid) {
        g_gps_new_data = false;
        
        /* 自动设置起飞点 */
        if (!g_gps_data.home_set && g_gps_data.satellites >= 6) {
            GPS_SetHome();
        }
        
        /* 计算相对位置 */
        if (g_gps_data.home_set) {
            GPS_CalculateRelativePosition();
        }
    }
}

/**
 * @brief 解析 NMEA 语句
 */
bool GPS_ParseSentence(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }
    
    /* 验证校验和 */
    if (!GPS_VerifyChecksum(sentence)) {
        return false;
    }
    
    /* 根据语句类型解析 */
    if (strncmp(sentence + 3, "GGA", 3) == 0) {
        return GPS_ParseGGA(sentence);
    } else if (strncmp(sentence + 3, "RMC", 3) == 0) {
        return GPS_ParseRMC(sentence);
    } else if (strncmp(sentence + 3, "VTG", 3) == 0) {
        return GPS_ParseVTG(sentence);
    }
    
    return false;
}

/**
 * @brief 解析 GNGGA 语句
 * $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 */
bool GPS_ParseGGA(const char *sentence)
{
    char field[20];
    
    /* 字段 1: 时间 */
    if (GPS_GetField(sentence, 1, field, sizeof(field))) {
        if (strlen(field) >= 6) {
            g_gps_data.time.hour = (field[0] - '0') * 10 + (field[1] - '0');
            g_gps_data.time.minute = (field[2] - '0') * 10 + (field[3] - '0');
            g_gps_data.time.second = (field[4] - '0') * 10 + (field[5] - '0');
        }
    }
    
    /* 字段 2-3: 纬度 */
    if (GPS_GetField(sentence, 2, field, sizeof(field))) {
        char dir[2] = {0};
        GPS_GetField(sentence, 3, dir, sizeof(dir));
        g_gps_data.latitude = GPS_ParseCoordinate(field, dir[0]);
    }
    
    /* 字段 4-5: 经度 */
    if (GPS_GetField(sentence, 4, field, sizeof(field))) {
        char dir[2] = {0};
        GPS_GetField(sentence, 5, dir, sizeof(dir));
        g_gps_data.longitude = GPS_ParseCoordinate(field, dir[0]);
    }
    
    /* 字段 6: 定位质量 */
    if (GPS_GetField(sentence, 6, field, sizeof(field))) {
        g_gps_data.fix_quality = (GPS_FixQuality_t)GPS_ParseInt(field);
    }
    
    /* 字段 7: 卫星数量 */
    if (GPS_GetField(sentence, 7, field, sizeof(field))) {
        g_gps_data.satellites = GPS_ParseInt(field);
    }
    
    /* 字段 8: HDOP */
    if (GPS_GetField(sentence, 8, field, sizeof(field))) {
        g_gps_data.hdop = GPS_ParseFloat(field);
    }
    
    /* 字段 9: 海拔高度 */
    if (GPS_GetField(sentence, 9, field, sizeof(field))) {
        g_gps_data.altitude = GPS_ParseFloat(field);
    }
    
    /* 更新有效性 */
    g_gps_data.valid = (g_gps_data.fix_quality > GPS_FIX_NONE);
    g_gps_data.last_update_tick = HAL_GetTick();
    g_gps_data.update_count++;
    g_gps_new_data = true;
    
    return true;
}

/**
 * @brief 解析 GNRMC 语句
 * $GNRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
 */
bool GPS_ParseRMC(const char *sentence)
{
    char field[20];
    
    /* 字段 2: 状态 (A=有效, V=无效) */
    if (GPS_GetField(sentence, 2, field, sizeof(field))) {
        if (field[0] != 'A') {
            g_gps_data.valid = false;
            return false;
        }
    }
    
    /* 字段 7: 速度 (节) */
    if (GPS_GetField(sentence, 7, field, sizeof(field))) {
        g_gps_data.speed_knots = GPS_ParseFloat(field);
        g_gps_data.speed_kmh = g_gps_data.speed_knots * 1.852f;
    }
    
    /* 字段 8: 航向 */
    if (GPS_GetField(sentence, 8, field, sizeof(field))) {
        g_gps_data.course = GPS_ParseFloat(field);
    }
    
    /* 字段 9: 日期 */
    if (GPS_GetField(sentence, 9, field, sizeof(field))) {
        if (strlen(field) >= 6) {
            g_gps_data.date.day = (field[0] - '0') * 10 + (field[1] - '0');
            g_gps_data.date.month = (field[2] - '0') * 10 + (field[3] - '0');
            g_gps_data.date.year = 2000 + (field[4] - '0') * 10 + (field[5] - '0');
        }
    }
    
    return true;
}

/**
 * @brief 解析 GNVTG 语句
 */
bool GPS_ParseVTG(const char *sentence)
{
    char field[20];
    
    /* 字段 1: 真航向 */
    if (GPS_GetField(sentence, 1, field, sizeof(field))) {
        g_gps_data.course = GPS_ParseFloat(field);
    }
    
    /* 字段 7: 速度 (km/h) */
    if (GPS_GetField(sentence, 7, field, sizeof(field))) {
        g_gps_data.speed_kmh = GPS_ParseFloat(field);
        g_gps_data.speed_knots = g_gps_data.speed_kmh / 1.852f;
    }
    
    return true;
}

/**
 * @brief 设置起飞点
 */
void GPS_SetHome(void)
{
    if (g_gps_data.valid) {
        g_gps_data.home_lat = g_gps_data.latitude;
        g_gps_data.home_lon = g_gps_data.longitude;
        g_gps_data.home_alt = g_gps_data.altitude;
        g_gps_data.home_set = true;
    }
}

/**
 * @brief 清除起飞点
 */
void GPS_ClearHome(void)
{
    g_gps_data.home_set = false;
    g_gps_data.home_lat = 0;
    g_gps_data.home_lon = 0;
    g_gps_data.home_alt = 0;
}

/**
 * @brief 计算相对位置
 */
void GPS_CalculateRelativePosition(void)
{
    if (!g_gps_data.home_set || !g_gps_data.valid) {
        return;
    }
    
    /* 计算距离和方位 */
    g_gps_data.home_distance = GPS_CalculateDistance(
        g_gps_data.home_lat, g_gps_data.home_lon,
        g_gps_data.latitude, g_gps_data.longitude);
    
    g_gps_data.home_bearing = GPS_CalculateBearing(
        g_gps_data.latitude, g_gps_data.longitude,
        g_gps_data.home_lat, g_gps_data.home_lon);
    
    /* 计算相对 X/Y 位置 */
    float bearing_rad = GPS_CalculateBearing(
        g_gps_data.home_lat, g_gps_data.home_lon,
        g_gps_data.latitude, g_gps_data.longitude) * GPS_DEG_TO_RAD;
    
    g_gps_data.rel_x = g_gps_data.home_distance * sinf(bearing_rad);  /* 东向 */
    g_gps_data.rel_y = g_gps_data.home_distance * cosf(bearing_rad);  /* 北向 */
}

/**
 * @brief 计算两点间距离 (Haversine 公式)
 */
float GPS_CalculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * GPS_DEG_TO_RAD;
    double lat2_rad = lat2 * GPS_DEG_TO_RAD;
    double dlat = (lat2 - lat1) * GPS_DEG_TO_RAD;
    double dlon = (lon2 - lon1) * GPS_DEG_TO_RAD;
    
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return (float)(GPS_EARTH_RADIUS * c);
}

/**
 * @brief 计算两点间方位角
 */
float GPS_CalculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * GPS_DEG_TO_RAD;
    double lat2_rad = lat2 * GPS_DEG_TO_RAD;
    double dlon = (lon2 - lon1) * GPS_DEG_TO_RAD;
    
    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    float bearing = (float)(atan2(y, x) * GPS_RAD_TO_DEG);
    
    /* 转换为 0-360 度 */
    if (bearing < 0) {
        bearing += 360.0f;
    }
    
    return bearing;
}

/**
 * @brief 检查 GPS 是否有效
 */
bool GPS_IsValid(void)
{
    return g_gps_data.valid;
}

/**
 * @brief 检查是否有 3D 定位
 */
bool GPS_Has3DFix(void)
{
    return (g_gps_data.valid && g_gps_data.satellites >= 4);
}

/**
 * @brief 获取卫星数量
 */
uint8_t GPS_GetSatellites(void)
{
    return g_gps_data.satellites;
}

/**
 * @brief UART 空闲中断回调
 */
void GPS_UART_IdleCallback(void)
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        
        /* 停止 DMA */
        HAL_UART_DMAStop(&huart5);
        
        /* 计算接收长度 */
        uint32_t rx_len = GPS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
        
        /* 解析接收到的数据 */
        for (uint32_t i = 0; i < rx_len; i++) {
            char c = gps_rx_buffer[i];
            
            if (c == '$') {
                /* 新语句开始 */
                gps_sentence_index = 0;
            }
            
            if (gps_sentence_index < GPS_SENTENCE_MAX_LEN - 1) {
                gps_sentence_buffer[gps_sentence_index++] = c;
            }
            
            if (c == '\n') {
                /* 语句结束 */
                gps_sentence_buffer[gps_sentence_index] = '\0';
                GPS_ParseSentence(gps_sentence_buffer);
                gps_sentence_index = 0;
            }
        }
        
        /* 重新启动 DMA */
        HAL_UART_Receive_DMA(&huart5, gps_rx_buffer, GPS_RX_BUFFER_SIZE);
    }
}

/**
 * @brief DMA 接收完成回调
 */
void GPS_DMA_RxCpltCallback(void)
{
    /* 循环模式下不需要重启 */
}

/**
 * @brief 验证 NMEA 校验和
 */
bool GPS_VerifyChecksum(const char *sentence)
{
    if (sentence[0] != '$') {
        return false;
    }
    
    uint8_t checksum = 0;
    const char *p = sentence + 1;
    
    /* 计算校验和 ($ 和 * 之间的异或) */
    while (*p && *p != '*') {
        checksum ^= *p++;
    }
    
    /* 比较校验和 */
    if (*p == '*' && *(p + 1) && *(p + 2)) {
        char hex[3] = {*(p + 1), *(p + 2), '\0'};
        uint8_t expected = (uint8_t)strtol(hex, NULL, 16);
        return (checksum == expected);
    }
    
    return false;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 解析坐标
 */
static double GPS_ParseCoordinate(const char *str, char direction)
{
    if (str == NULL || strlen(str) < 4) {
        return 0.0;
    }
    
    double value = atof(str);
    int degrees = (int)(value / 100);
    double minutes = value - degrees * 100;
    double result = degrees + minutes / 60.0;
    
    if (direction == 'S' || direction == 'W') {
        result = -result;
    }
    
    return result;
}

/**
 * @brief 解析浮点数
 */
static float GPS_ParseFloat(const char *str)
{
    if (str == NULL || strlen(str) == 0) {
        return 0.0f;
    }
    return (float)atof(str);
}

/**
 * @brief 解析整数
 */
static int GPS_ParseInt(const char *str)
{
    if (str == NULL || strlen(str) == 0) {
        return 0;
    }
    return atoi(str);
}

/**
 * @brief 获取 NMEA 字段
 */
static char* GPS_GetField(const char *sentence, uint8_t field_num, char *buffer, uint8_t buf_size)
{
    if (sentence == NULL || buffer == NULL) {
        return NULL;
    }
    
    const char *p = sentence;
    uint8_t current_field = 0;
    uint8_t buf_index = 0;
    
    buffer[0] = '\0';
    
    while (*p) {
        if (*p == ',') {
            current_field++;
            if (current_field > field_num) {
                break;
            }
        } else if (current_field == field_num && buf_index < buf_size - 1) {
            if (*p != '*' && *p != '\r' && *p != '\n') {
                buffer[buf_index++] = *p;
            }
        }
        p++;
    }
    
    buffer[buf_index] = '\0';
    return buffer;
}