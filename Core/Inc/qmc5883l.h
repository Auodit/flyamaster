#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "main.h"
#include "mahony.h"  // 需要 Mahony 结构体进行倾斜补偿

// ==================== QMC5883L 寄存器地址 ====================
#define QMC5883L_ADDR       (0x0D << 1)  // 7-bit: 0x0D, 写: 0x1A, 读: 0x1B

// 数据寄存器
#define QMC5883L_REG_X_LSB  0x00  // X 轴低字节
#define QMC5883L_REG_X_MSB  0x01  // X 轴高字节
#define QMC5883L_REG_Y_LSB  0x02  // Y 轴低字节
#define QMC5883L_REG_Y_MSB  0x03  // Y 轴高字节
#define QMC5883L_REG_Z_LSB  0x04  // Z 轴低字节
#define QMC5883L_REG_Z_MSB  0x05  // Z 轴高字节

// 状态与控制寄存器
#define QMC5883L_REG_STATUS 0x06  // 状态寄存器 (DRDY, OVL, DOR)
#define QMC5883L_REG_TOUT_LSB 0x07  // 温度输出低字节
#define QMC5883L_REG_TOUT_MSB 0x08  // 温度输出高字节
#define QMC5883L_REG_CONTROL1 0x09  // 控制寄存器 1
#define QMC5883L_REG_CONTROL2 0x0A  // 控制寄存器 2
#define QMC5883L_REG_SET_RESET 0x0B  // SET/RESET 周期寄存器
#define QMC5883L_REG_CHIP_ID  0x0D  // 芯片 ID (固定值 0xFF)

// 控制寄存器 1 配置位
#define QMC5883L_MODE_STANDBY   0x00  // 待机模式
#define QMC5883L_MODE_CONTINUOUS 0x01  // 连续测量模式

#define QMC5883L_ODR_10HZ    0x00  // 10Hz
#define QMC5883L_ODR_50HZ    0x04  // 50Hz ← 我们使用这个
#define QMC5883L_ODR_100HZ   0x08  // 100Hz
#define QMC5883L_ODR_200HZ   0x0C  // 200Hz

#define QMC5883L_RNG_2G      0x00  // ±2 Gauss (灵敏度高)
#define QMC5883L_RNG_8G      0x10  // ±8 Gauss

#define QMC5883L_OSR_512     0x00  // 过采样 512 (噪声低，速度慢)
#define QMC5883L_OSR_256     0x40  // 过采样 256
#define QMC5883L_OSR_128     0x80  // 过采样 128
#define QMC5883L_OSR_64      0xC0  // 过采样 64 (速度快)

// 状态寄存器位
#define QMC5883L_STATUS_DRDY 0x01  // 数据就绪
#define QMC5883L_STATUS_OVL  0x02  // 数据溢出
#define QMC5883L_STATUS_DOR  0x04  // 数据跳过

// ==================== 磁偏角配置 ====================
// 磁偏角 (Magnetic Declination)，单位：度
// 北京地区约 -6.5°，上海约 -5.0°，用户需根据当地情况修改
#ifndef MAG_DECLINATION
#define MAG_DECLINATION  -3.5f // 默认 0.0°，用户可在编译时定义 -DMAG_DECLINATION=-6.5f
#endif

// ==================== 数据结构 ====================

/**
 * @brief QMC5883L 校准参数
 */
typedef struct {
    // 硬磁偏移 (8字校准法获得)
    float offset_x, offset_y, offset_z;
    
    // 软磁校准系数 (椭球拟合，可选，默认为 1.0)
    float scale_x, scale_y, scale_z;
    
    // 校准状态标志
    uint8_t calibrated;
} QMC5883L_Calibration_t;

/**
 * @brief QMC5883L 传感器数据
 */
typedef struct {
    // 原始磁场数据 (ADC 读数)
    int16_t mag_x_raw, mag_y_raw, mag_z_raw;
    
    // 校准后的磁场数据 (单位: Gauss)
    float mag_x, mag_y, mag_z;
    
    // 倾斜补偿后的水平磁场
    float mag_x_h, mag_y_h;
    
    // 计算得到的航向角
    float heading;          // 原始航向角 (0~360°)
    float heading_filtered; // 互补滤波后的航向角
    
    // 磁干扰检测
    float mag_strength;     // 磁场强度 (Gauss)
    uint8_t interference_detected;  // 干扰标志 (1=干扰)
    
    // 状态标志
    uint8_t data_ready;     // 数据就绪
    uint8_t initialized;    // 初始化完成
    uint32_t last_update;   // 最后更新时间戳
} QMC5883L_t;

// ==================== 全局变量 ====================
extern volatile QMC5883L_t QMC_Data;
extern volatile QMC5883L_Calibration_t QMC_Calibration;

// ==================== 函数声明 ====================

/**
 * @brief 初始化 QMC5883L
 * @param hi2c I2C 句柄 (I2C1)
 * @return 0: 成功, 1: 失败
 */
uint8_t QMC5883L_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取原始磁场数据
 * @param hi2c I2C 句柄
 * @return 0: 成功, 1: 失败
 */
uint8_t QMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c);

/**
 * @brief 计算航向角 (带倾斜补偿)
 * @param mahony Mahony 姿态结构体指针 (用于获取 Roll/Pitch)
 * @return 航向角 (0~360°)
 */
float QMC5883L_CalculateHeading(Mahony_TypeDef *mahony);

/**
 * @brief 8字校准法 - 启动校准
 * @note 用户需要在 20 秒内以 8 字形旋转飞机，记录磁场极值
 */
void QMC5883L_Calibrate(void);

/**
 * @brief 8字校准法 - 更新校准数据 (需周期调用)
 * @return 1: 校准完成, 0: 校准中
 */
uint8_t QMC5883L_CalibrateUpdate(void);

/**
 * @brief 应用校准参数
 */
void QMC5883L_ApplyCalibration(void);

/**
 * @brief 磁干扰检测
 * @return 1: 检测到干扰, 0: 正常
 */
uint8_t QMC5883L_CheckInterference(void);

/**
 * @brief 互补滤波融合 (融合陀螺仪 Yaw 和磁力计航向)
 * @param gyro_yaw_deg 陀螺仪 Yaw 角速度积分值 (度)
 * @param mag_heading 磁力计航向角 (度)
 * @param dt 时间间隔 (秒)
 * @param alpha 融合系数 (0.98 = 98% 陀螺仪, 2% 磁力计)
 * @return 融合后的航向角 (度)
 */
float QMC5883L_ComplementaryFilter(float gyro_yaw_deg, float mag_heading, float dt, float alpha);

#endif // __QMC5883L_H





