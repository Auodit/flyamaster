# 🔗 接口定义与命名规范 (Interface Definition)

**创建时间**: 2026-01-16  
**状态**: 活跃维护中  
**作用**: 防止重复定义、确保命名一致性、统一接口规范

---

## 📋 文档说明

本文档是飞控项目的 **"代码字典"**,记录所有结构体、全局变量、函数原型和宏定义。  
**强制规则**: 每次编写代码前,必须先查阅此文档,严禁臆造变量名或重复定义!

---

## 1. 全局变量命名规范 (Naming Convention)

| 前缀 | 用途 | 示例 | 作用域 |
|:---:|:---|:---|:---|
| `g_` | 通用全局变量 | `g_motor_pwm[4]`, `g_roll` | 跨文件 |
| `Module_` | 模块专属变量 | `SBUS_Data`, `GPS_Data`, `Flow_Data` | 模块内部+外部访问 |
| `volatile` | RTOS共享/ISR访问 | `volatile SPL06_TypeDef SPL06_Data` | 多任务安全 |
| `static` | 文件内私有 | `static uint8_t rx_buffer[256]` | 单文件 |

**规则**:
1. 所有 RTOS 任务之间共享的变量**必须**加 `volatile` 修饰符
2. ISR (中断服务例程) 修改的变量**必须**加 `volatile`
3. 新建全局变量时,优先使用 `Module_` 前缀,如 `Flow_Data`, `QMC_Data`
4. 禁止使用无意义缩写 (如 `temp`, `tmp`, `dat`),必须语义明确

---

## 2. 数据结构定义 (Structure Definitions)

### 2.1 姿态解算模块 (Mahony)

**定义位置**: [`mahony.h:4`](../FLYAMASTER/Core/Inc/mahony.h:4)

```c
typedef struct {
    float q0, q1, q2, q3;  // 四元数 (w, x, y, z)
    float integralFBx, integralFBy, integralFBz; // 积分误差
    float Kp, Ki;  // 算法参数
} Mahony_TypeDef;
```

**成员说明**:
- `q0~q3`: 四元数分量,描述飞机姿态
- `integralFBx/y/z`: 积分误差补偿项 (用于修正陀螺仪零漂)
- `Kp`: 比例增益 (推荐 2.0~5.0)
- `Ki`: 积分增益 (推荐 0.0~0.1)

---

### 2.2 PID 控制器

**定义位置**: [`pid.h:6`](../FLYAMASTER/Core/Inc/pid.h:6)

```c
typedef struct {
    float Kp, Ki, Kd;
    float max_out;      // 输出限幅
    float max_i;        // 积分限幅
    float integral;     // 积分累加值
    float prev_error;   // 上一次误差
    float prev_measure; // 上一次测量值 (用于微分先行)
} PID_TypeDef;
```

**函数接口**:
```c
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i);
void PID_Reset(PID_TypeDef *pid);
float PID_Calculate(PID_TypeDef *pid, float target, float measured, float dt);
```

---

### 2.3 遥控器 SBUS 模块

**定义位置**: [`sbus.h:28`](../FLYAMASTER/Core/Inc/sbus.h:28)

```c
typedef struct {
    uint16_t channels[SBUS_CHANNEL_NUMBER]; // 16个通道值 (范围 172~1811)
    uint8_t failsafe; // 失控保护标志 (0:正常, 1:丢帧, 2:失控)
    uint8_t frame_lost;
} SBUS_Data_t;
```

**全局变量**:
```c
extern SBUS_Data_t SBUS_Data;
extern uint8_t SBUS_RxBuffer[SBUS_FRAME_SIZE]; // 25字节
```

**函数接口**:
```c
void SBUS_Init(UART_HandleTypeDef *huart);
void SBUS_Parse(uint8_t *buffer);
uint8_t SBUS_IsConnected(void);
float SBUS_GetThrottle(void);    // 返回 0.0 ~ 1.0
float SBUS_GetRoll(void);        // 返回 -1.0 ~ 1.0
float SBUS_GetPitch(void);       // 返回 -1.0 ~ 1.0
float SBUS_GetYaw(void);         // 返回 -1.0 ~ 1.0
```

---

### 2.4 光流模块

**定义位置**: [`optical_flow.h:14`](../FLYAMASTER/Core/Inc/optical_flow.h:14)

```c
typedef struct {
    int16_t flow_x;         // X轴像素位移
    int16_t flow_y;         // Y轴像素位移
    uint16_t distance;      // 激光测距高度 (mm)
    uint8_t quality;        // 质量 (0~255)
    uint8_t valid;          // 数据有效标志
    uint32_t last_update;   // 最后更新时间
} OpticalFlow_t;
```

**全局变量**:
```c
extern OpticalFlow_t Flow_Data;
```

**函数接口**:
```c
void OpticalFlow_Init(UART_HandleTypeDef *huart);
void OpticalFlow_Parse(uint8_t *buf, uint16_t len);
uint8_t OpticalFlow_GetVelocity(float *vx, float *vy);
float OpticalFlow_GetHeight(void);
```

---

### 2.5 GPS 模块

**定义位置**: [`gps.h:25`](../FLYAMASTER/Core/Inc/gps.h:25)

```c
typedef struct {
    // 定位状态
    uint8_t fix_type;           // 0=无效, 1=GPS定位, 2=DGPS定位
    uint8_t satellites;         // 可见卫星数量
    float hdop;                 // 水平精度因子 (<1.5优秀, <2.5良好, >5差)
    
    // 地理坐标 (小数度格式)
    double latitude;            // 纬度 (°, -90 ~ +90)
    double longitude;           // 经度 (°, -180 ~ +180)
    float altitude;             // 海拔高度 (m, WGS84)
    
    // XY 坐标系 (相对于上电点/Home, 单位:米)
    float pos_x;                // 东向距离 (m, East+)
    float pos_y;                // 北向距离 (m, North+)
    double home_lat;            // Home 点纬度 (°)
    double home_lon;            // Home 点经度 (°)
    uint8_t home_set;           // Home 点已设置标志
    
    // 运动数据
    float speed_kmh;            // 地速 (km/h)
    float course;               // 航向角 (0-360°, 正北=0°)
    
    // 数据质量
    uint8_t valid;              // 数据有效标志
    uint32_t last_update;       // 最后更新时间戳 (ms)
    uint32_t checksum_error_count;  // 校验失败计数 (调试用)
} GPS_Data_t;
```

**全局变量**:
```c
extern GPS_Data_t GPS_Data;
```

**函数接口**:
```c
void GPS_Init(UART_HandleTypeDef *huart);
uint8_t GPS_Validate_Checksum(char* line);
void GPS_Parse_NMEA(char* line);
void GPS_Process_Data(uint8_t* buffer, uint16_t length);
void GPS_To_XY(double lat, double lon, double origin_lat, double origin_lon, float* x, float* y);
void GPS_Set_Home(void);
uint8_t GPS_IsValid(void);
```

---

### 2.6 磁力计 QMC5883L

**定义位置**: [`qmc5883l.h:75`](../FLYAMASTER/Core/Inc/qmc5883l.h:75)

```c
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
```

**全局变量**:
```c
extern volatile QMC5883L_t QMC_Data;
extern volatile QMC5883L_Calibration_t QMC_Calibration;
```

**函数接口**:
```c
uint8_t QMC5883L_Init(I2C_HandleTypeDef *hi2c);
uint8_t QMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c);
float QMC5883L_CalculateHeading(Mahony_TypeDef *mahony);
void QMC5883L_Calibrate(void);
uint8_t QMC5883L_CalibrateUpdate(void);
void QMC5883L_ApplyCalibration(void);

// v1.2.0 新增: 电流补偿接口
void QMC5883L_CompensateCurrent(float throttle);
void QMC5883L_SetCurrentCompensation(float kx, float ky, float kz);
```

**电流补偿说明** (v1.2.0 新增):
- 公式: `Mag_corrected = Mag_raw - Throttle × K`
- 用于补偿电机电流产生的磁场干扰
- 标定方法: 飞机静止，记录不同油门下的磁场读数，计算 K 系数

---

### 2.7 气压计 SPL06

**定义位置**: [`spl06.h:77`](../FLYAMASTER/Core/Inc/spl06.h:77)

```c
typedef struct {
    // 校准系数 (从芯片读取)
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    
    // 测量结果
    float pressure;         // 气压 (Pa)
    float temperature;      // 温度 (°C)
    float altitude;         // 海拔高度 (m)
    
    // 参考值 (用于相对高度计算)
    float pressure_ref;     // 参考气压 (上电时的气压)
    float altitude_ref;     // 参考高度 (设为 0)
    
    // 状态标志
    uint8_t initialized;    // 初始化完成标志
    uint8_t data_ready;     // 数据就绪标志
    uint32_t last_update;   // 最后更新时间 (ms)
} SPL06_TypeDef;
```

**全局变量**:
```c
extern volatile SPL06_TypeDef SPL06_Data; // Risk #066: 添加 volatile
```

**函数接口**:
```c
uint8_t SPL06_Init(I2C_HandleTypeDef *hi2c);
uint8_t SPL06_ReadRawData(I2C_HandleTypeDef *hi2c);
float SPL06_PressureToAltitude(float pressure, float pressure_ref);
float SPL06_GetAltitude(volatile SPL06_TypeDef *spl06);
void SPL06_SetReference(volatile SPL06_TypeDef *spl06);
```

---

### 2.8 Flash 参数管理

**定义位置**: [`flash_params.h:70`](../FLYAMASTER/Core/Inc/flash_params.h:70)

```c
typedef struct __attribute__((packed)) {
    // Header (16 字节)
    uint32_t magic;       // 魔数: 0x464C4159 ("FLAY")
    uint16_t version;     // 版本号 (用于双缓冲区选择)
    uint16_t crc16;       // CRC16 校验值
    uint32_t total_size;  // 数据总大小 (用于校验)
    uint32_t reserved1;   // 保留
    
    // PID 参数 (120 字节)
    PID_Params_t pid_roll_angle;
    PID_Params_t pid_roll_rate;
    PID_Params_t pid_pitch_angle;
    PID_Params_t pid_pitch_rate;
    PID_Params_t pid_yaw_angle;
    PID_Params_t pid_yaw_rate;
    PID_Params_t pid_pos_x;
    PID_Params_t pid_pos_y;
    PID_Params_t pid_alt_pos;
    PID_Params_t pid_alt_vel;
    
    // 校准数据 (76 字节)
    float gyro_offset[3];
    float accel_offset[3];
    float mag_calib[12];
    float pressure_ref;
    
    // 飞行配置 (24 字节)
    float hover_throttle;
    uint8_t channel_map[16];
    uint8_t flight_mode_memory;
    uint8_t reserved2[7];
    
    // 系统信息 (16 字节)
    uint32_t total_flight_time;
    uint8_t fw_version[4];
    uint32_t save_count;
    uint32_t reserved3;
} FlashParams_t;
```

**全局变量**:
```c
extern FlashParams_t g_flash_params;  // 全局参数实例
extern uint8_t g_params_dirty;        // 参数脏标志 (需要保存)
```

**函数接口**:
```c
uint8_t FlashParams_Init(void);
uint8_t FlashParams_Load(void);
uint8_t FlashParams_Save(void);
void FlashParams_RestoreDefaults(void);
void FlashParams_ApplyToPID(void);
void FlashParams_ReadFromPID(void);
void FlashParams_PeriodicSave(void);
```

---

### 2.9 匿名上位机 V7 协议

**定义位置**: [`ano_v7.h:43`](../FLYAMASTER/Core/Inc/ano_v7.h:43)

```c
// 姿态数据帧 (0x01) - 18字节
typedef struct {
    int16_t roll;      // 横滚角 ×100 (度)
    int16_t pitch;     // 俯仰角 ×100 (度)
    int16_t yaw;       // 航向角 ×100 (度)
    int32_t alt;       // 高度 ×100 (cm)
    int16_t vx;        // X速度 ×100 (m/s)
    int16_t vy;        // Y速度 ×100 (m/s)
    int16_t vz;        // Z速度 ×100 (m/s)
} __attribute__((packed)) AnoV7_Attitude_t;

// GPS数据帧 (0x09) - 22字节
typedef struct {
    int32_t  latitude;   // 纬度 ×10^7 (度)
    int32_t  longitude;  // 经度 ×10^7 (度)
    int32_t  altitude;   // 海拔高度 ×100 (cm, WGS84)
    uint16_t speed;      // 地速 ×100 (cm/s)
    uint16_t course;     // 航向角 ×100 (度, 0-36000)
    uint8_t  satellites; // 可见卫星数量
    uint8_t  fix_type;   // 定位类型 (0=无效, 1=GPS, 2=DGPS)
    uint16_t hdop;       // HDOP ×100 (水平精度因子)
    uint8_t  valid;      // 数据有效标志
    uint8_t  reserved;   // 保留字节(对齐)
} __attribute__((packed)) AnoV7_GPS_t;
```

**函数接口**:
```c
void AnoV7_Init(UART_HandleTypeDef *huart);
void AnoV7_SendAttitude(void);
void AnoV7_SendSensor(void);
void AnoV7_SendMotor(void);
void AnoV7_SendStatus(void);
void AnoV7_SendGPS(void);
void AnoV7_Parse(uint8_t *buf, uint16_t len);
void AnoV7_RegisterPIDCallback(AnoV7_PID_Callback callback);
```

---

### 2.10 滤波器模块

**定义位置**: [`filter.h:7`](../FLYAMASTER/Core/Inc/filter.h:7)

```c
typedef struct {
    float b0, b1, b2;  // 前馈系数
    float a1, a2;      // 反馈系数
    float x1, x2;      // 输入历史
    float y1, y2;      // 输出历史
} LPF2_TypeDef;
```

**函数接口**:
```c
void LPF2_Init(LPF2_TypeDef *lpf, float sample_freq, float cutoff_freq);
float LPF2_Apply(LPF2_TypeDef *lpf, float input);
```

---

### 2.12 RingBuffer 异步日志模块 (v1.2.0 新增)

**定义位置**: [`ring_buffer.h:15`](../FLYAMASTER/Core/Inc/ring_buffer.h:15)

```c
#define RING_BUFFER_SIZE 512  // 缓冲区大小

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;      // 写入位置 (生产者)
    volatile uint16_t tail;      // 读取位置 (DMA 消费者)
    volatile uint8_t dma_busy;   // DMA 忙标志
    UART_HandleTypeDef *huart;   // 关联的 UART 句柄
} RingBuffer_t;
```

**函数接口**:
```c
void RingBuffer_Init(UART_HandleTypeDef *huart);
uint16_t RingBuffer_Write(const uint8_t *data, uint16_t len);
void RingBuffer_StartDMA(void);
void RingBuffer_DMA_TxCpltCallback(void);
int Log_Printf(const char *format, ...);  // 非阻塞 printf 替代
```

**使用说明**:
- 替代阻塞式 `printf`，解决 Issue #111 (高频任务中的控制回路抖动)
- 使用 DMA2_Stream7 (USART1_TX) 进行异步传输
- 在 `HAL_UART_TxCpltCallback` 中调用 `RingBuffer_DMA_TxCpltCallback()`

---

### 2.11 OSD 模块 (AT7456E)

**定义位置**: [`at7456e.h:20`](../FLYAMASTER/Core/Inc/at7456e.h:20)

```c
typedef struct {
    uint8_t initialized;      // 初始化完成标志
    uint8_t video_detected;   // 视频信号检测标志
    uint8_t video_standard;   // 视频制式 (1=PAL, 2=NTSC)
    uint32_t last_update;     // 最后更新时间
} OSD_Data_t;
```

**全局变量**:
```c
extern OSD_Data_t OSD_Data;
```

**函数接口**:
```c
uint8_t AT7456E_Init(SPI_HandleTypeDef *hspi);
void AT7456E_Update(void);
void AT7456E_Clear(void);
void AT7456E_WriteString(uint8_t row, uint8_t col, const char *str);
```

---

## 3. 宏定义 (Macro Definitions)

### 3.1 电机控制

**定义位置**: [`motor.h:6`](../FLYAMASTER/Core/Inc/motor.h:6)

```c
#define MOTOR_MIN_PWM 1000
#define MOTOR_MAX_PWM 2000
#define MOTOR_IDLE_PWM 1100 // 怠速 (AirMode)
```

---

### 3.2 光流参数

**定义位置**: [`optical_flow.h:6`](../FLYAMASTER/Core/Inc/optical_flow.h:6)

```c
#define FLOW_SCALE_FACTOR   0.01f    // 缩放系数 (需根据模块校准)
#define FLOW_MIN_HEIGHT     0.05f    // 最小有效高度 5cm
#define FLOW_MAX_HEIGHT     3.0f     // 最大有效高度 3m
#define FLOW_MIN_QUALITY    30       // 最小有效质量
```

---

### 3.3 SBUS 协议

**定义位置**: [`sbus.h:6`](../FLYAMASTER/Core/Inc/sbus.h:6)

```c
#define SBUS_FRAME_SIZE 25
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_MIN_VALUE   172
#define SBUS_MID_VALUE   992
#define SBUS_MAX_VALUE   1811

// 通道映射定义
#define CH_ROLL     0   // 横滚 (Aileron)
#define CH_PITCH    1   // 俯仰 (Elevator)
#define CH_THROTTLE 2   // 油门 (Throttle)
#define CH_YAW      3   // 偏航 (Rudder)
#define CH_ARM      4   // 解锁开关
#define CH_MODE     5   // 飞行模式开关
```

---

### 3.4 GPS 参数

**定义位置**: [`gps.h:7`](../FLYAMASTER/Core/Inc/gps.h:7)

```c
#define GPS_MIN_SATELLITES      4       // 最少卫星数
#define GPS_MAX_HDOP            2.5f    // 最大水平精度因子
#define GPS_TIMEOUT_MS          1000    // GPS 数据超时 (ms)
#define GPS_RX_BUF_SIZE        256      // NMEA 句子长度 <80字符
#define EARTH_RADIUS_M         6378137.0 // 地球半径 (米)
```

---

### 3.5 磁力计参数

**定义位置**: [`qmc5883l.h:8`](../FLYAMASTER/Core/Inc/qmc5883l.h:8)

```c
#define QMC5883L_ADDR       (0x0D << 1)  // 7-bit: 0x0D
#define MAG_DECLINATION     -3.5f        // 磁偏角 (度, 根据当地调整)
```

---

### 3.6 气压计参数

**定义位置**: [`spl06.h:18`](../FLYAMASTER/Core/Inc/spl06.h:18)

```c
#define SPL06_ADDR              (0x76 << 1)  // SDO=GND: 0x76
#define SPL06_ADDR_ALT          (0x77 << 1)  // SDO=VDD: 0x77
#define SPL06_PRODUCT_ID        0x10
```

---

### 3.7 Flash 参数

**定义位置**: [`flash_params.h:113`](../FLYAMASTER/Core/Inc/flash_params.h:113)

```c
#define FLASH_PARAMS_ADDR_A    0x000000  // Block A 地址
#define FLASH_PARAMS_ADDR_B    0x000200  // Block B 地址 (512 字节偏移)
#define FLASH_PARAMS_MAGIC     0x464C4159  // "FLAY"
#define FLASH_PARAMS_VERSION   1           // 当前版本号
#define FLASH_SAVE_DELAY_MS    2000        // 修改后延迟 2 秒保存
```

---

### 3.8 匿名上位机协议

**定义位置**: [`ano_v7.h:20`](../FLYAMASTER/Core/Inc/ano_v7.h:20)

```c
#define ANO_V7_FRAME_HEADER1    0xAA    // 帧头1
#define ANO_V7_FRAME_HEADER2    0xAF    // 帧头2

// 功能字定义
#define ANO_V7_FUNC_ATTITUDE    0x01    // 姿态数据帧 (FC→PC)
#define ANO_V7_FUNC_SENSOR      0x02    // 传感器数据帧 (FC→PC)
#define ANO_V7_FUNC_MOTOR       0x04    // 电机PWM帧 (FC→PC)
#define ANO_V7_FUNC_STATUS      0x05    // 状态数据帧 (FC→PC)
#define ANO_V7_FUNC_GPS         0x09    // GPS数据帧 (FC→PC)
#define ANO_V7_FUNC_PID_READ    0x10    // 读取PID (PC→FC)
#define ANO_V7_FUNC_PID_WRITE   0x11    // 写入PID (PC→FC)
#define ANO_V7_FUNC_PID_DATA    0x12    // PID数据帧 (FC→PC)

// 错误码定义 (v1.2.0 新增, 位掩码格式)
#define ANO_ERROR_NONE          0x0000  // 无错误
#define ANO_ERROR_IMU_FAIL      0x0001  // IMU 初始化/通信失败
#define ANO_ERROR_BARO_FAIL     0x0002  // 气压计初始化/通信失败
#define ANO_ERROR_MAG_FAIL      0x0004  // 磁力计初始化/通信失败
#define ANO_ERROR_GPS_TIMEOUT   0x0008  // GPS 数据超时 (>3s)
#define ANO_ERROR_RC_LOST       0x0010  // 遥控器信号丢失
#define ANO_ERROR_LOW_BATTERY   0x0020  // 电池电压过低 (<10.5V)
#define ANO_ERROR_MOTOR_LOCK    0x0040  // 电机锁定/堵转
#define ANO_ERROR_FLASH_FAIL    0x0080  // Flash 读写失败
#define ANO_ERROR_SENSOR_CALIB  0x0100  // 传感器需要校准
#define ANO_ERROR_ATTITUDE_ERR  0x0200  // 姿态解算异常 (>60°)
#define ANO_ERROR_FAILSAFE      0x0400  // Failsafe 触发
#define ANO_ERROR_ARMING_BLOCK  0x0800  // 解锁被阻止 (安全检查未通过)
```

---

### 3.9 OSD 参数

**定义位置**: [`at7456e.h:10`](../FLYAMASTER/Core/Inc/at7456e.h:10)

```c
#define OSD_MAX_ROWS_PAL    16
#define OSD_MAX_COLS_PAL    30
#define OSD_CHAR_SPACE      0x00
```

---

## 4. 外设句柄命名规范 (Peripheral Handles) - STM32F405RGT6

| 外设 | 句柄名称 | 引脚分配 | 用途 | DMA |
|:---:|:---|:---|:---|:---|
| **USART1** | `huart1` | PA9/PA10 | 树莓派/日志 (921600 baud) | DMA2_Stream7 TX, DMA2_Stream2 RX |
| **USART2** | `huart2` | PA2/PA3 | GPS 模块 (115200 baud) | DMA1_Stream5 RX (Circular) |
| **UART4** | `huart4` | PC10/PC11 | SBUS (100k, Even, 2Stop) + SmartAudio | DMA1_Stream2 RX, DMA1_Stream4 TX |
| **UART5** | `huart5` | PC12/PD2 | 光流模块 / 匿名上位机 (115200 baud) | DMA1_Stream7 TX |
| **I2C1** | `hi2c1` | PB6/PB7 | MPU6050 + SPL06 (400kHz) | DMA1_Stream0 RX, DMA1_Stream6 TX |
| **I2C2** | `hi2c2` | PB10/PB11 | QMC5883L 磁力计 (100kHz) | DMA1_Stream3 RX |
| **SPI1** | `hspi1` | PA5/PA6/PA7 | W25Q128 + AT7456E (10.5MHz) | DMA2_Stream0 RX, DMA2_Stream3 TX |
| **TIM1** | `htim1` | PA8 (CH1) | WS2812 RGB 灯 (PWM) | DMA2_Stream1 |
| **TIM4** | `htim4` | - | HAL 时基 (Timebase Source) | - |
| **TIM8** | `htim8` | PC6-PC9 (CH1-4) | 电机 PWM (400Hz) | - |
| **ADC1** | `hadc1` | PC0 (IN10), PC5 (IN15) | 电池电压+电流 | DMA2_Stream4 (Circular) |
| **USB_OTG_FS** | `hpcd_USB_OTG_FS` | PA11/PA12 | CDC 虚拟串口 | - |
| **GPIO** | `Power_Key` | PA0-WKUP | 软开关按键检测 (Input, Pull-down) | - |
| **GPIO** | `Power_Hold` | PC1 | 电源锁存控制 (Output) | - |
| **GPIO** | `BUZZER` | PC2 | 蜂鸣器 (Output) | - |
| **GPIO** | `LED_BLUE` | PC3 | 蓝色 LED (Output) | - |
| **GPIO** | `LED_RED` | PC4 | 红色 LED (Output) | - |
| **GPIO** | `USB_DETECT` | PB9 | USB 插入检测 (Input, Pull-down) | - |
| **GPIO** | `FLASH_CS` | PA4 | Flash 片选 (Output, High) | - |
| **GPIO** | `OSD_CS` | PB12 | OSD 片选 (Output, High) | - |

**规则**:
- **严禁臆造外设句柄**! 例如文档里只定义了 `huart4`,代码里不能写 `huart6`
- 如需新增外设,必须先更新 [`08_hardware_design.md`](08_hardware_design.md)
- **注意**: F405 没有 UART6，GPS 使用 USART2，SBUS 使用 UART4

---

## 5. 函数命名规范 (Function Naming)

| 模块 | 命名格式 | 示例 |
|:---:|:---|:---|
| 初始化函数 | `Module_Init()` | `GPS_Init()`, `QMC5883L_Init()` |
| 数据读取 | `Module_Read*()` | `SPL06_ReadRawData()`, `QMC5883L_ReadRaw()` |
| 数据解析 | `Module_Parse()` | `GPS_Parse_NMEA()`, `SBUS_Parse()` |
| 数据获取 | `Module_Get*()` | `SBUS_GetThrottle()`, `Flow_GetVelocity()` |
| 校准函数 | `Module_Calibrate()` | `QMC5883L_Calibrate()`, `SPL06_SetReference()` |
| 计算函数 | `Module_Calculate*()` | `QMC5883L_CalculateHeading()`, `PID_Calculate()` |
| 发送函数 | `Module_Send*()` | `AnoV7_SendAttitude()`, `AnoV7_SendGPS()` |

---

### 2.13 调试工具模块 (v1.2.0 新增)

**定义位置**: [`debug_tools.h:15`](../FLYAMASTER/Core/Inc/debug_tools.h:15)

```c
// I2C 扫描结果结构体
typedef struct {
    uint8_t found_count;        // 发现的设备数量
    uint8_t addresses[16];      // 发现的设备地址 (最多 16 个)
} I2C_ScanResult_t;

// 磁力计电流补偿标定状态
typedef enum {
    MAG_CALIB_IDLE = 0,         // 空闲状态
    MAG_CALIB_COLLECTING_LOW,   // 采集低油门数据
    MAG_CALIB_COLLECTING_HIGH,  // 采集高油门数据
    MAG_CALIB_CALCULATING,      // 计算补偿系数
    MAG_CALIB_DONE              // 标定完成
} MagCalibState_t;

// 磁力计电流补偿标定数据
typedef struct {
    MagCalibState_t state;      // 当前状态
    float mag_x_low, mag_y_low, mag_z_low;   // 低油门采样
    float mag_x_high, mag_y_high, mag_z_high; // 高油门采样
    float throttle_low, throttle_high;        // 油门值
    uint16_t sample_count_low, sample_count_high;
    float k_x, k_y, k_z;        // 计算得到的补偿系数
    uint8_t valid;              // 标定结果有效标志
} MagCurrentCalib_t;
```

**全局变量**:
```c
extern MagCurrentCalib_t g_mag_current_calib;
```

**函数接口**:
```c
// I2C 总线扫描
uint8_t I2C_ScanBus(I2C_HandleTypeDef *hi2c, I2C_ScanResult_t *result);
void I2C_PrintScanResult(I2C_ScanResult_t *result);
void I2C_ScanAllBuses(void);

// 磁力计电流补偿标定
void MagCurrentCalib_Start(void);
void MagCurrentCalib_Update(float mag_x, float mag_y, float mag_z, float throttle);
MagCalibState_t MagCurrentCalib_GetState(void);
void MagCurrentCalib_PrintResult(void);
```

**使用说明**:
- **I2C 扫描**: 在 `main()` 初始化后调用 `I2C_ScanAllBuses()` 检测总线设备
- **磁力计标定**:
  1. 卸下螺旋桨！
  2. 调用 `MagCurrentCalib_Start()` 开始标定
  3. 保持低油门 (<5%) 等待采集完成
  4. 缓慢增加油门到 >70% 等待采集完成
  5. 系统自动计算 K 系数并打印结果

---

## 6. 更新日志 (Update Log)

| 日期 | 变更内容 | 责任人 |
|:---:|:---|:---|
| 2026-01-16 | 创建接口定义文档 | Doro |
| 2026-01-16 | 添加 GPS 模块接口 | Doro |
| 2026-01-16 | 添加 QMC5883L 磁力计接口 | Doro |
| 2026-01-16 | 添加 Flash 参数管理接口 | Doro |
| 2026-01-16 | 添加匿名上位机 V7 协议接口 | Doro |
| 2026-01-18 | 添加 RingBuffer 异步日志模块 | Doro |
| 2026-01-18 | 添加 QMC5883L 电流补偿接口 | Doro |
| 2026-01-18 | 更新 Motor_Mix Airmode 算法说明 | Doro |
| 2026-01-18 | 更新 PID Anti-windup 算法说明 | Doro |
| 2026-01-18 | 添加调试工具模块 (I2C 扫描 + 磁力计标定) | Doro |
| 2026-01-18 | 添加 AnoV7 错误码定义 (12 个错误码) | Doro |
| 2026-01-23 | 添加 USB_DETECT 引脚定义 (PB9) | Doro |

---

## 7. 冲突检查清单 (Conflict Checklist)

在创建新变量/函数前,必须检查:

- [ ] 是否已在本文档中定义过?
- [ ] 命名是否符合项目规范? (前缀、语义)
- [ ] 是否需要 `volatile` 修饰? (RTOS/ISR 共享)
- [ ] 是否需要 `extern` 声明? (跨文件访问)
- [ ] 是否需要 `__attribute__((packed))`? (网络协议/Flash存储)
- [ ] 是否需要更新 `pinout_allocation.md` 或 `cubemx_setup_guide.md`?

---

**使用规范**:
1. 每次编写代码前,**必须**先查阅此文档
2. 如果发现文档中没有你需要的接口,**必须**先更新此文档再写代码
3. 修改结构体定义时,**必须**同步更新 [`algorithm_definition.md`](algorithm_definition.md)
4. 发现重复定义时,立即报告并在 [`issues_and_feedback.md`](issues_and_feedback.md) 中记录
