/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "mpu6050.h"
#include "usart.h"
#include "sbus.h"
#include "tim.h"
#include "motor.h"
#include "pid.h"
#include "filter.h"
#include "optical_flow.h"
#include "protocol.h"
#include "adc.h"
#include "buzzer.h"
#include "altitude.h"
#include "w25qxx.h"
#include "flash_params.h"
#include "qmc5883l.h"  // QMC5883L 磁力计驱动

// 外部变量声明 (来自 tim.c, usart.c, i2c.c 和 spi.c)
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;  // 原有上位机协议使用 UART1 (PA9/PA10)
extern UART_HandleTypeDef huart3;  // SBUS 接收机使用 UART3 (PB10/PB11)
extern UART_HandleTypeDef huart4;  // 光流模块使用 UART4 (PA0/PA1)
extern UART_HandleTypeDef huart5;  // 匿名上位机 V7 协议使用 UART5 (PC12/PD2)
extern I2C_HandleTypeDef hi2c1;    // MPU6050, QMC5883L 使用 I2C1 (PB8/PB9)
extern I2C_HandleTypeDef hi2c2;    // SPL06 气压计使用 I2C2 (PB10/PB11)
extern SPI_HandleTypeDef hspi1;    // W25Q128 Flash 使用 SPI1 (PA5/PA6/PA7)

// 安全相关宏定义
#define PRE_ARM_MAX_TILT        20.0f   // 解锁前最大倾角 (度)
#define CRASH_MAX_TILT          60.0f   // 炸机检测倾角阈值 (度)
#define CRASH_DETECT_TIME       500     // 倾角超限持续时间 (ms)
#define MOTOR_STUCK_THROTTLE    0.9f    // 满油门阈值
#define MOTOR_STUCK_ACC_MIN     0.5f    // 电机卡死加速度阈值 (g)
#define MOTOR_STUCK_TIME        500     // 电机卡死检测时间 (ms)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ============= 飞控核心变量 =============

// 姿态数据 (由 imu_task 更新，ctl_task 读取)
volatile float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;
volatile float g_gyro_x = 0.0f, g_gyro_y = 0.0f, g_gyro_z = 0.0f;

// 光流速度数据 (由 comm_task 更新)
volatile float g_flow_vx = 0.0f, g_flow_vy = 0.0f;

// 飞行状态机
typedef enum {
    FLIGHT_DISARMED = 0,  // 上锁状态 (电机停止)
    FLIGHT_ARMED = 1      // 解锁状态 (电机可控)
} FlightState_t;

// 飞行模式
typedef enum {
    MODE_MANUAL = 0,      // 手动模式 (姿态控制)
    MODE_ALTITUDE = 1,    // 定高模式 (高度锁定)
    MODE_POSITION = 2     // 定点模式 (位置锁定)
} FlightMode_t;

volatile FlightState_t g_flight_state = FLIGHT_DISARMED;
volatile FlightMode_t g_flight_mode = MODE_MANUAL;

static uint32_t arm_gesture_start = 0;
static uint32_t disarm_gesture_start = 0;
#define ARM_GESTURE_DURATION 1000  // 需要保持解锁手势 1 秒

// PID 控制器 (外环: 角度, 内环: 角速度)
PID_TypeDef pid_roll_angle, pid_roll_rate;
PID_TypeDef pid_pitch_angle, pid_pitch_rate;
PID_TypeDef pid_yaw_angle, pid_yaw_rate;  // Yaw 角度环 + 角速度环

// 位置环 PID (用于定点模式)
PID_TypeDef pid_pos_x, pid_pos_y;

// 高度控制 PID (用于定高模式)
PID_TypeDef pid_alt_pos;   // 高度位置环
PID_TypeDef pid_alt_vel;   // 高度速度环

// 高度控制目标
static float alt_target = 1.0f;  // 目标高度 (m)，默认 1 米
static uint8_t alt_hold_enabled = 0;  // 高度保持使能

// 悬停油门估计
#define HOVER_THROTTLE_INIT  0.45f   // 初始悬停油门估计
static float hover_throttle = HOVER_THROTTLE_INIT;

// Yaw 航向锁定相关变量
static float yaw_target_locked = 0.0f;
static uint8_t yaw_locked = 0;
#define YAW_STICK_DEADZONE 0.05f  // Yaw 摇杆死区

// 低通滤波器 (用于陀螺仪)
LPF2_TypeDef lpf_gyro_x, lpf_gyro_y, lpf_gyro_z;

// 光流接收缓冲区 (外部定义，用于 DMA)
extern uint8_t flow_rx_buf[64];

// 电池电压
volatile float g_battery_voltage = 0.0f;
#define BATTERY_CELL_COUNT  3  // 默认 3S 电池

// 上位机心跳看门狗 (防止树莓派卡死时飞机失控)
volatile uint32_t g_pi_last_heartbeat = 0;  // 最后收到上位机指令的时间
#define PI_HEARTBEAT_TIMEOUT  500  // 500ms 超时

// 炸机检测相关变量
static uint32_t crash_tilt_start = 0;      // 倾角超限开始时间
static uint32_t motor_stuck_start = 0;     // 电机卡死检测开始时间

// 匿名上位机 PID 写入回调
void AnoV7_PID_Handler(AnoV7_PID_t *pid_data) {
    if (pid_data == NULL) return;
    
    // 获取 Flash 参数中的 PID 指针
    PID_Params_t *p_params = FlashParams_GetPID((PID_Index_t)pid_data->pid_id);
    
    if (p_params != NULL) {
        // 更新参数
        p_params->kp = pid_data->kp;
        p_params->ki = pid_data->ki;
        p_params->kd = pid_data->kd;
        
        // 应用到实时 PID 控制器
        FlashParams_ApplyToPID();
        
        // 标记参数已修改 (触发延迟保存)
        FlashParams_MarkDirty();
        
        // 发送回传确认
        AnoV7_SendPID(pid_data->pid_id);
        
        printf("[AnoV7] PID Updated: ID=%d, P=%.3f, I=%.3f, D=%.3f\r\n",
               pid_data->pid_id, pid_data->kp, pid_data->ki, pid_data->kd);
    }
}

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ctl_taskHandle;
osThreadId imu_taskHandle;
osThreadId comm_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// ============= 安全检查函数 =============

/**
 * @brief 解锁前安全检查
 * @return 1: 安全可解锁, 0: 不安全
 */
static uint8_t PreArm_SafetyCheck(void) {
    // 1. 检查倾角是否过大 (防止歪着解锁)
    if (fabs(g_roll) > PRE_ARM_MAX_TILT || fabs(g_pitch) > PRE_ARM_MAX_TILT) {
        printf("Pre-Arm FAILED: Tilt too large! Roll=%.1f, Pitch=%.1f\r\n", g_roll, g_pitch);
        return 0;
    }
    
    // 2. 检查油门是否归零 (防止带油门解锁)
    float throttle = SBUS_GetThrottle();
    if (throttle > 0.1f) {
        printf("Pre-Arm FAILED: Throttle not zero! Throttle=%.2f\r\n", throttle);
        return 0;
    }
    
    // 3. 检查电池电压是否健康
    if (Battery_IsCritical(g_battery_voltage, BATTERY_CELL_COUNT)) {
        printf("Pre-Arm FAILED: Battery critical! V=%.2f\r\n", g_battery_voltage);
        return 0;
    }
    
    // 4. 检查陀螺仪是否已校准
    if (!MPU6050_IsCalibrated()) {
        printf("Pre-Arm FAILED: Gyro not calibrated!\r\n");
        return 0;
    }
    
    return 1;  // 所有检查通过
}

/**
 * @brief 炸机检测 (倾角过大或电机卡死)
 * @return 1: 检测到炸机, 0: 正常
 */
static uint8_t Crash_Detection(float throttle, float acc_magnitude) {
    uint32_t now = HAL_GetTick();
    
    // 1. 倾角超限检测
    if (fabs(g_roll) > CRASH_MAX_TILT || fabs(g_pitch) > CRASH_MAX_TILT) {
        if (crash_tilt_start == 0) {
            crash_tilt_start = now;
        } else if (now - crash_tilt_start > CRASH_DETECT_TIME) {
            printf(">>> CRASH: Tilt exceeded %.0f deg! <<<\r\n", CRASH_MAX_TILT);
            return 1;
        }
    } else {
        crash_tilt_start = 0;  // 恢复正常，重置计时
    }
    
    // 2. 电机卡死检测 (满油门但无加速度响应)
    if (throttle > MOTOR_STUCK_THROTTLE) {
        if (acc_magnitude < MOTOR_STUCK_ACC_MIN) {
            if (motor_stuck_start == 0) {
                motor_stuck_start = now;
            } else if (now - motor_stuck_start > MOTOR_STUCK_TIME) {
                printf(">>> CRASH: Motor stuck! Throttle=%.2f, Acc=%.2f <<<\r\n", throttle, acc_magnitude);
                return 1;
            }
        } else {
            motor_stuck_start = 0;
        }
    } else {
        motor_stuck_start = 0;
    }
    
    return 0;  // 正常
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCtlTask(void const * argument);
void StartImuTask(void const * argument);
void StartCommTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ctl_task */
  osThreadDef(ctl_task, StartCtlTask, osPriorityHigh, 0, 512);
  ctl_taskHandle = osThreadCreate(osThread(ctl_task), NULL);

  /* definition and creation of imu_task */
  osThreadDef(imu_task, StartImuTask, osPriorityRealtime, 0, 512);
  imu_taskHandle = osThreadCreate(osThread(imu_task), NULL);

  /* definition and creation of comm_task */
  osThreadDef(comm_task, StartCommTask, osPriorityAboveNormal, 0, 256);
  comm_taskHandle = osThreadCreate(osThread(comm_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // ========== QMC5883L 磁力计初始化（Risk #028 修复：移到任务启动前） ==========
  printf("Initializing QMC5883L Magnetometer...\r\n");
  if (QMC5883L_Init(&hi2c1) == 0) {
      printf("QMC5883L Init Success! (I2C1, 50Hz, ±2G)\r\n");
  } else {
      printf("QMC5883L Init Failed! Check I2C1 wiring.\r\n");
  }
  
  // ========== Flash 参数存储初始化 ==========
  printf("Initializing Flash Storage...\r\n");
  if (FlashParams_Init() == 0) {
      printf("Flash Init Success! ID: 0x%04X\r\n", W25QXX_ReadID());
      
      // 应用加载的参数到 PID 控制器
      FlashParams_ApplyToPID();
  } else {
      printf("Flash Init Failed! Using default params.\r\n");
  }
  
  // ========== 匿名上位机 V7 协议初始化 ==========
  printf("Initializing Anonymous V7 Protocol...\r\n");
  AnoV7_Init(&huart5);
  AnoV7_RegisterPIDCallback(AnoV7_PID_Handler); // 注册 PID 写入回调
  printf("Anonymous V7 Init Success! (UART5, PC12/PD2, 115200 baud)\r\n");
  
  // 启动 SBUS DMA 接收
  SBUS_Init(&huart3);
  
  // 初始化电机 PWM (TIM3: M1/M2, TIM4: M3/M4)
  Motor_Init(&htim3, &htim4);
  
  // 初始化光流模块 (使用 UART4 - PA0/PA1)
  OpticalFlow_Init(&huart4);
  
  // 初始化蜂鸣器
  Buzzer_Init();
  
  // 初始化 PID 控制器 (如果 Flash 加载失败，使用默认值)
  // 外环 (角度环): 输出限幅 ±200 deg/s，无积分
  PID_Init(&pid_roll_angle, 5.0f, 0.0f, 0.0f, 200.0f, 0.0f);
  PID_Init(&pid_pitch_angle, 5.0f, 0.0f, 0.0f, 200.0f, 0.0f);
  
  // 内环 (角速度环): 输出限幅 ±0.5 (对应混控范围)，积分限幅 ±0.2
  PID_Init(&pid_roll_rate, 0.8f, 0.02f, 0.01f, 0.5f, 0.2f);
  PID_Init(&pid_pitch_rate, 0.8f, 0.02f, 0.01f, 0.5f, 0.2f);
  PID_Init(&pid_yaw_rate, 1.0f, 0.05f, 0.0f, 0.5f, 0.2f);
  
  // Yaw 角度环 (用于航向锁定): 输出限幅 ±100 deg/s
  PID_Init(&pid_yaw_angle, 3.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  
  // 位置环 PID (定点模式使用)
  PID_Init(&pid_pos_x, 0.5f, 0.0f, 0.0f, 15.0f, 0.0f);  // 输出限幅 ±15 度
  PID_Init(&pid_pos_y, 0.5f, 0.0f, 0.0f, 15.0f, 0.0f);
  
  // 高度环 PID (定高模式使用)
  // 位置环: 输出限幅 ±2.0 m/s
  PID_Init(&pid_alt_pos, 1.5f, 0.0f, 0.0f, 2.0f, 0.0f);
  // 速度环: 输出限幅 ±0.3 (油门增量)
  PID_Init(&pid_alt_vel, 0.15f, 0.02f, 0.01f, 0.3f, 0.15f);
  
  // 初始化高度融合模块
  Altitude_Init(&Alt_Data);
  
  // 初始化陀螺仪低通滤波器 (采样 500Hz, 截止 30Hz)
  LPF2_Init(&lpf_gyro_x, 500.0f, 30.0f);
  LPF2_Init(&lpf_gyro_y, 500.0f, 30.0f);
  LPF2_Init(&lpf_gyro_z, 500.0f, 30.0f);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  printf("System Started! FreeRTOS is running...\r\n");
  
  // 启动提示音
  Buzzer_SetState(BUZZER_CALIBRATING);
  
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    
    // 更新蜂鸣器状态机
    Buzzer_Update();
    
    osDelay(10);  // 10ms 周期供蜂鸣器状态机使用
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCtlTask */
/**
* @brief Function implementing the ctl_task thread.
*        控制任务: 串级 PID + 电机混控
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCtlTask */
void StartCtlTask(void const * argument)
{
  /* USER CODE BEGIN StartCtlTask */
  float target_roll = 0.0f, target_pitch = 0.0f;
  float target_yaw_rate = 0.0f;
  float roll_rate_out, pitch_rate_out, yaw_rate_out;
  float throttle, roll_ctl, pitch_ctl, yaw_ctl;
  float pos_roll_offset = 0.0f, pos_pitch_offset = 0.0f;
  float throttle_out = 0.0f;  // 最终油门输出
  float alt_vel_target = 0.0f;  // 目标垂直速度
  float alt_throttle_offset = 0.0f;  // 高度环油门补偿
  
  // 等待传感器初始化完成
  osDelay(500);
  
  printf("Control Task Started!\r\n");
  
  /* Infinite loop */
  for(;;)
  {
    // ========== 1. 解锁/上锁状态机 ==========
    if (g_flight_state == FLIGHT_DISARMED) {
        // 检测解锁手势
        if (SBUS_IsConnected() && SBUS_CheckArmGesture()) {
            if (arm_gesture_start == 0) {
                arm_gesture_start = HAL_GetTick();
            } else if (HAL_GetTick() - arm_gesture_start > ARM_GESTURE_DURATION) {
                // ========== 解锁前安全检查 ==========
                if (PreArm_SafetyCheck()) {
                    g_flight_state = FLIGHT_ARMED;
                    arm_gesture_start = 0;
                    printf(">>> ARMED! <<<\r\n");
                    
                    // 解锁提示音
                    Buzzer_SetState(BUZZER_ARMED);
                    
                    // 启用 AirMode 电机怠速保护
                    Motor_SetAirMode(1);
                    printf("AirMode: Enabled (Idle PWM = 1100)\r\n");
                    
                    // 重置 PID 积分
                    PID_Reset(&pid_roll_rate);
                    PID_Reset(&pid_pitch_rate);
                    PID_Reset(&pid_yaw_rate);
                    PID_Reset(&pid_pos_x);
                    PID_Reset(&pid_pos_y);
                    
                    // 重置炸机检测计时器
                    crash_tilt_start = 0;
                    motor_stuck_start = 0;
                } else {
                    // 安全检查未通过
                    arm_gesture_start = 0;
                    Buzzer_SetState(BUZZER_FAILSAFE);
                }
            }
        } else {
            arm_gesture_start = 0;
        }
        
        // 上锁状态: 电机停止
        Motor_Stop();
        
    } else {  // FLIGHT_ARMED
        // 检测上锁手势
        if (SBUS_CheckDisarmGesture()) {
            if (disarm_gesture_start == 0) {
                disarm_gesture_start = HAL_GetTick();
            } else if (HAL_GetTick() - disarm_gesture_start > ARM_GESTURE_DURATION) {
                g_flight_state = FLIGHT_DISARMED;
                disarm_gesture_start = 0;
                printf(">>> DISARMED! <<<\r\n");
                
                // 禁用 AirMode（允许电机完全停止）
                Motor_SetAirMode(0);
                printf("AirMode: Disabled (Full Stop)\r\n");
                
                // 上锁提示音
                Buzzer_SetState(BUZZER_DISARMED);
            }
        } else {
            disarm_gesture_start = 0;
        }
        
        // ========== 上锁时自动保存参数 ==========
        // 检测到从 ARMED 切换到 DISARMED 时，保存参数到 Flash
        static FlightState_t last_flight_state = FLIGHT_DISARMED;
        if (last_flight_state == FLIGHT_ARMED && g_flight_state == FLIGHT_DISARMED) {
            // 刚刚上锁，检查是否有参数需要保存
            if (FlashParams_IsDirty()) {
                printf("Disarmed, saving params to Flash...\r\n");
                FlashParams_Save();
            }
        }
        last_flight_state = g_flight_state;
        
        // 失控保护: 信号丢失或接收机进入失控模式立即上锁
        // Risk #091 修复：增加 SBUS Failsafe 标志位检查
        if (!SBUS_IsConnected() || SBUS_Data.failsafe) {
            g_flight_state = FLIGHT_DISARMED;
            Motor_Stop();
            Buzzer_SetState(BUZZER_FAILSAFE);
            if (SBUS_Data.failsafe) {
                printf(">>> FAILSAFE: RX Failsafe Active! <<<\r\n");
            } else {
                printf(">>> FAILSAFE: Signal Lost! <<<\r\n");
            }
            osDelay(4);
            continue;
        }
        
        // 低电压保护
        if (Battery_IsCritical(g_battery_voltage, BATTERY_CELL_COUNT)) {
            // 危急电压: 自动降落 (降低油门)
            Buzzer_SetState(BUZZER_CRITICAL);
        } else if (Battery_IsLow(g_battery_voltage, BATTERY_CELL_COUNT)) {
            // 低电压警告
            Buzzer_SetState(BUZZER_LOW_VOLTAGE);
        }
        
        // ========== 炸机检测 ==========
        // 计算加速度合矢量 (用于电机卡死检测)
        // 使用 MPU6050 的真实加速度数据
        float acc_magnitude = sqrtf(
            MPU6050_Data.Ax * MPU6050_Data.Ax +
            MPU6050_Data.Ay * MPU6050_Data.Ay +
            MPU6050_Data.Az * MPU6050_Data.Az
        );
        throttle = SBUS_GetThrottle();
        
        if (Crash_Detection(throttle, acc_magnitude)) {  // 传入真实加速度
            g_flight_state = FLIGHT_DISARMED;
            Motor_Stop();
            Buzzer_SetState(BUZZER_FAILSAFE);
            printf(">>> AUTO DISARM: Crash detected! <<<\r\n");
            osDelay(4);
            continue;
        }
        
        // 上位机心跳超时保护 (定点模式下生效)
        if (g_flight_mode == MODE_POSITION) {
            if (HAL_GetTick() - g_pi_last_heartbeat > PI_HEARTBEAT_TIMEOUT) {
                // 树莓派超时，切换回手动模式
                g_flight_mode = MODE_MANUAL;
                printf(">>> PI TIMEOUT: Switch to MANUAL <<<\r\n");
            }
        }
        
        // ========== 2. 飞行模式切换 (CH5 三段开关) ==========
        int16_t ch5_val = SBUS_Data.channels[4];  // CH5 用于模式切换
        if (ch5_val < 500) {
            g_flight_mode = MODE_MANUAL;
        } else if (ch5_val < 1500) {
            g_flight_mode = MODE_ALTITUDE;
        } else {
            g_flight_mode = MODE_POSITION;
        }
        
        // ========== 3. 获取遥控器输入 ==========
        throttle = SBUS_GetThrottle();       // 0.0 ~ 1.0
        roll_ctl = SBUS_GetRoll();           // -1.0 ~ 1.0
        pitch_ctl = SBUS_GetPitch();         // -1.0 ~ 1.0
        yaw_ctl = SBUS_GetYaw();             // -1.0 ~ 1.0
        
        // ========== 3.5 磁力计校准触发 (上锁状态下) ==========
        // 手势: 双摇杆外八字 (左摇杆左下，右摇杆右下) 持续 2 秒
        // Left: Thr < 0.1, Yaw < -0.9
        // Right: Pitch < -0.9, Roll > 0.9
        static uint32_t mag_calib_start = 0;
        if (g_flight_state == FLIGHT_DISARMED) {
            if (throttle < 0.1f && yaw_ctl < -0.9f && pitch_ctl < -0.9f && roll_ctl > 0.9f) {
                if (mag_calib_start == 0) {
                    mag_calib_start = HAL_GetTick();
                } else if (HAL_GetTick() - mag_calib_start > 2000) {
                    // 触发校准
                    QMC5883L_Calibrate();
                    Buzzer_SetState(BUZZER_CALIBRATING); // 提示音
                    mag_calib_start = 0; // 重置防止重复触发
                }
            } else {
                mag_calib_start = 0;
            }
        }

        // ========== 4. 根据飞行模式计算期望角度 ==========
        pos_roll_offset = 0.0f;
        pos_pitch_offset = 0.0f;
        
        if (g_flight_mode == MODE_POSITION && Flow_Data.valid) {
            // 定点模式: 光流速度补偿
            // 光流 X 对应 Roll, 光流 Y 对应 Pitch
            pos_roll_offset = PID_Calculate(&pid_pos_x, 0.0f, g_flow_vx, 0.004f);
            pos_pitch_offset = PID_Calculate(&pid_pos_y, 0.0f, g_flow_vy, 0.004f);
        }
        
        // 计算期望角度 (最大倾角 30 度 + 位置补偿)
        target_roll = roll_ctl * 30.0f + pos_roll_offset;
        target_pitch = pitch_ctl * 30.0f + pos_pitch_offset;
        
        // ========== Yaw 航向锁定逻辑 ==========
        // 防止纯 6 轴的 Yaw 漂移导致定点时画圈
        if (fabs(yaw_ctl) < YAW_STICK_DEADZONE) {
            // 摇杆回中，锁定当前航向
            if (!yaw_locked) {
                yaw_target_locked = g_yaw;
                yaw_locked = 1;
            }
            // 使用角度环 PID 锁定航向
            target_yaw_rate = PID_Calculate(&pid_yaw_angle, yaw_target_locked, g_yaw, 0.004f);
        } else {
            // 摇杆有输入，解除锁定，直接控制角速度
            yaw_locked = 0;
            target_yaw_rate = yaw_ctl * 200.0f;  // 最大偏航速率 200 deg/s
        }
        
        // ========== 5. 串级 PID 计算 ==========
        // 外环: 角度 -> 期望角速度
        float target_roll_rate = PID_Calculate(&pid_roll_angle, target_roll, g_roll, 0.004f);
        float target_pitch_rate = PID_Calculate(&pid_pitch_angle, target_pitch, g_pitch, 0.004f);
        
        // 内环: 角速度 -> 电机控制量
        // 注意: 陀螺仪输出是 rad/s，需要转换为 deg/s
        roll_rate_out = PID_Calculate(&pid_roll_rate, target_roll_rate, g_gyro_x * 57.29578f, 0.004f);
        pitch_rate_out = PID_Calculate(&pid_pitch_rate, target_pitch_rate, g_gyro_y * 57.29578f, 0.004f);
        yaw_rate_out = PID_Calculate(&pid_yaw_rate, target_yaw_rate, g_gyro_z * 57.29578f, 0.004f);
        
        // ========== 5.5 高度环 PID 控制 (定高模式) ==========
        if (g_flight_mode == MODE_ALTITUDE && Altitude_IsReady(&Alt_Data)) {
            // 定高模式: 使用高度环 PID 控制油门
            float current_height = Altitude_GetHeight(&Alt_Data);
            float current_vel = Altitude_GetVelocity(&Alt_Data);
            
            // 当油门推过中点时，启动高度保持
            if (throttle > 0.45f && throttle < 0.55f) {
                // 油门在中点附近，启用高度保持
                if (!alt_hold_enabled) {
                    alt_target = current_height;  // 锁定当前高度
                    alt_hold_enabled = 1;
                    PID_Reset(&pid_alt_pos);
                    PID_Reset(&pid_alt_vel);
                    printf("Alt Hold: Target=%.2fm\r\n", alt_target);
                }
                
                // 串级 PID: 位置环 -> 速度环
                alt_vel_target = PID_Calculate(&pid_alt_pos, alt_target, current_height, 0.004f);
                alt_throttle_offset = PID_Calculate(&pid_alt_vel, alt_vel_target, current_vel, 0.004f);
                
                // 使用悬停油门 + PID 补偿
                throttle_out = hover_throttle + alt_throttle_offset;
                
            } else {
                // 油门不在中点，手动控制高度
                alt_hold_enabled = 0;
                
                if (throttle > 0.55f) {
                    // 推油门: 上升
                    alt_target = current_height + (throttle - 0.55f) * 0.1f;  // 上升幅度
                } else if (throttle < 0.45f) {
                    // 收油门: 下降
                    alt_target = current_height - (0.45f - throttle) * 0.1f;  // 下降幅度
                    if (alt_target < 0.1f) alt_target = 0.1f;  // 最低高度限制
                }
                
                throttle_out = throttle;  // 手动油门
            }
            
            // 油门限幅
            if (throttle_out < 0.0f) throttle_out = 0.0f;
            if (throttle_out > 1.0f) throttle_out = 1.0f;
            
        } else {
            // 手动模式或定点模式: 直接使用遥控器油门
            alt_hold_enabled = 0;
            throttle_out = throttle;
        }
        
        // ========== 6. 电机混控 ==========
        Motor_Mix(throttle_out, roll_rate_out, pitch_rate_out, yaw_rate_out);
    }
    
    // 4ms 周期 (250Hz)
    osDelay(4);
  }
  /* USER CODE END StartCtlTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  Mahony_TypeDef Mahony;
  float roll, pitch, yaw;
  float gx_filtered, gy_filtered, gz_filtered;
  float acc_z_earth;  // 地球坐标系 Z 轴加速度
  uint8_t ret;

  // 等待传感器上电稳定
  osDelay(100);

  printf("Initializing MPU6050...\r\n");
  ret = MPU6050_Init(&hi2c1);

  if (ret == 0) {
      printf("MPU6050 Init Success!\r\n");
      
      // ========== 陀螺仪零偏校准 ==========
      // 校准期间飞机必须静止！
      MPU6050_CalibrateGyro(&hi2c1);
      
  } else {
      printf("MPU6050 Init Failed! Check wiring.\r\n");
  }

  // QMC5883L 已在 MX_FREERTOS_Init() 中初始化（避免阻塞高频任务）
  uint8_t qmc_ret = QMC_Data.initialized ? 0 : 1;

  // ========== SPL06 气压计初始化 ==========
  printf("Initializing SPL06 Barometer...\r\n");
  uint8_t spl06_ret = SPL06_Init(&hi2c2);
  if (spl06_ret == 0) {
      printf("SPL06 Init Success!\r\n");
  } else {
      printf("SPL06 Init Failed! Check I2C2 wiring.\r\n");
  }

  // 初始化 Mahony 算法 (Kp=2.0, Ki=0.001)
  Mahony_Init(&Mahony, 2.0f, 0.001f);

  /* Infinite loop */
  static uint32_t spl06_timer = 0;  // SPL06 读取定时器
  static uint8_t qmc_divider = 0;   // QMC5883L 降采样计数器 (500Hz -> 50Hz)
  
  for(;;)
  {
    if (ret == 0) {
        // 使用带校准补偿的读取函数
        MPU6050_Read_All_Calibrated(&hi2c1, (MPU6050_t *)&MPU6050_Data);
        
        // 陀螺仪低通滤波 (滤除电机震动)
        gx_filtered = LPF2_Apply(&lpf_gyro_x, MPU6050_Data.Gx);
        gy_filtered = LPF2_Apply(&lpf_gyro_y, MPU6050_Data.Gy);
        gz_filtered = LPF2_Apply(&lpf_gyro_z, MPU6050_Data.Gz);
        
        // 姿态解算 (使用滤波后的陀螺仪数据)
        Mahony_Update(&Mahony,
                      gx_filtered, gy_filtered, gz_filtered,
                      MPU6050_Data.Ax, MPU6050_Data.Ay, MPU6050_Data.Az,
                      0.002f);
        
        // 获取欧拉角
        Mahony_GetEulerAngle(&Mahony, &roll, &pitch, &yaw);
        
        // ========== QMC5883L 磁力计读取与融合 (50Hz 降采样) ==========
        if (qmc_ret == 0 && ++qmc_divider >= 10) {
            qmc_divider = 0;  // 500Hz / 10 = 50Hz
            
            // 读取原始磁场数据
            if (QMC5883L_ReadRaw(&hi2c1) == 0) {
                // 更新磁力计校准状态 (如果正在校准)
                if (QMC5883L_CalibrateUpdate()) {
                    // 校准完成，蜂鸣器提示
                    Buzzer_SetState(BUZZER_ARMED); // 短响提示
                }

                // 计算航向角（带倾斜补偿）
                float mag_heading = QMC5883L_CalculateHeading(&Mahony);
                
                // 互补滤波融合：98% 陀螺仪 + 2% 磁力计
                // 防止磁干扰时，降低磁力计权重
                float alpha = 0.98f;  // 默认融合系数
                
                // Risk #031 修复：添加迟滞逻辑，防止边界抖动导致日志刷屏
                static uint8_t interference_state = 0;  // 0=正常, 1=干扰
                
                // 迟滞判断（Hysteresis）
                if (QMC_Data.mag_strength > 0.76f || QMC_Data.mag_strength < 0.14f) {
                    // 进入干扰状态（高阈值）
                    if (!interference_state) {
                        interference_state = 1;
                        printf("[QMC5883L] Magnetic interference detected! Strength=%.3f G\r\n",
                               QMC_Data.mag_strength);
                    }
                } else if (QMC_Data.mag_strength < 0.73f && QMC_Data.mag_strength > 0.17f) {
                    // 退出干扰状态（低阈值）
                    if (interference_state) {
                        interference_state = 0;
                        printf("[QMC5883L] Interference cleared. Strength=%.3f G\r\n",
                               QMC_Data.mag_strength);
                    }
                }
                // 0.73~0.76 和 0.14~0.17 之间是死区，状态不变
                
                // 根据状态调整融合系数
                if (interference_state) {
                    alpha = 0.995f;  // 干扰模式：99.5% 陀螺仪 + 0.5% 磁力计
                } else {
                    alpha = 0.98f;   // 正常模式：98% 陀螺仪 + 2% 磁力计
                }
                
                // 执行互补滤波（融合陀螺仪 Yaw 和磁力计航向）
                yaw = QMC5883L_ComplementaryFilter(yaw, mag_heading, 0.02f, alpha);
            }
        }
        
        // 更新全局变量 (供 ctl_task 使用)
        g_roll = roll;
        g_pitch = pitch;
        g_yaw = yaw;
        g_gyro_x = gx_filtered;
        g_gyro_y = gy_filtered;
        g_gyro_z = gz_filtered;
        
        // ========== 高度融合 ==========
        // 1. 将机体加速度转换到地球坐标系 Z 轴
        acc_z_earth = Altitude_TransformAccZ(&Mahony,
                                              MPU6050_Data.Ax,
                                              MPU6050_Data.Ay,
                                              MPU6050_Data.Az);
        
        // 2. 互补滤波融合更新 (2ms 周期)
        Altitude_FusionUpdate(&Alt_Data, acc_z_earth, 0.002f);

        // 打印欧拉角 (格式: Roll,Pitch,Yaw) - 配合串口绘图工具使用
        // 降低打印频率，避免阻塞 (每 50ms 打印一次)
        static uint8_t print_cnt = 0;
        if (++print_cnt >= 25) {
            print_cnt = 0;
            printf("%.2f,%.2f,%.2f\r\n", roll, pitch, yaw);
        }
    }
    
    // ========== SPL06 气压计数据读取 (30ms 间隔，约 33Hz) ==========
    if (spl06_ret == 0 && HAL_GetTick() - spl06_timer >= 30) {
        spl06_timer = HAL_GetTick();
        
        // 读取气压和温度原始数据
        if (SPL06_ReadRawData(&hi2c2) == 0) {
            // 数据读取成功，SPL06_Data 已更新 (包括 pressure, temperature, altitude)
            
            // ========== 更新气压计高度 ==========
            // 智能切换逻辑在 Altitude_FusionUpdate 中自动处理（<1.8m激光，>2.2m气压，中间混合）
            if (SPL06_Data.altitude > 0.0f && SPL06_Data.altitude < 100.0f) {
                Altitude_UpdateBarometer(&Alt_Data, SPL06_Data.altitude);
            }
            
            // 调试打印 (降低频率，每 1 秒打印一次)
            static uint8_t baro_print_cnt = 0;
            if (++baro_print_cnt >= 33) {
                baro_print_cnt = 0;
                printf("[SPL06] P=%.2f Pa, T=%.2f C, Alt=%.2f m\r\n",
                       SPL06_Data.pressure,
                       SPL06_Data.temperature,
                       SPL06_Data.altitude);
            }
        }
    }
    
    // 2ms 周期 (500Hz)
    osDelay(2);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the comm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  static uint32_t status_send_tick = 0;
  static uint32_t voltage_read_tick = 0;
  Status_Packet_t status;
  
  // ========== 匿名上位机 V7 发送定时器 ==========
  static uint32_t ano_attitude_tick = 0;   // 姿态帧 20Hz (50ms)
  static uint32_t ano_sensor_tick = 0;     // 传感器帧 10Hz (100ms)
  static uint32_t ano_motor_tick = 0;      // 电机帧 10Hz (100ms)
  static uint32_t ano_status_tick = 0;     // 状态帧 5Hz (200ms)
  
  /* Infinite loop */
  for(;;)
  {
    // ========== 1. 解析 SBUS 数据 ==========
    // SBUS 帧间隔 14ms，这里 10ms 检查一次足够
    SBUS_Parse(SBUS_RxBuffer);

    // ========== 2. 解析光流数据 ==========
    // 从 DMA 缓冲区解析光流帧 (LC302 协议)
    OpticalFlow_Parse(flow_rx_buf, 64);
    
    // ========== 3. 获取带高度补偿的光流速度 ==========
    // 使用新的 OpticalFlow_GetVelocity 函数，自动进行高度补偿
    float vx_temp, vy_temp;
    if (OpticalFlow_GetVelocity(&vx_temp, &vy_temp)) {
        g_flow_vx = vx_temp;
        g_flow_vy = vy_temp;
    } else {
        g_flow_vx = 0.0f;
        g_flow_vy = 0.0f;
    }
    
    // ========== 3.5 更新激光高度到高度融合模块 ==========
    // 从光流模块的 distance 字段获取激光高度
    if (Flow_Data.valid) {
        float laser_height_m = Flow_Data.distance / 1000.0f;  // mm -> m
        Altitude_UpdateLaser(&Alt_Data, laser_height_m);
    }
    
    // ========== 4. 读取电池电压 ==========
    // 每 500ms 读取一次 (电压变化慢)
    if (HAL_GetTick() - voltage_read_tick >= 500) {
        voltage_read_tick = HAL_GetTick();
        g_battery_voltage = Battery_GetVoltage();
    }
    
    // ========== 5. 更新上位机心跳 (当收到有效指令时) ==========
    // 注意：这里只是示例，实际应在 Protocol_Parse 回调中更新
    // 当收到有效的上位机指令包时，调用: g_pi_last_heartbeat = HAL_GetTick();
    
    // ========== 6. 周期性保存参数到 Flash ==========
    // 每 2 秒检查一次是否有参数需要保存
    FlashParams_PeriodicSave();
    
    // ========== 7. 发送状态数据到上位机 (原有协议) ==========
    // 每 100ms 发送一次
    if (HAL_GetTick() - status_send_tick >= 100) {
        status_send_tick = HAL_GetTick();
        
        status.roll = g_roll;
        status.pitch = g_pitch;
        status.yaw = g_yaw;
        status.voltage = g_battery_voltage;
        
        Protocol_SendStatus(&huart1, &status);
    }
    
    // ========== 8. 匿名上位机 V7 协议定时发送 ==========
    uint32_t current_tick = HAL_GetTick();
    
    // 8.1 姿态数据帧 - 20Hz (50ms)
    if (current_tick - ano_attitude_tick >= 50) {
        ano_attitude_tick = current_tick;
        AnoV7_SendAttitude();
    }
    
    // 8.2 传感器数据帧 - 10Hz (100ms)
    if (current_tick - ano_sensor_tick >= 100) {
        ano_sensor_tick = current_tick;
        AnoV7_SendSensor();
    }
    
    // 8.3 电机 PWM 帧 - 10Hz (100ms)
    if (current_tick - ano_motor_tick >= 100) {
        ano_motor_tick = current_tick;
        AnoV7_SendMotor();
    }
    
    // 8.4 状态数据帧 - 5Hz (200ms)
    if (current_tick - ano_status_tick >= 200) {
        ano_status_tick = current_tick;
        AnoV7_SendStatus();
    }

    osDelay(10);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
