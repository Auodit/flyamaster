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
#include "mpu6050.h"
#include "mahony.h"
#include "crsf.h"
#include "flight_control.h"
#include "ring_buffer.h"
#include "gps.h"
#include "at7456e.h"
#include "battery.h"
#include "qmc5883l.h"
#include "led_indicator.h"
#include "smartaudio.h"
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_TASK_PERIOD_MS      2       /* 500Hz */
#define CTL_TASK_PERIOD_MS      4       /* 250Hz */
#define LOG_TASK_PERIOD_MS      10      /* 100Hz */
#define COMM_TASK_PERIOD_MS     100     /* 10Hz */
#define GPS_TASK_PERIOD_MS      100     /* 10Hz */
#define DEFAULT_TASK_PERIOD_MS  100     /* 10Hz (电池监控需要更高频率) */
#define MAG_UPDATE_DIVIDER      5       /* 磁力计更新分频 (500Hz / 5 = 100Hz) */

/* 蜂鸣器报警模式 */
#define BUZZER_OFF              0
#define BUZZER_WARNING          1       /* 低电压警告：间歇鸣叫 */
#define BUZZER_CRITICAL         2       /* 电压临界：急促鸣叫 */
#define BUZZER_OVERCURRENT      3       /* 过流：连续鸣叫 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint32_t imu_task_counter = 0;
static uint32_t ctl_task_counter = 0;
static uint32_t log_task_counter = 0;
static uint32_t default_task_counter = 0;
static uint8_t buzzer_mode = BUZZER_OFF;

/* I2C1 互斥锁句柄 (MPU6050 和 SPL06 共享) */
osMutexId_t i2c1MutexHandle = NULL;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ctl_task */
osThreadId_t ctl_taskHandle;
const osThreadAttr_t ctl_task_attributes = {
  .name = "ctl_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for log_task */
osThreadId_t log_taskHandle;
const osThreadAttr_t log_task_attributes = {
  .name = "log_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for comm_task */
osThreadId_t comm_taskHandle;
const osThreadAttr_t comm_task_attributes = {
  .name = "comm_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gps_task */
osThreadId_t gps_taskHandle;
const osThreadAttr_t gps_task_attributes = {
  .name = "gps_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void LED_UpdateAll(void);
static void Battery_Check(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartLogTask(void *argument);
void StartCommTask(void *argument);
void StartTask06(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* I2C1 互斥锁 (MPU6050 和 SPL06 共享) */
    const osMutexAttr_t i2c1Mutex_attributes = {
      .name = "i2c1Mutex",
      .attr_bits = osMutexRecursive | osMutexPrioInherit,
      .cb_mem = NULL,
      .cb_size = 0
    };
    i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(StartTask02, NULL, &imu_task_attributes);

  /* creation of ctl_task */
  ctl_taskHandle = osThreadNew(StartTask03, NULL, &ctl_task_attributes);

  /* creation of log_task */
  log_taskHandle = osThreadNew(StartLogTask, NULL, &log_task_attributes);

  /* creation of comm_task */
  comm_taskHandle = osThreadNew(StartCommTask, NULL, &comm_task_attributes);

  /* creation of gps_task */
  gps_taskHandle = osThreadNew(StartTask06, NULL, &gps_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  
  /* 系统监控任务：LED 指示灯、电池检测、看门狗 */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  /* 初始化 LED 指示灯系统 */
  LED_Init();
  
  /* 启动 LED 自检灯效 */
  LED_StartSelfTest();
  
  /* 初始化电池监控 (启动 ADC DMA) */
  Battery_Init();
  
  /* 等待 ADC 稳定 + LED 自检完成 */
  osDelay(100);
  
  /* 等待 LED 自检完成 */
  while (!LED_IsSelfTestDone()) {
    LED_Update(HAL_GetTick());
    osDelay(10);
  }
  
  /* 自动检测电池节数 */
  Battery_AutoDetectCells();
  
  /* 设置初始 LED 状态 */
  LED_SetSysState(SYS_STATE_STANDBY);
  LED_SetErrState(ERR_STATE_NONE);
  LED_SetGpsState(GPS_STATE_NONE);
  LED_SetRxState(RX_STATE_FAILSAFE);
  LED_SetModeState(MODE_STATE_ANGLE);
  LED_SetLogState(LOG_STATE_IDLE);
  LED_SetVtxState(VTX_STATE_NORMAL);
  
  /* Infinite loop */
  for(;;)
  {
    /* 更新电池数据 (dt = 0.1s @ 10Hz) */
    Battery_Update(0.1f);
    
    /* 更新所有 LED 状态 */
    LED_UpdateAll();
    
    /* LED 闪烁更新 */
    LED_Update(HAL_GetTick());
    
    /* 电池状态检测与报警 */
    Battery_Check();
    
    /* 喂狗 (如果启用了 IWDG) */
    /* HAL_IWDG_Refresh(&hiwdg); */
    
    /* 计数器 */
    default_task_counter++;
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DEFAULT_TASK_PERIOD_MS));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  
  /* IMU 任务：传感器读取 + 姿态解算 (500Hz) */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint8_t mag_divider = 0;
  
  /* 初始化 MPU6050 */
  MPU6050_Init();
  
  /* 初始化 QMC5883L 磁力计 (I2C2 独立总线) */
  QMC5883L_Init();
  
  /* 等待传感器稳定 */
  osDelay(100);
  
  /* 校准陀螺仪 */
  MPU6050_CalibrateGyro(500);
  
  /* 初始化 Mahony 滤波器 (使用默认参数) */
  Mahony_InitDefault();
  
  /* Infinite loop */
  for(;;)
  {
    /* 触发 DMA 读取 MPU6050 */
    MPU6050_ReadDMA();
    
    /* 等待 DMA 完成 (通过信号量或短延时) */
    osDelay(1);
    
    /* 处理 DMA 数据 */
    MPU6050_ProcessDMAData();
    
    /* 磁力计更新 (100Hz，每 5 次 IMU 更新读取一次) */
    mag_divider++;
    if (mag_divider >= MAG_UPDATE_DIVIDER) {
      mag_divider = 0;
      if (g_qmc5883l_data.initialized) {
        QMC5883L_Read();
      }
    }
    
    /* Mahony 姿态解算 (自动融合磁力计数据) */
    Mahony_Update();
    
    /* 计数器 */
    imu_task_counter++;
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ctl_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  
  /* 控制任务：CRSF/ELRS 解析 + PID 计算 + 电机混控 (250Hz) */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  /* 初始化 CRSF (ELRS 接收机) */
  CRSF_Init();
  
  /* 初始化飞行控制 */
  FC_Init();
  
  /* 等待 IMU 初始化完成 */
  osDelay(500);
  
  /* Infinite loop */
  for(;;)
  {
    /* 处理 CRSF 数据 */
    CRSF_Process();
    
    /* 飞行控制主循环 */
    FC_Update();
    
    /* 计数器 */
    ctl_task_counter++;
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTL_TASK_PERIOD_MS));
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
* @brief Function implementing the log_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN StartLogTask */
  
  /* 日志任务：异步日志发送 (100Hz) */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  /* 初始化日志系统 */
  Log_Init();
  
  /* Infinite loop */
  for(;;)
  {
    /* 尝试启动 DMA 发送 */
    Log_StartDMATransmit();
    
    /* 定期输出飞行数据 */
    if (log_task_counter % 10 == 0) {  /* 每 100ms */
      Log_Printf("R:%.1f,P:%.1f,Y:%.1f,T:%.2f\r\n",
                 g_mahony.euler.roll,
                 g_mahony.euler.pitch,
                 g_mahony.euler.yaw,
                 g_rc_data.throttle);
    }
    
    /* 计数器 */
    log_task_counter++;
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
  }
  /* USER CODE END StartLogTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the comm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  
  /* 通信任务：OSD 刷新 + 上位机协议 (10Hz) */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  /* 初始化 OSD */
  AT7456E_Init();
  
  /* Infinite loop */
  for(;;)
  {
    /* 更新 OSD 显示 */
    AT7456E_ClearScreen();
    
    /* ===== 第一行：电池信息 ===== */
    /* 电压 */
    AT7456E_WriteFloat(1, 1, g_battery.voltage, 1);
    AT7456E_WriteString(5, 1, "V");
    
    /* 电流 */
    AT7456E_WriteFloat(7, 1, g_battery.current, 1);
    AT7456E_WriteString(11, 1, "A");
    
    /* 已消耗 mAh */
    AT7456E_WriteInt(14, 1, (int)g_battery.mah_drawn, 0);
    AT7456E_WriteString(18, 1, "mAh");
    
    /* ===== 第二行：姿态信息 ===== */
    AT7456E_WriteString(1, 2, "R:");
    AT7456E_WriteFloat(3, 2, g_mahony.euler.roll, 1);
    
    AT7456E_WriteString(10, 2, "P:");
    AT7456E_WriteFloat(12, 2, g_mahony.euler.pitch, 1);
    
    /* ===== 第三行：飞行状态 ===== */
    /* 解锁状态 */
    if (FC_IsArmed()) {
      AT7456E_WriteString(1, 3, "ARMED");
    } else {
      AT7456E_WriteString(1, 3, "DISARM");
    }
    
    /* 飞行模式 */
    switch (FC_GetMode()) {
      case FC_MODE_STABILIZE:
        AT7456E_WriteString(10, 3, "STAB");
        break;
      case FC_MODE_ACRO:
        AT7456E_WriteString(10, 3, "ACRO");
        break;
      default:
        AT7456E_WriteString(10, 3, "----");
        break;
    }
    
    /* RSSI 信号强度 */
    AT7456E_WriteString(16, 3, "RSSI:");
    AT7456E_WriteInt(21, 3, g_rc_data.rssi, 0);
    
    /* ===== 第四行：警告信息 ===== */
    if (g_battery.status == BATTERY_CRITICAL) {
      AT7456E_WriteString(1, 4, "!!! LOW BATTERY !!!");
    } else if (g_battery.status == BATTERY_OVERCURRENT) {
      AT7456E_WriteString(1, 4, "!!! OVERCURRENT !!!");
    } else if (g_fc_data.failsafe_active) {
      AT7456E_WriteString(1, 4, "!!! FAILSAFE !!!");
    } else if (g_battery.status == BATTERY_WARNING) {
      AT7456E_WriteString(1, 4, "LOW BATT WARNING");
    }
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the gps_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  
  /* GPS 任务：NMEA 解析 + 位置计算 (10Hz) */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  /* 初始化 GPS */
  GPS_Init();
  
  /* Infinite loop */
  for(;;)
  {
    /* 处理 GPS 数据 */
    GPS_Process();
    
    /* 周期性延时 */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GPS_TASK_PERIOD_MS));
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 更新所有 LED 状态 (根据系统状态自动设置)
 */
static void LED_UpdateAll(void)
{
  /* ===== LED_SYS: 系统状态 ===== */
  if (FC_IsArmed()) {
    if (g_fc_data.state == FC_STATE_FLYING) {
      LED_SetSysState(SYS_STATE_FLYING);
    } else {
      LED_SetSysState(SYS_STATE_ARMED);
    }
  } else {
    LED_SetSysState(SYS_STATE_STANDBY);
  }
  
  /* ===== LED_ERR: 错误/警告状态 ===== */
  if (g_fc_data.failsafe_active) {
    LED_SetErrState(ERR_STATE_CRITICAL);
  } else if (g_battery.status == BATTERY_CRITICAL) {
    LED_SetErrState(ERR_STATE_CRITICAL);
  } else if (g_battery.status == BATTERY_OVERCURRENT) {
    LED_SetErrState(ERR_STATE_SENSOR_FAIL);
  } else if (g_battery.status == BATTERY_WARNING) {
    LED_SetErrState(ERR_STATE_LOW_BATTERY);
  } else {
    LED_SetErrState(ERR_STATE_NONE);
  }
  
  /* ===== LED_GPS: GPS 状态 ===== */
  if (g_gps_data.fix_type >= 3 && g_gps_data.satellites >= 6) {
    LED_SetGpsState(GPS_STATE_LOCKED);
  } else if (g_gps_data.satellites > 0) {
    LED_SetGpsState(GPS_STATE_SEARCHING);
  } else {
    LED_SetGpsState(GPS_STATE_NONE);
  }
  
  /* ===== LED_RX: 遥控接收状态 ===== */
  if (CRSF_IsFailsafe()) {
    LED_SetRxState(RX_STATE_FAILSAFE);
  } else if (g_rc_data.rssi < 30) {
    LED_SetRxState(RX_STATE_WEAK);
  } else {
    LED_SetRxState(RX_STATE_OK);
  }
  
  /* ===== LED_MODE: 飞行模式 ===== */
  switch (FC_GetMode()) {
    case FC_MODE_ACRO:
      LED_SetModeState(MODE_STATE_ACRO);
      break;
    case FC_MODE_RTH:
      LED_SetModeState(MODE_STATE_RTH);
      break;
    default:
      LED_SetModeState(MODE_STATE_ANGLE);
      break;
  }
  
  /* ===== LED_LOG: 黑匣子状态 ===== */
  /* TODO: 根据 W25QXX 写入状态更新 */
  /* 暂时保持 IDLE */
  
  /* ===== LED_VTX: 图传状态 ===== */
  /* TODO: 根据 SmartAudio PIT 模式更新 */
  /* 暂时保持 NORMAL */
}

/**
 * @brief 电池状态检测与蜂鸣器报警
 */
static void Battery_Check(void)
{
  static uint8_t beep_counter = 0;
  
  /* 根据电池状态设置报警模式 */
  Battery_Status_t status = Battery_GetStatus();
  
  switch (status) {
    case BATTERY_OVERCURRENT:
      buzzer_mode = BUZZER_OVERCURRENT;
      break;
    case BATTERY_CRITICAL:
      buzzer_mode = BUZZER_CRITICAL;
      break;
    case BATTERY_WARNING:
      buzzer_mode = BUZZER_WARNING;
      break;
    default:
      buzzer_mode = BUZZER_OFF;
      break;
  }
  
  /* 蜂鸣器控制 */
  beep_counter++;
  
  switch (buzzer_mode) {
    case BUZZER_OFF:
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      break;
      
    case BUZZER_WARNING:
      /* 低电压警告：每 2 秒响 0.5 秒 (20 次循环) */
      if (beep_counter % 20 < 5) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      }
      break;
      
    case BUZZER_CRITICAL:
      /* 电压临界：每 0.5 秒响 0.25 秒 (5 次循环) */
      if (beep_counter % 5 < 2) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      }
      break;
      
    case BUZZER_OVERCURRENT:
      /* 过流：连续鸣叫 */
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      break;
  }
  
  /* 失控保护也触发报警 */
  if (g_fc_data.failsafe_active) {
    /* 失控：快速交替鸣叫 */
    if (beep_counter % 2 == 0) {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
  }
}

/* USER CODE END Application */

