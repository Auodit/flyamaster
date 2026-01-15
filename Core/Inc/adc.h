/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

// 电池电压采集
// 假设使用分压电阻: 10K + 10K (1:2分压)
// 满量程 3.3V ADC 对应 6.6V 电池电压
#define VOLTAGE_DIVIDER_RATIO  2.0f   // 分压比
#define ADC_REF_VOLTAGE        3.3f   // 参考电压
#define ADC_RESOLUTION         4096   // 12位 ADC

// 低电压告警阈值 (单节锂电 3.5V * 2 = 7.0V 两节串联)
// 对于 4S 电池: 3.5V * 4 = 14.0V
#define LOW_VOLTAGE_2S         7.0f
#define LOW_VOLTAGE_3S         10.5f
#define LOW_VOLTAGE_4S         14.0f
#define CRITICAL_VOLTAGE_2S    6.4f
#define CRITICAL_VOLTAGE_3S    9.6f
#define CRITICAL_VOLTAGE_4S    12.8f

float Battery_GetVoltage(void);
uint8_t Battery_IsLow(float voltage, uint8_t cell_count);
uint8_t Battery_IsCritical(float voltage, uint8_t cell_count);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

