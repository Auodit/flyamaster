/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "mahony.h"
#include "filter.h"
#include "mpu6050.h"
#include "motor.h"
#include "optical_flow.h"
#include "protocol.h"
#include "sbus.h"
#include "adc.h"
#include "buzzer.h"
#include "altitude.h"
#include "spl06.h"
#include "qmc5883l.h"
#include "w25qxx.h"
#include "flash_params.h"
#include "ano_v7.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define BATTERY_Pin GPIO_PIN_0
#define BATTERY_GPIO_Port GPIOC
#define Optical_flow_sensor_Pin GPIO_PIN_0
#define Optical_flow_sensor_GPIO_Port GPIOA
#define Optical_flow_sensorA1_Pin GPIO_PIN_1
#define Optical_flow_sensorA1_GPIO_Port GPIOA
#define PI_Pin GPIO_PIN_2
#define PI_GPIO_Port GPIOA
#define PIA3_Pin GPIO_PIN_3
#define PIA3_GPIO_Port GPIOA
#define Flash_CS_Pin GPIO_PIN_4
#define Flash_CS_GPIO_Port GPIOA
#define FlashSCK_Pin GPIO_PIN_5
#define FlashSCK_GPIO_Port GPIOA
#define FlashMISO_Pin GPIO_PIN_6
#define FlashMISO_GPIO_Port GPIOA
#define FlashMOSI_Pin GPIO_PIN_7
#define FlashMOSI_GPIO_Port GPIOA
#define SPL06__Pin GPIO_PIN_10
#define SPL06__GPIO_Port GPIOB
#define SPL06_Pin GPIO_PIN_11
#define SPL06_GPIO_Port GPIOB
#define NRF24L01_Pin GPIO_PIN_12
#define NRF24L01_GPIO_Port GPIOB
#define NRF24L01B13_Pin GPIO_PIN_13
#define NRF24L01B13_GPIO_Port GPIOB
#define NRF24L01B14_Pin GPIO_PIN_14
#define NRF24L01B14_GPIO_Port GPIOB
#define NRF24L01B15_Pin GPIO_PIN_15
#define NRF24L01B15_GPIO_Port GPIOB
#define SBUS_Pin GPIO_PIN_8
#define SBUS_GPIO_Port GPIOD
#define SBUSD9_Pin GPIO_PIN_9
#define SBUSD9_GPIO_Port GPIOD
#define NRF24L01G8_Pin GPIO_PIN_8
#define NRF24L01G8_GPIO_Port GPIOG
#define RGB_LED_Pin GPIO_PIN_8
#define RGB_LED_GPIO_Port GPIOC
#define Debug_Pin GPIO_PIN_9
#define Debug_GPIO_Port GPIOA
#define DebugA10_Pin GPIO_PIN_10
#define DebugA10_GPIO_Port GPIOA
#define ANO_V7_Pin GPIO_PIN_12
#define ANO_V7_GPIO_Port GPIOC
#define ANO_V7D2_Pin GPIO_PIN_2
#define ANO_V7D2_GPIO_Port GPIOD
#define MPU6050_Pin GPIO_PIN_5
#define MPU6050_GPIO_Port GPIOB
#define MPU6050B6_Pin GPIO_PIN_6
#define MPU6050B6_GPIO_Port GPIOB
#define MPU6050B7_Pin GPIO_PIN_7
#define MPU6050B7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
