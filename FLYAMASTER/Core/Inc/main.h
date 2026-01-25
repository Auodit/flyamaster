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
#define Power_Key_Pin GPIO_PIN_0
#define Power_Key_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define Optical_Flow_Pin GPIO_PIN_12
#define Optical_Flow_GPIO_Port GPIOC
#define Optical_FlowD2_Pin GPIO_PIN_2
#define Optical_FlowD2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* SPI 片选引脚定义 (根据硬件设计文档 08_hardware_design.md v2.5.7) */
/* 注意: CubeMX 生成的 FLASH_CS 在 PA4，但设计文档要求:
 *   - PA4 = OSD_CS (AT7456E OSD 芯片)
 *   - PC4 = FLASH_CS (W25Q128 Flash)
 * 如果 PCB 按设计文档布线，需要在 CubeMX 中修正引脚标签
 */
#define OSD_CS_Pin GPIO_PIN_4
#define OSD_CS_GPIO_Port GPIOC

/* LED 引脚定义 (根据硬件设计文档 08_hardware_design.md) */
#define LED_BLUE_Pin GPIO_PIN_3
#define LED_BLUE_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOC

/* 蜂鸣器引脚定义 */
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOC

/* 电源控制引脚定义 */
#define PWR_HOLD_Pin GPIO_PIN_1
#define PWR_HOLD_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
