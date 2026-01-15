/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
    GPIO_InitStruct.Pin = BATTERY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATTERY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
    HAL_GPIO_DeInit(BATTERY_GPIO_Port, BATTERY_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief 读取电池电压
 * @return 电池电压 (V)
 */
float Battery_GetVoltage(void) {
    static float last_valid_voltage = 11.1f;  // 默认3S满电电压
    uint32_t adc_raw = 0;
    float voltage;
    
    // 启动 ADC 转换
    HAL_ADC_Start(&hadc1);
    
    // Risk #052 修复：检查转换是否成功
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return last_valid_voltage;  // 转换失败，返回上次有效值
    }
    
    adc_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    // 计算实际电压
    // V_adc = adc_raw / 4096 * 3.3V
    // V_battery = V_adc * 分压比
    voltage = (float)adc_raw / ADC_RESOLUTION * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
    
    // Risk #052 修复：合理性检查 (2S~4S范围: 6V~16.8V)
    if (voltage >= 6.0f && voltage <= 16.8f) {
        last_valid_voltage = voltage;
    }
    
    return last_valid_voltage;
}

/**
 * @brief 检测电池是否低电压
 * @param voltage 当前电压
 * @param cell_count 电池节数 (2/3/4)
 * @return 1=低电压, 0=正常
 */
uint8_t Battery_IsLow(float voltage, uint8_t cell_count) {
    float threshold;
    
    switch (cell_count) {
        case 2:  threshold = LOW_VOLTAGE_2S; break;
        case 3:  threshold = LOW_VOLTAGE_3S; break;
        case 4:  threshold = LOW_VOLTAGE_4S; break;
        default: threshold = LOW_VOLTAGE_3S; break;
    }
    
    return (voltage < threshold) ? 1 : 0;
}

/**
 * @brief 检测电池是否危急电压
 * @param voltage 当前电压
 * @param cell_count 电池节数 (2/3/4)
 * @return 1=危急, 0=正常
 */
uint8_t Battery_IsCritical(float voltage, uint8_t cell_count) {
    float threshold;
    
    switch (cell_count) {
        case 2:  threshold = CRITICAL_VOLTAGE_2S; break;
        case 3:  threshold = CRITICAL_VOLTAGE_3S; break;
        case 4:  threshold = CRITICAL_VOLTAGE_4S; break;
        default: threshold = CRITICAL_VOLTAGE_3S; break;
    }
    
    return (voltage < threshold) ? 1 : 0;
}

/* USER CODE END 1 */
