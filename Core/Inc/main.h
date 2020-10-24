/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define EN_PACK_SENSE_Pin GPIO_PIN_13
#define EN_PACK_SENSE_GPIO_Port GPIOC
#define CHARGER_DISABLE_Pin GPIO_PIN_14
#define CHARGER_DISABLE_GPIO_Port GPIOC
#define CHARGER_PLUGGED_Pin GPIO_PIN_15
#define CHARGER_PLUGGED_GPIO_Port GPIOC
#define CHARGER_PG_Pin GPIO_PIN_0
#define CHARGER_PG_GPIO_Port GPIOA
#define GAIN_Pin GPIO_PIN_1
#define GAIN_GPIO_Port GPIOA
#define GAIN_BASS_Pin GPIO_PIN_2
#define GAIN_BASS_GPIO_Port GPIOA
#define GAIN_MIDDLE_Pin GPIO_PIN_3
#define GAIN_MIDDLE_GPIO_Port GPIOA
#define GAIN_TREBLE_Pin GPIO_PIN_4
#define GAIN_TREBLE_GPIO_Port GPIOA
#define PACK_VOLTAGE_Pin GPIO_PIN_5
#define PACK_VOLTAGE_GPIO_Port GPIOA
#define TEMP_SENSOR1_Pin GPIO_PIN_6
#define TEMP_SENSOR1_GPIO_Port GPIOA
#define LEVELMETER_Pin GPIO_PIN_7
#define LEVELMETER_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_0
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_2
#define GPIO3_GPIO_Port GPIOB
#define GPIO4_Pin GPIO_PIN_12
#define GPIO4_GPIO_Port GPIOB
#define MUTE_AMP2_Pin GPIO_PIN_13
#define MUTE_AMP2_GPIO_Port GPIOB
#define DIAG_AMP2_Pin GPIO_PIN_14
#define DIAG_AMP2_GPIO_Port GPIOB
#define STEPUP_DISABLE_Pin GPIO_PIN_15
#define STEPUP_DISABLE_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_8
#define STATUS_LED_GPIO_Port GPIOA
#define CHARGER_STAT1_Pin GPIO_PIN_11
#define CHARGER_STAT1_GPIO_Port GPIOA
#define BT_FAULT_Pin GPIO_PIN_12
#define BT_FAULT_GPIO_Port GPIOA
#define CHARGER_STAT2_Pin GPIO_PIN_6
#define CHARGER_STAT2_GPIO_Port GPIOF
#define BT_EN_Pin GPIO_PIN_7
#define BT_EN_GPIO_Port GPIOF
#define ENABLE_9V_Pin GPIO_PIN_15
#define ENABLE_9V_GPIO_Port GPIOA
#define STBY_AMP2_Pin GPIO_PIN_3
#define STBY_AMP2_GPIO_Port GPIOB
#define POWER_Pin GPIO_PIN_4
#define POWER_GPIO_Port GPIOB
#define U3V3_ENABLED_Pin GPIO_PIN_5
#define U3V3_ENABLED_GPIO_Port GPIOB
#define RST_BT_Pin GPIO_PIN_6
#define RST_BT_GPIO_Port GPIOB
#define MUTE_AMP1_Pin GPIO_PIN_7
#define MUTE_AMP1_GPIO_Port GPIOB
#define STBY_AMP1_Pin GPIO_PIN_8
#define STBY_AMP1_GPIO_Port GPIOB
#define DIAG_AMP1_Pin GPIO_PIN_9
#define DIAG_AMP1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
