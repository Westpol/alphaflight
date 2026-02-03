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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_cordic.h"
#include "stm32h7xx_ll_crc.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

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
#define BARO_EXTI_Pin LL_GPIO_PIN_3
#define BARO_EXTI_GPIO_Port GPIOE
#define BARO_CS_Pin LL_GPIO_PIN_4
#define BARO_CS_GPIO_Port GPIOE
#define MAGNETO_CS_Pin LL_GPIO_PIN_13
#define MAGNETO_CS_GPIO_Port GPIOC
#define MAGNETO_EXTI_Pin LL_GPIO_PIN_0
#define MAGNETO_EXTI_GPIO_Port GPIOC
#define IMU_EXTI_2_Pin LL_GPIO_PIN_4
#define IMU_EXTI_2_GPIO_Port GPIOA
#define IMU_CS_Pin LL_GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOC
#define V_SENSE_Pin LL_GPIO_PIN_5
#define V_SENSE_GPIO_Port GPIOC
#define CUR_SENSE_Pin LL_GPIO_PIN_0
#define CUR_SENSE_GPIO_Port GPIOB
#define IMU_EXTI_1_Pin LL_GPIO_PIN_2
#define IMU_EXTI_1_GPIO_Port GPIOB
#define SD_CARD_DETECT_Pin LL_GPIO_PIN_12
#define SD_CARD_DETECT_GPIO_Port GPIOD
#define RGB_B_Pin LL_GPIO_PIN_13
#define RGB_B_GPIO_Port GPIOD
#define RGB_G_Pin LL_GPIO_PIN_14
#define RGB_G_GPIO_Port GPIOD
#define RGB_R_Pin LL_GPIO_PIN_15
#define RGB_R_GPIO_Port GPIOD
#define POWER_GOOD_EXTI_Pin LL_GPIO_PIN_10
#define POWER_GOOD_EXTI_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
