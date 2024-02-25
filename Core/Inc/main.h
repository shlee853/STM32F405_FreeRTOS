/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CLOCK_PER_USEC 72   //168
#define CLOCK_PER_MSEC 72000 //168000
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
#define ADC1_CM_Pin GPIO_PIN_1
#define ADC1_CM_GPIO_Port GPIOC
#define ADC2_VM_Pin GPIO_PIN_2
#define ADC2_VM_GPIO_Port GPIOC
#define SPI1_NSS_PIN_Pin GPIO_PIN_4
#define SPI1_NSS_PIN_GPIO_Port GPIOA
#define SPI1_SCK_PIN_Pin GPIO_PIN_5
#define SPI1_SCK_PIN_GPIO_Port GPIOA
#define SPI1_MISO_PIN_Pin GPIO_PIN_6
#define SPI1_MISO_PIN_GPIO_Port GPIOA
#define SPI1_MOSI_PIN_Pin GPIO_PIN_7
#define SPI1_MOSI_PIN_GPIO_Port GPIOA
#define SPI1_INT_Pin GPIO_PIN_4
#define SPI1_INT_GPIO_Port GPIOC
#define UART6_TX_DEBUG_Pin GPIO_PIN_6
#define UART6_TX_DEBUG_GPIO_Port GPIOC
#define UART6_RX_DEBUG_Pin GPIO_PIN_7
#define UART6_RX_DEBUG_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
