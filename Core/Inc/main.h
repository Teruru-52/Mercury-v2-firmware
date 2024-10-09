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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// #include "cmsis_os.h"
#include "adc.h"
// #include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_10
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_11
#define LED6_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_12
#define SW_GPIO_Port GPIOB
#define MOTOR_FAN_PH_Pin GPIO_PIN_13
#define MOTOR_FAN_PH_GPIO_Port GPIOB
#define SDIO_SW_Pin GPIO_PIN_10
#define SDIO_SW_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_11
#define LED7_GPIO_Port GPIOA
#define LED8_Pin GPIO_PIN_12
#define LED8_GPIO_Port GPIOA
#define LED9_Pin GPIO_PIN_15
#define LED9_GPIO_Port GPIOA
#define LED10_Pin GPIO_PIN_3
#define LED10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
