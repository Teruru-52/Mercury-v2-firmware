/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

GPIO_Value LED_FRONT_RIGHT1 = {LED2_GPIO_Port, LED2_Pin};
GPIO_Value LED_FRONT_RIGHT2 = {LED3_GPIO_Port, LED3_Pin};
GPIO_Value LED_FRONT_RIGHT3 = {LED1_GPIO_Port, LED1_Pin};
GPIO_Value LED_FRONT_LEFT1 = {LED5_GPIO_Port, LED5_Pin};
GPIO_Value LED_FRONT_LEFT2 = {LED4_GPIO_Port, LED4_Pin};
GPIO_Value LED_FRONT_LEFT3 = {LED6_GPIO_Port, LED6_Pin};
GPIO_Value LED_GREEN = {LED7_GPIO_Port, LED7_Pin};
GPIO_Value LED_TALE_LEFT = {LED8_GPIO_Port, LED8_Pin};
GPIO_Value LED_RED = {LED9_GPIO_Port, LED9_Pin};
GPIO_Value LED_TALE_RIGHT = {LED10_GPIO_Port, LED10_Pin};

GPIO_Value USER_SW = {SW_GPIO_Port, SW_Pin};
GPIO_Value SPI1_CS = {SPI1_CS_GPIO_Port, SPI1_CS_Pin};
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LED10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin
                          |LED8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin
                          |LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SDIO_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDIO_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
GPIO_PinState Read_GPIO(GPIO_Value GPIO) {
  return HAL_GPIO_ReadPin(GPIO.GPIOx, GPIO.GPIO_PIN_x);
}

void Toggle_GPIO(GPIO_Value GPIO) {
  HAL_GPIO_TogglePin(GPIO.GPIOx, GPIO.GPIO_PIN_x);
}

void Write_GPIO(GPIO_Value GPIO, GPIO_PinState PinState) {
  HAL_GPIO_WritePin(GPIO.GPIOx, GPIO.GPIO_PIN_x, PinState);
}

// void SetTxLED(GPIO_PinState PinState)
// {
//   Write_GPIO(LED_CAN_TX, PinState);
// }

// void ActivateTxLED(void)
// {
//   if (BLMD_Access_Lamp.FDCAN_TX == ENABLE)
//   {
//     Write_GPIO(LED_CAN_TX, GPIO_PIN_SET);
//   }
// }

// void ResetTxLED(void)
// {
//   BLMD_Access_Lamp.FDCAN_TX = DISABLE;
// }

// void SetRxLED(GPIO_PinState PinState)
// {
//   Write_GPIO(LED_CAN_RX, PinState);
// }

// void ActivateRxLED(void)
// {
//   if (BLMD_Access_Lamp.FDCAN_RX == ENABLE)
//   {
//     Write_GPIO(LED_CAN_RX, GPIO_PIN_SET);
//   }
// }

// void ResetRxLED(void)
// {
//   BLMD_Access_Lamp.FDCAN_RX = DISABLE;
// }
/* USER CODE END 2 */
