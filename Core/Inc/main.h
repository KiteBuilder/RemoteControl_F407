/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "mb.h"
#include "mbport.h"
#include "mt_port.h"
#include "mbutils.h"
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
#define MANUAL_LED_Pin GPIO_PIN_14
#define MANUAL_LED_GPIO_Port GPIOG
#define RXEN_L_Pin GPIO_PIN_3
#define RXEN_L_GPIO_Port GPIOB
#define UART_MOD_H_Pin GPIO_PIN_7
#define UART_MOD_H_GPIO_Port GPIOD
#define TE_485_L_Pin GPIO_PIN_12
#define TE_485_L_GPIO_Port GPIOC
#define HDPLX_H_Pin GPIO_PIN_15
#define HDPLX_H_GPIO_Port GPIOA
#define STOP_KEY_Pin GPIO_PIN_5
#define STOP_KEY_GPIO_Port GPIOE
#define RETURN_KEY_Pin GPIO_PIN_6
#define RETURN_KEY_GPIO_Port GPIOE
#define DXEN_H_Pin GPIO_PIN_10
#define DXEN_H_GPIO_Port GPIOG
#define START_KEY_Pin GPIO_PIN_7
#define START_KEY_GPIO_Port GPIOI
#define PRELOAD_KEY_Pin GPIO_PIN_6
#define PRELOAD_KEY_GPIO_Port GPIOI
#define STATUS_LED_Pin GPIO_PIN_9
#define STATUS_LED_GPIO_Port GPIOG
#define EN_5EXT_Pin GPIO_PIN_2
#define EN_5EXT_GPIO_Port GPIOI
#define OSC_En_Pin GPIO_PIN_15
#define OSC_En_GPIO_Port GPIOC
#define CHOCK_LED_Pin GPIO_PIN_7
#define CHOCK_LED_GPIO_Port GPIOF
#define OLED_RS_Pin GPIO_PIN_9
#define OLED_RS_GPIO_Port GPIOF
#define OLED_DB7_Pin GPIO_PIN_12
#define OLED_DB7_GPIO_Port GPIOF
#define OLED_DB5_Pin GPIO_PIN_15
#define OLED_DB5_GPIO_Port GPIOF
#define LOCK_KEY_Pin GPIO_PIN_11
#define LOCK_KEY_GPIO_Port GPIOE
#define FAILURE_LED_Pin GPIO_PIN_9
#define FAILURE_LED_GPIO_Port GPIOD
#define START_LED_Pin GPIO_PIN_8
#define START_LED_GPIO_Port GPIOD
#define OLED_RW_Pin GPIO_PIN_7
#define OLED_RW_GPIO_Port GPIOA
#define OLED_E_Pin GPIO_PIN_1
#define OLED_E_GPIO_Port GPIOB
#define OLED_DB4_Pin GPIO_PIN_0
#define OLED_DB4_GPIO_Port GPIOB
#define WINDOW_KEY_Pin GPIO_PIN_11
#define WINDOW_KEY_GPIO_Port GPIOF
#define OLED_DB6_Pin GPIO_PIN_7
#define OLED_DB6_GPIO_Port GPIOE
#define LedOut_Pin GPIO_PIN_10
#define LedOut_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define MODBUS_SLAVE_ADDRESS     0x03 //Modbus slave address(ID)

#define REG_COILS_START          1
#define REG_COILS_NREGS          4

#define REG_DISCRETE_COILS_START 1
#define REG_DISCRETE_NREGS       5

#define REG_INPUT_START          1
#define REG_INPUT_NREGS          7

#define REG_HOLDING_START        1
#define REG_HOLDING_NREGS        7
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
