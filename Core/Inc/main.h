/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

void startThreads(void);
void startInitThread(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLINK_SIG_Pin GPIO_PIN_1
#define BLINK_SIG_GPIO_Port GPIOA
#define MEM_WP_Pin GPIO_PIN_11
#define MEM_WP_GPIO_Port GPIOC
#define MEM_CS_Pin GPIO_PIN_15
#define MEM_CS_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_0
#define IMU_CS_GPIO_Port GPIOD
#define IMU_INT_Pin GPIO_PIN_2
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI2_IRQn
#define PA9_Pin GPIO_PIN_9
#define PA9_GPIO_Port GPIOA
#define BLINK_PWM_Pin GPIO_PIN_6
#define BLINK_PWM_GPIO_Port GPIOA
#define PD4_Pin GPIO_PIN_4
#define PD4_GPIO_Port GPIOD
#define PD9_Pin GPIO_PIN_9
#define PD9_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define CUSTOM_BT_PARAMETERS 1
//#define DYNAMIC_MODE	1

// FREERTOS THREAD SPECIFIC
#define GRAB_SAMPLE_BIT							0x0100
#define TERMINATE_THREAD_BIT					0x0200
#define IMU_DATA_RDY_BIT						0x0400


extern volatile unsigned long ulHighFrequencyTimerTicks;
extern volatile uint8_t sensorThreadsRunning;

typedef enum
{
    CFG_LPM_APP,
    CFG_LPM_APP_BLE,
    /* USER CODE BEGIN CFG_LPM_Id_t */

    /* USER CODE END CFG_LPM_Id_t */
} CFG_LPM_Id_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
