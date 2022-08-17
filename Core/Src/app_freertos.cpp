/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "captivate_config.h"
#include "lp5523.h"
#include "thermopile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t lightsComplexQueueHandle;
const osMessageQueueAttr_t lightsComplexQueue_attributes = { .name =
		"lightsComplexQueue" };
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };
/* Definitions for frontLightsThre */
osThreadId_t frontLightsThreHandle;
const osThreadAttr_t frontLightsThre_attributes = {
	.name = "frontLightsTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 4,
	.priority = (osPriority_t) osPriorityBelowNormal,
	.tz_module = 0,
	.reserved = 0
  };
/* Definitions for thermopileTask */
osThreadId_t thermopileTaskHandle;
const osThreadAttr_t thermopileTask_attributes = {
	.name = "thermopileTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 4,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };
/* Definitions for messageI2C_Lock */
osMutexId_t messageI2C_LockHandle;
const osMutexAttr_t messageI2C_Lock_attributes = {
  .name = "messageI2C_Lock"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ThreadFrontLightsComplexTask(void *argument);
void Thermopile_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of messageI2C_Lock */
  messageI2C_LockHandle = osMutexNew(&messageI2C_Lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	/* creation of lightsSimpleQueue */
	lightsComplexQueueHandle = osMessageQueueNew(3, sizeof(union ColorComplex),
			&lightsComplexQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of frontLightsThre */
//  frontLightsThreHandle = osThreadNew(ThreadFrontLightsComplexTask, NULL, &frontLightsThre_attributes);

  /* creation of thermopileTask */
  thermopileTaskHandle = osThreadNew(Thermopile_Task, NULL, &thermopileTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

	while(1){
		ledStartupSequence();
		osDelay(1500);
	}
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ThreadFrontLightsComplexTask */
/**
* @brief Function implementing the frontLightsThre thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ThreadFrontLightsComplexTask */
__weak void ThreadFrontLightsComplexTask(void *argument)
{
  /* USER CODE BEGIN ThreadFrontLightsComplexTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ThreadFrontLightsComplexTask */
}

/* USER CODE BEGIN Header_Thermopile_Task */
/**
* @brief Function implementing the thermopileTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thermopile_Task */
__weak void Thermopile_Task(void *argument)
{
  /* USER CODE BEGIN Thermopile_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Thermopile_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

