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
#include "spectrometer.h"
#include "lux.h"
#include "bme.h"
#include "imu.h"
#include "blink.h"
#include "packet.h"
#include "sht.h"
#include "sgp.h"
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

osThreadId_t specTaskHandle;
const osThreadAttr_t specTask_attributes = {
	.name = "spectrometerTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 1,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t luxTaskHandle;
const osThreadAttr_t luxTask_attributes = {
	.name = "luxTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 1,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t shtTaskHandle;
const osThreadAttr_t shtTask_attributes = {
	.name = "shtTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 1,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t bmeTaskHandle;
const osThreadAttr_t bmeTask_attributes = {
	.name = "bmeTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 6,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t sgpTaskHandle;
const osThreadAttr_t sgpTask_attributes = {
	.name = "sgpTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 6,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
	.name = "imuTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 6,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t blinkTaskHandle;
const osThreadAttr_t blinkTask_attributes = {
	.name = "blinkTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 512 * 2,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

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
	.stack_size = 512 * 2,
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
	.stack_size = 512 * 2,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

osThreadId_t senderTaskHandle;
const osThreadAttr_t senderTask_attributes = {
	.name = "senderTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityAboveNormal,
	.tz_module = 0,
	.reserved = 0
  };

/* Definitions for messageI2C_Lock */
osMutexId_t messageI2C1_LockHandle;
const osMutexAttr_t messageI2C1_Lock_attributes = {
  .name = "messageI2C1_Lock"
};

osMessageQueueId_t packet_QueueHandle;
const osMessageQueueAttr_t packetQueue_attributes =
		{ .name = "packetQueue" };

osMessageQueueId_t packetAvail_QueueHandle;
const osMessageQueueAttr_t packetAvailQueue_attributes =
		{ .name = "packetAvail" };

#ifdef KEEP_GENERATED_FREERTOS
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for frontLightsThre */
osThreadId_t frontLightsThreHandle;
const osThreadAttr_t frontLightsThre_attributes = {
  .name = "frontLightsThre",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};
/* Definitions for thermopileTask */
osThreadId_t thermopileTaskHandle;
const osThreadAttr_t thermopileTask_attributes = {
  .name = "thermopileTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for messageI2C1_Lock */
osMutexId_t messageI2C1_LockHandle;
const osMutexAttr_t messageI2C1_Lock_attributes = {
  .name = "messageI2C1_Lock"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#endif
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
  /* creation of messageI2C1_Lock */
  messageI2C1_LockHandle = osMutexNew(&messageI2C1_Lock_attributes);

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
  frontLightsThreHandle = osThreadNew(ThreadFrontLightsComplexTask, NULL, &frontLightsThre_attributes);

//  /* creation of thermopileTask */
//  thermopileTaskHandle = osThreadNew(Thermopile_Task, NULL, &thermopileTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	packet_QueueHandle = osMessageQueueNew(MAX_PACKET_QUEUE_SIZE, sizeof(SensorPacket *),
			&packetQueue_attributes);

	packetAvail_QueueHandle = osMessageQueueNew(MAX_PACKET_QUEUE_SIZE, sizeof(SensorPacket *),
				&packetAvailQueue_attributes);

//  specTaskHandle = osThreadNew(Spec_Task, NULL, &specTask_attributes);
//  luxTaskHandle = osThreadNew(LuxTask, NULL, &luxTask_attributes);
//  bmeTaskHandle = osThreadNew(BME_Task, NULL, &bmeTask_attributes);
//  imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);
//  blinkTaskHandle = osThreadNew(BlinkTask, NULL, &blinkTask_attributes);
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
//	osDelay(1000);
	while(1){
//		ledStartupSequence();
//		osDelay(5000);
		osDelay(1);
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
#ifdef __cplusplus
extern "C" {
#endif
void startThreads(){


	senderTaskHandle = osThreadNew(senderThread, NULL, &senderTask_attributes);
//
	/* start sensor subsystem threads */
	blinkTaskHandle = osThreadNew(BlinkTask, NULL, &blinkTask_attributes);
//
//	//  /* creation of thermopileTask */
	thermopileTaskHandle = osThreadNew(Thermopile_Task, NULL, &thermopileTask_attributes);

	shtTaskHandle = osThreadNew(ShtTask, NULL, &shtTask_attributes);
	sgpTaskHandle = osThreadNew(SgpTask, NULL, &sgpTask_attributes);
	bmeTaskHandle = osThreadNew(BME_Task, NULL, &bmeTask_attributes);


	  //imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);

	  //	  luxTaskHandle = osThreadNew(LuxTask, NULL, &luxTask_attributes);


//	  /* creation of frontLightsThre */
//	  frontLightsThreHandle = osThreadNew(ThreadFrontLightsComplexTask, NULL, &frontLightsThre_attributes);
	//


}
#ifdef __cplusplus
}
#endif
/* USER CODE END Application */

