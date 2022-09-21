/*
 * ppg.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ADDR 0x68

//extern osThreadId_t imuTaskHandle;
extern osTimerId_t periodicIMUTimer_id;
//const osThreadAttr_t imuTask_attributes = {
//	.name = "imuTask",
//	.attr_bits = osThreadDetached,
//	.cb_mem = NULL,
//	.cb_size = 0,
//	.stack_mem = NULL,
//	.stack_size = 128 * 8,
//	.priority = (osPriority_t) osPriorityNormal,
//	.tz_module = 0,
//	.reserved = 0
//  };

void IMU_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_PPG_H_ */
