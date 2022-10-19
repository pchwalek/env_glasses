/*
 * ppg.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_BME_H_
#define INC_BME_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif


//extern osThreadId_t bmeTaskHandle;
extern osTimerId_t periodicBMETimer_id;


//const osThreadAttr_t bmeTask_attributes = {
//	.name = "bmeTask",
//	.attr_bits = osThreadDetached,
//	.cb_mem = NULL,
//	.cb_size = 0,
//	.stack_mem = NULL,
//	.stack_size = 128 * 6,
//	.priority = (osPriority_t) osPriorityNormal,
//	.tz_module = 0,
//	.reserved = 0
//  };

void BME_Task(void *argument);
void saveBME_StateConfig();
void recoverBME_StateConfig();

#ifdef __cplusplus
}
#endif

#endif /* INC_PPG_H_ */
