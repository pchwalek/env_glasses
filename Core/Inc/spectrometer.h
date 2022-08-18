/*
 * ppg.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_SPECTROMETER_H_
#define INC_SPECTROMETER_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPEC_ADDR 0x39

//extern osThreadId_t specTaskHandle;
extern osTimerId_t periodicSpecTimer_id;
//const osThreadAttr_t specTask_attributes = {
//	.name = "specTask",
//	.attr_bits = osThreadDetached,
//	.cb_mem = NULL,
//	.cb_size = 0,
//	.stack_mem = NULL,
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal,
//	.tz_module = 0,
//	.reserved = 0
//  };

void Spec_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_PPG_H_ */
