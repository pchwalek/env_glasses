/*
 * mic.h
 *
 *  Created on: Jan 4, 2022
 *      Author: patri
 */

#ifndef INC_MIC_H_
#define INC_MIC_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t micTaskHandle;
extern osTimerId_t periodicMicTimer_id;
const osThreadAttr_t micTask_attributes = {
	.name = "micTask",
	.attr_bits = osThreadDetached,
	.cb_mem = NULL,
	.cb_size = 0,
	.stack_mem = NULL,
	.stack_size = 128 * 8,
	.priority = (osPriority_t) osPriorityNormal,
	.tz_module = 0,
	.reserved = 0
  };

void Mic_Task(void *argument);


#ifdef __cplusplus
}
#endif

#endif /* INC_MIC_H_ */
