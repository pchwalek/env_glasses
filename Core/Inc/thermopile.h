/*
 * ppg.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_THERMOPILE_H_
#define INC_THERMOPILE_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TP_ADDR 0x0C
//#define TP_OUTER_ADDR 0x0D

extern osThreadId_t thermopileTaskHandle;
extern osTimerId_t periodicThermopileTimer_id;
//const osThreadAttr_t thermopileTask_attributes = {
//	.name = "thermTask",
//	.attr_bits = osThreadDetached,
//	.cb_mem = NULL,
//	.cb_size = 0,
//	.stack_mem = NULL,
//	.stack_size = 128 * 4,
//	.priority = (osPriority_t) osPriorityNormal,
//	.tz_module = 0,
//	.reserved = 0
//  };

void Thermopile_Task(void *argument);
static void triggerThermopileSample (void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_PPG_H_ */
