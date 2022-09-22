/*
 * ppg.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_SGP_H_
#define INC_SGP_H_

#include "cmsis_os2.h"


#ifdef __cplusplus
extern "C" {
#endif


extern osTimerId_t periodicSgpTimer_id;

void SgpTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_PPG_H_ */
