/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "sgp.h"
#include "SensirionI2CSgp41.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"
#include "sensirion_gas_index_algorithm.h"
#include "sht.h"

#define SGP_SAMPLE_SYS_PERIOD_MS		1000 //how often do we want the system to sample
#define SEND_SGP_EVERY_X_S				1
#define MAX_SGP_SAMPLES_PACKET	int((SEND_SGP_EVERY_X_S*1000)/SGP_SAMPLE_SYS_PERIOD_MS)

typedef struct sgpSamples {
	uint16_t srawVoc;
	uint16_t srawNox;
	int32_t voc_index_value;
	int32_t nox_index_value;
	uint32_t timestamp;
} sgpSample;


static void triggerSgpSample(void *argument);
static sgpSample sgpData[MAX_SGP_SAMPLES_PACKET];



static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicSgpTimer_id;

SensirionI2CSgp41 sgp41;
GasIndexAlgorithmParams paramsNox;
GasIndexAlgorithmParams paramsVoc;



void SgpTask(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;
	uint16_t error;

    uint16_t defaultRh = 0x8000;
    uint16_t defaultT = 0x6666;

    uint16_t _Rh = 0;
	uint16_t _T = 0;

	int32_t voc_index_value, nox_index_value;


    GasIndexAlgorithm_init(&paramsVoc, (int32_t) GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&paramsNox, (int32_t) GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	if (!sgp41.begin(&hi2c1)) {
		osDelay(100);
	}

    uint16_t serialNumber[3];
    uint8_t serialNumberSize = 3;
    error = sgp41.getSerialNumber(serialNumber, serialNumberSize);
    while (error) {
    	osSemaphoreRelease(messageI2C1_LockHandle);
		osDelay(10);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
		error = sgp41.getSerialNumber(serialNumber, serialNumberSize);
    }

    uint16_t testResult;
    error = sgp41.executeSelfTest(testResult);
    while(error) {
    	osSemaphoreRelease(messageI2C1_LockHandle);
      	osDelay(10);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
        error = sgp41.executeSelfTest(testResult);
      }
    if(testResult != 0xD400){
    	osSemaphoreRelease(messageI2C1_LockHandle);
    	while(1){
    		osDelay(10000); // TODO: terminate thread instead
    	}
    }

	header.payloadLength = MAX_SGP_SAMPLES_PACKET * sizeof(sgpSample);

	uint16_t sgpIdx = 0;
	uint32_t sgpID = 0;

	uint32_t sgpSample;

	// time in seconds needed for NOx conditioning
	uint16_t conditioning_s = 10;

	osSemaphoreRelease(messageI2C1_LockHandle);
	periodicSgpTimer_id = osTimerNew(triggerSgpSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicSgpTimer_id, SGP_SAMPLE_SYS_PERIOD_MS);


	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
			if(timeLeftForSample < SGP_SAMPLE_SYS_PERIOD_MS){
				osDelay(timeLeftForSample);
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);


			if (conditioning_s > 0) {
				// During NOx conditioning (10s) SRAW NOx will remain 0
				if(shtTemp != -1 && shtHum != -1){
					_Rh = (65535.0 / 100) * shtHum;
					_T = (65535.0 / 175) * (shtTemp+45);
					error = sgp41.executeConditioning(_Rh, _T, sgpData[sgpIdx].srawVoc);
				}else{
					error = sgp41.executeConditioning(defaultRh, defaultT, sgpData[sgpIdx].srawVoc);
				}
				sgpData[sgpIdx].srawNox = 0;
			} else {
				// Read Measurement
				if(shtTemp != -1 && shtHum != -1){
					_Rh = (65535.0 / 100) * shtHum;
					_T = (65535.0 / 175) * (shtTemp+45);
					error = sgp41.measureRawSignals(_Rh, _T, sgpData[sgpIdx].srawVoc, sgpData[sgpIdx].srawNox);
				}else{
					error = sgp41.measureRawSignals(defaultRh, defaultT, sgpData[sgpIdx].srawVoc, sgpData[sgpIdx].srawNox);
				}
			}
			osSemaphoreRelease(messageI2C1_LockHandle);

			if(error){
				continue;
			}

			if (conditioning_s > 0)	{
		        conditioning_s--;
			}else{
				GasIndexAlgorithm_process(&paramsNox, sgpData[sgpIdx].srawNox, &sgpData[sgpIdx].nox_index_value);
				GasIndexAlgorithm_process(&paramsVoc, sgpData[sgpIdx].srawVoc, &sgpData[sgpIdx].voc_index_value);
			}

			sgpData[sgpIdx].timestamp = HAL_GetTick();

			sgpIdx++;

			if (sgpIdx >= MAX_SGP_SAMPLES_PACKET) {
				header.packetType = SGP;
				header.packetID = sgpID;
				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {
					memcpy(&(packet->header), &header, sizeof(PacketHeader));
					memcpy(packet->payload, sgpData, header.payloadLength);
					queueUpPacket(packet);
				}
				sgpID++;
				sgpIdx = 0;
			}

			timeLeftForSample = HAL_GetTick();
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicSgpTimer_id);
			break;
		}
	}
}

static void triggerSgpSample(void *argument) {
	osThreadFlagsSet(sgpTaskHandle, GRAB_SAMPLE_BIT);
}
