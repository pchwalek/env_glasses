/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "spectrometer.h"
#include "Adafruit_AS7341.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"

#define SPEC_ATIME								100
#define SPEC_ASTEP								999
#define SPEC_SAMPLE_PERIOD_MS			(uint32_t)(((SPEC_ATIME+1)*(SPEC_ASTEP+1)*2.78)/1000)
#define SPEC_SAMPLE_PERIOD_S			(uint32_t)(((SPEC_ATIME+1)*(SPEC_ASTEP+1)*2.78)/1000000)
#define SPEC_SAMPLE_SYS_PERIOD_MS		5000 //how often do we want the system to sample
#define SEND_SPEC_EVERY_X_S				30
#define MAX_SPEC_SAMPLES_PACKET	(SEND_SPEC_EVERY_X_S*1000)/SPEC_SAMPLE_SYS_PERIOD_MS
#define SPEC_FLICKER_DELAY				510

typedef struct specSamples {
	uint16_t _415;
	uint16_t _445;
	uint16_t _480;
	uint16_t _515;
	uint16_t _clear_1;
	uint16_t _nir_1;
	uint16_t _555;
	uint16_t _590;
	uint16_t _630;
	uint16_t _680;
	uint16_t _clear_2;
	uint16_t _nir_2;
	uint16_t flicker;
} specSample;

union SPEC_UNION {
	specSample s;
	uint16_t s_array[13];
};

static void triggerSpectrometerSample(void *argument);

static SPEC_UNION specData[MAX_SPEC_SAMPLES_PACKET];

static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicSpecTimer_id;

Adafruit_AS7341 specSensor;

void Spec_Task(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	if (!specSensor.begin(SPEC_ADDR, &hi2c1, 0)) {
		osDelay(100);
	}
	specSensor.setATIME(100);
	specSensor.setASTEP(999);
	specSensor.setGain(AS7341_GAIN_256X);

	header.payloadLength = MAX_SPEC_SAMPLES_PACKET * sizeof(SPEC_UNION);
	header.reserved[0] = SPEC_SAMPLE_SYS_PERIOD_MS;
	header.reserved[1] = SEND_SPEC_EVERY_X_S;

	uint16_t specIdx = 0;
	uint32_t specID = 0;

	specSensor.startReading();
	osSemaphoreRelease(messageI2C1_LockHandle);
	periodicSpecTimer_id = osTimerNew(triggerSpectrometerSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicSpecTimer_id, SPEC_SAMPLE_SYS_PERIOD_MS);

	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
			if(timeLeftForSample < SPEC_SAMPLE_PERIOD_MS){
				osDelay(timeLeftForSample);
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			while (!specSensor.checkReadingProgress()) {
				osSemaphoreRelease(messageI2C1_LockHandle);
				osDelay(10);
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			while (!specSensor.getAllChannels(specData[specIdx].s_array)) {
				osSemaphoreRelease(messageI2C1_LockHandle);
				osDelay(10);
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			specData[specIdx].s.flicker = specSensor.detectFlickerHz();
			osSemaphoreRelease(messageI2C1_LockHandle);

//			specIdx++;

//			if (specIdx >= MAX_SPEC_SAMPLES_PACKET) {
//				header.packetType = SPECTROMETER;
//				header.packetID = specID;
//				header.msFromStart = HAL_GetTick();
//				packet = grabPacket();
//				if (packet != NULL) {
//					memcpy(&(packet->header), &header, sizeof(PacketHeader));
//					memcpy(packet->payload, specData, header.payloadLength);
//					queueUpPacket(packet);
//				}
//				specID++;
//				specIdx = 0;
//			}
			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			specSensor.startReading();
			osSemaphoreRelease(messageI2C1_LockHandle);

			timeLeftForSample = HAL_GetTick();
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicSpecTimer_id);
			break;
		}
	}
}

static void triggerSpectrometerSample(void *argument) {
	osThreadFlagsSet(specTaskHandle, GRAB_SAMPLE_BIT);
}
