/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "sht.h"
#include "Adafruit_SHT4x.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"


#define SHT_SAMPLE_SYS_PERIOD_MS		5000 //how often do we want the system to sample
#define SEND_SHT_EVERY_X_S				30
#define MAX_SHT_SAMPLES_PACKET	(SEND_SHT_EVERY_X_S*1000)/SHT_SAMPLE_SYS_PERIOD_MS

typedef struct shtSamples {
	float temp;
	float hum;
	uint32_t timestamp;
} shtSample;


static void triggerShtSample(void *argument);
static shtSample shtData[MAX_SHT_SAMPLES_PACKET];


static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicShtTimer_id;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

float shtTemp, shtHum;

void ShtTask(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;

	shtTemp = -1;
	shtHum = -1;

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	if (!sht4.begin(&hi2c1)) {
		osDelay(100);
	}

	sht4.setPrecision(SHT4X_HIGH_PRECISION);
	sht4.setHeater(SHT4X_NO_HEATER);

	header.payloadLength = MAX_SHT_SAMPLES_PACKET * sizeof(shtSample);
	header.reserved[0] = (uint8_t) SHT4X_HIGH_PRECISION;
	header.reserved[1] = (uint8_t) SHT4X_NO_HEATER;

	uint16_t shtIdx = 0;
	uint32_t shtID = 0;


	uint32_t shtSample;

	osSemaphoreRelease(messageI2C1_LockHandle);
	periodicShtTimer_id = osTimerNew(triggerShtSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicShtTimer_id, SHT_SAMPLE_SYS_PERIOD_MS);


	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
			if(timeLeftForSample < SHT_SAMPLE_SYS_PERIOD_MS){
				osDelay(timeLeftForSample);
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			if(sht4.getEvent()){
				shtData[shtIdx].temp = sht4._temperature;
				shtData[shtIdx].hum = sht4._humidity;
				shtData[shtIdx].timestamp = HAL_GetTick();

				shtTemp = shtData[shtIdx].temp;
				shtHum = shtData[shtIdx].hum;
			}else{
				continue;
			}

			osSemaphoreRelease(messageI2C1_LockHandle);

			shtIdx++;

			if (shtIdx >= MAX_SHT_SAMPLES_PACKET) {
				header.packetType = SHT;
				header.packetID = shtID;
				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {
					memcpy(&(packet->header), &header, sizeof(PacketHeader));
					memcpy(packet->payload, shtData, header.payloadLength);
					queueUpPacket(packet);
				}
				shtID++;
				shtIdx = 0;
			}

			timeLeftForSample = HAL_GetTick();
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicShtTimer_id);
			break;
		}
	}
}

static void triggerShtSample(void *argument) {
	osThreadFlagsSet(shtTaskHandle, GRAB_SAMPLE_BIT);
}
