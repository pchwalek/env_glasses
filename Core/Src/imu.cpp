/*
 * imu.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "imu.h"
#include "Adafruit_ICM20948.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"

#define IMU_SAMPLE_PERIOD_MS		5
#define MAX_IMU_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(imu_sample)

static void triggerIMUSample(void *argument);

static imu_sample imuData[MAX_IMU_SAMPLES_PACKET];

static PacketHeader header;
osTimerId_t periodicIMUTimer_id;

Adafruit_ICM20948 imu;

void IMU_Task(void *argument){
	SensorPacket *packet = NULL;
	uint32_t flags = 0;

	osDelay(1000);

	while(!imu.begin_SPI(&hspi2,IMU_CS_GPIO_Port,IMU_CS_Pin)){
		osDelay(100);
	}

  header.payloadLength = MAX_IMU_SAMPLES_PACKET * sizeof(imu_sample);
  header.reserved[0] = IMU_SAMPLE_PERIOD_MS;

  uint16_t imuIdx = 0;
  uint32_t imuID = 0;

  periodicIMUTimer_id = osTimerNew(triggerIMUSample,
			osTimerPeriodic, NULL, NULL);
	osTimerStart(periodicIMUTimer_id, IMU_SAMPLE_PERIOD_MS);

  while(1){
  	flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
  					osFlagsWaitAny, osWaitForever);

  	if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
			imu.getSample(&imuData[imuIdx]);
			imuIdx++;

			if(imuIdx >= MAX_IMU_SAMPLES_PACKET){
				header.packetType = IMU;
				header.packetID = imuID;
				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if(packet != NULL){
					memcpy(&(packet->header), &header, sizeof(PacketHeader));
					memcpy(packet->payload, imuData, header.payloadLength);
					queueUpPacket(packet);
				}
				imuID++;
				imuIdx = 0;
			}

  	}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicIMUTimer_id);
		}
	}
}

static void triggerIMUSample(void *argument) {
	osThreadFlagsSet(imuTaskHandle, GRAB_SAMPLE_BIT);
}
