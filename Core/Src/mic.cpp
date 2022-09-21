/*
 * mic.cpp
 *
 *  Created on: Jan 4, 2022
 *      Author: patri
 */

#include "mic.h"

#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"


//#define MAX_LIDAR_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(lidar_sample)

#define MAX_MIC_SAMPLES_PACKET  10

static PacketHeader header;

void Mic_Task(void *argument){
	SensorPacket *packet = NULL;
	uint32_t flags = 0;

//  header.payloadLength = MAX_LIDAR_SAMPLES_PACKET * sizeof(lidar_sample);

  uint16_t micIdx = 0;
  uint32_t micID = 0;


  while(1){
  	flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
  					osFlagsWaitAny, osWaitForever);

  	if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
		  // Use polling function to know when a new measurement is ready.



  		micIdx++;

			if(micIdx >= MAX_MIC_SAMPLES_PACKET){
				header.packetType = MIC;
				header.packetID = micID;
				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if(packet != NULL){
//					memcpy(&(packet->header), &header, sizeof(PacketHeader));
//					memcpy(packet->payload, lidarData, header.payloadLength);
					queueUpPacket(packet);
				}
				micID++;
				micIdx = 0;
			}

  	}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
//			osTimerDelete(periodicLidarTimer_id);
		}
	}
}

static void triggerMicSample(void *argument) {
	osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
}
