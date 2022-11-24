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

//#define IMU_SAMPLE_PERIOD_MS		5

#define IMU_SAMPLE_PERIOD_MS		50

//#define MAX_IMU_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(imu_sample)
#define MAX_IMU_SAMPLES_PACKET	1
#define MAX_IMU_PKT_SIZE	408

static void triggerIMUSample(void *argument);

static imu_sample imuData[MAX_IMU_SAMPLES_PACKET];

static PacketHeader header;
osTimerId_t periodicIMUTimer_id;

Adafruit_ICM20948 imu;
#define MAX_FIFO_CNT 4096
#define IMU_PKT_SIZE 12
static uint8_t data[MAX_FIFO_CNT];


void IMU_Task(void *argument){
	SensorPacket *packet = NULL;
	uint32_t flags = 0;

	osDelay(1000);

	while(!imu.begin_SPI(&hspi2,IMU_CS_GPIO_Port,IMU_CS_Pin)){
		osDelay(100);
	}

//  header.payloadLength = MAX_IMU_SAMPLES_PACKET * sizeof(imu_sample);
//  header.payloadLength = 408;
  header.packetType = IMU;

  header.reserved[0] = IMU_SAMPLE_PERIOD_MS;

  uint16_t imuIdx = 0;
  uint32_t imuID = 0;

  uint8_t intStatus[4] = {0};

  uint16_t fifo_cnt, fifo_cnt_2;

  uint16_t sampleTracker = 0;
  uint16_t start_idx = 0;
  uint16_t end_idx = 0;

  uint32_t lastTick = HAL_GetTick();
  float sampleRate = 0;

  periodicIMUTimer_id = osTimerNew(triggerIMUSample,
			osTimerPeriodic, NULL, NULL);
	osTimerStart(periodicIMUTimer_id, IMU_SAMPLE_PERIOD_MS);

  while(1){
  	flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
  					osFlagsWaitAny, osWaitForever);

  	if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
  		imu.getFIFOcnt(&fifo_cnt);

  		if(fifo_cnt != 0){
			fifo_cnt = fifo_cnt - (fifo_cnt % IMU_PKT_SIZE); // ensure reading only complete packets
	//
	//  		/* circular buffer logic */
	//  		if(end_idx == end_idx){
	//  			if( (end_idx + fifo_cnt) <= MAX_FIFO_CNT){
	//  		  		imu.readFIFO(&data[end_idx], fifo_cnt );
	//  		  		end_idx += fifo_cnt;
	//  			}
	//  			else{
	//  		  		imu.readFIFO(&data[end_idx], MAX_FIFO_CNT - end_idx);
	//
	//  			}
	//  		}

			imu.readFIFO(data, fifo_cnt );
			sampleTracker += fifo_cnt;
	//  		imu.getFIFOcnt(&fifo_cnt_2);
	//  		sampleRate = (fifo_cnt/6.0) / (( HAL_GetTick() - lastTick) / 1000.0);
			lastTick = HAL_GetTick();
			start_idx = 0;

			while(sampleTracker != 0){
				header.packetID = imuID;
				header.msFromStart = HAL_GetTick();
				if(sampleTracker >= MAX_IMU_PKT_SIZE){
					header.payloadLength = fifo_cnt;
				}else{
					header.payloadLength = sampleTracker;
				}
				packet = grabPacket();
				if(packet != NULL){
					memcpy(&(packet->header), &header, sizeof(PacketHeader));
					memcpy(packet->payload, &data[start_idx], header.payloadLength);
					queueUpPacket(packet);
				}
				sampleTracker -= header.payloadLength;
				start_idx += header.payloadLength;
				imuID++;
			}
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == IMU_INT_Pin) triggerIMUSample(NULL);
}

