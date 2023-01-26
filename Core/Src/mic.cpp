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

#include "stm32wbxx_hal_sai.h"
#include "sai.h"

#include "arm_math.h"
#include "math.h"

#define MIC_DATA_SIZE		4096 // make multiple of 2 for simplicity
#define MIC_HALF_DATA_SIZE	MIC_DATA_SIZE >> 1 // make multiple of 2 for simplicity
uint32_t micData[MIC_DATA_SIZE];
float micDataFloat[4096];

//#define MAX_LIDAR_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(lidar_sample)

#define MAX_MIC_SAMPLES_PACKET  10

static PacketHeader header;
arm_rfft_instance_q31 fft_instance;
volatile uint8_t buffer_tracker;

void Mic_Task(void *argument){

	uint32_t micID = 0;
	uint32_t flags = 0;

	struct MicSensor sensorSettings;

	arm_rfft_init_q31(&fft_instance, MIC_HALF_DATA_SIZE, 0, 0);

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct InertialSensor));
	}else{
		sensorSettings.mic_sample_frequency = SAI_AUDIO_FREQUENCY_22K;
		sensorSettings.sys_sample_period_ms = 30000; // every 30 seconds
	}

	hsai_BlockA1.Init.AudioFrequency = sensorSettings.mic_sample_frequency;

	HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);

//	HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);

	HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);

	while(1){
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);



		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			/* apply FFT */
			// (1) convert to correct format
			arm_shift_q31((q31_t *) &micData[buffer_tracker*MIC_HALF_DATA_SIZE], 8,
					(q31_t *) &micData[buffer_tracker*MIC_HALF_DATA_SIZE], MIC_HALF_DATA_SIZE );
			arm_shift_q31((q31_t *) &micData[buffer_tracker*MIC_HALF_DATA_SIZE], -8,
					(q31_t *) &micData[buffer_tracker*MIC_HALF_DATA_SIZE], MIC_HALF_DATA_SIZE );

			// convert to float for ease of use
			// arm_q31_to_float ((q31_t *) &micData[buffer_tracker*MIC_HALF_DATA_SIZE],
//				&micDataFloat[buffer_tracker*MIC_HALF_DATA_SIZE], MIC_HALF_DATA_SIZE);

			// (2) apply FFT (output format 12:20; 12 exponent bits, 20 fractional bits)
//			arm_rfft_q31(&fft_instance, &micData[buffer_tracker*MIC_HALF_DATA_SIZE], &micData[buffer_tracker*MIC_HALF_DATA_SIZE]);
//
//
//			/* packetize data */
//			header.packetType = MIC;
//			header.packetID = micID;
//			header.msFromStart = HAL_GetTick();
//			packet = grabPacket();
//			if(packet != NULL){
////					memcpy(&(packet->header), &header, sizeof(PacketHeader));
////					memcpy(packet->payload, lidarData, header.payloadLength);
//				queueUpPacket(packet);
//			}
//			micID++;


		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			HAL_SAI_DMAStop(&hsai_BlockA1);
			osThreadExit();
			break;
		}
	}
//	SensorPacket *packet = NULL;
//	uint32_t flags = 0;
//
////  header.payloadLength = MAX_LIDAR_SAMPLES_PACKET * sizeof(lidar_sample);
//
//  uint16_t micIdx = 0;
//  uint32_t micID = 0;
//
//
//  while(1){
//  	flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
//  					osFlagsWaitAny, osWaitForever);
//
//  	if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
//		  // Use polling function to know when a new measurement is ready.
//
//
//
//  		micIdx++;
//
//			if(micIdx >= MAX_MIC_SAMPLES_PACKET){
//				header.packetType = MIC;
//				header.packetID = micID;
//				header.msFromStart = HAL_GetTick();
//				packet = grabPacket();
//				if(packet != NULL){
////					memcpy(&(packet->header), &header, sizeof(PacketHeader));
////					memcpy(packet->payload, lidarData, header.payloadLength);
//					queueUpPacket(packet);
//				}
//				micID++;
//				micIdx = 0;
//			}
//
//  	}
//
//		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
////			osTimerDelete(periodicLidarTimer_id);
//		}
//	}
	osThreadExit();
}

//void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
//	buffer_tracker = (!buffer_tracker) & 0x01;
//	osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
//}
