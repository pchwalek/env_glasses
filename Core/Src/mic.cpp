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

// pretty OK tutorial: https://stm32f4-discovery.net/2014/10/stm32f4-fft-example/

#define MIC_DATA_SIZE		4096 // make multiple of 2 for simplicity
#define MIC_HALF_DATA_SIZE	MIC_DATA_SIZE >> 1 // make multiple of 2 for simplicity

/* if the microphone sample period is equal or greater than MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF,
 * the microphone will be turned off in between samples
 */
#define MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF	30000
uint32_t micData[MIC_DATA_SIZE];
float micDataFloat[4096];

static void triggerMicSample(void *argument);

//#define MAX_LIDAR_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(lidar_sample)

#define MAX_MIC_SAMPLES_PACKET  10

static PacketHeader header;
//arm_rfft_instance_q31 fft_instance;
arm_rfft_fast_instance_f32 fft_instance;
volatile uint8_t buffer_tracker;

osTimerId_t periodicMicTimer_id;

volatile uint8_t secondMicSample = 0;
volatile uint8_t micLowPowerMode = 0;


void Mic_Task(void *argument){

	uint32_t micID = 0;
	uint32_t flags = 0;
	float32_t maxvalue;
	uint32_t maxindex;
	float32_t dominantFrequency;

	struct MicSensor sensorSettings;

//	arm_rfft_init_q31(&fft_instance, MIC_HALF_DATA_SIZE, 0, 0);
	arm_rfft_fast_init_f32(&fft_instance, MIC_DATA_SIZE);

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct MicSensor));
	}else{
		sensorSettings.mic_sample_frequency = SAI_AUDIO_FREQUENCY_48K;
		sensorSettings.sys_sample_period_ms = 30000; // every 30 seconds
	}

	//todo: temporary code which should be removed once website is updated
	sensorSettings.mic_sample_frequency = SAI_AUDIO_FREQUENCY_48K;
	sensorSettings.sys_sample_period_ms = 30000; // every 30 seconds

	hsai_BlockA1.Init.AudioFrequency = sensorSettings.mic_sample_frequency;

	HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);

	/* prime the SAI channel
	 * (needs to be started to ensure first sample does not have
	 * null bytes since I2S takes a few cycles to turn on)
	 */
	HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *) micData, 256, 1);  //purposeful short timeout

	if(sensorSettings.sys_sample_period_ms >= MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF){
		micLowPowerMode = 1;
	}else{
		micLowPowerMode = 0;
	}

	periodicMicTimer_id = osTimerNew(triggerMicSample, osTimerPeriodic,
				NULL, NULL);
	osTimerStart(periodicMicTimer_id, sensorSettings.sys_sample_period_ms);

	while(1){
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);


		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
			if(micLowPowerMode){
				HAL_SAI_DeInit(&hsai_BlockA1);
				HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);
			}

		/* shifting by 8 so that the 24-bit (2's complement) number
		   * is in the 24 MSBs. When using ARM functions, we will treat
		   * this uint32_t array as a Q31 format where it'll be treated
		   * as if the original values were divided by (2^24)
		   */

		  for(int i = 0; i<MIC_DATA_SIZE; i++){
			  micData[i] = micData[i] << 8;
		  }

		  /* convert to float and scale by 2^24 so that we get the exact values
		   * as intended from the microphone. However, because scale doesn't matter
		   * for FFT, we can theoretically skip the below two steps and do a Q31
		   * FFT to save <10ms in computation
		   */
		  arm_q31_to_float ((q31_t *) &micData[0],
						&micDataFloat[0], MIC_DATA_SIZE); //~4ms

		  arm_scale_f32 (&micDataFloat[0], 16777216, &micDataFloat[0], MIC_DATA_SIZE); //~4ms

		  /* WARNING: this function modifies dataMicF */
		  arm_rfft_fast_f32(&fft_instance, micDataFloat, micDataFloat, 0); //~22ms

		  arm_cmplx_mag_f32(micDataFloat, micDataFloat, MIC_DATA_SIZE >> 1); //~6ms

		  arm_max_f32(&micDataFloat[1], (MIC_DATA_SIZE >> 1) - 1, &maxvalue, &maxindex); //~2ms

		  dominantFrequency = maxindex * 48000 / 4096.0;

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
			osTimerDelete(periodicMicTimer_id);
			HAL_SAI_MspDeInit(&hsai_BlockA1);
			osThreadExit();
			break;
		}
	}

	osThreadExit();
}

static void triggerMicSample(void *argument){
	HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	/* found that when the mic clock gets started via the interrupt call,
	 * the first several bytes (up to several 100) can be null so a second go
	 * is warranted.
	 */
	if( (secondMicSample == 0) && (micLowPowerMode) ){
		secondMicSample = 1;
		HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);
	}
	else{
		secondMicSample = 0;
		osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
	}
}

//void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
//
//}
