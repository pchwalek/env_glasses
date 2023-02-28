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

#include "dt_server_app.h"
#include "uuid.h"

// pretty OK tutorial: https://stm32f4-discovery.net/2014/10/stm32f4-fft-example/

//#define MIC_DATA_SIZE		4096 // make multiple of 2 for simplicity
#define MIC_DATA_SIZE 10000
#define MIC_HALF_DATA_SIZE 5000
#define MIC_PACKET_SAMPLE_SIZE 110

//#define MIC_HALF_DATA_SIZE	MIC_DATA_SIZE >> 1 // make multiple of 2 for simplicity

/* if the microphone sample period is equal or greater than MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF,
 * the microphone will be turned off in between samples
 */
#define MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF	5000
uint32_t micData[MIC_DATA_SIZE];
float micDataFloat[4096];

static void triggerMicSample(void *argument);

//#define MAX_LIDAR_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(lidar_sample)

#define MAX_MIC_SAMPLES_PACKET  10


struct MicCal{
	uint8_t index;
	uint8_t length;
	uint32_t values[MIC_PACKET_SAMPLE_SIZE];
};

//static PacketHeader header;
//arm_rfft_instance_q31 fft_instance;
arm_rfft_fast_instance_f32 fft_instance;
volatile uint8_t buffer_tracker;

osTimerId_t periodicMicTimer_id;

uint32_t* micDataPointer;

volatile uint8_t secondMicSample = 0;
volatile uint8_t micLowPowerMode = 0;

static DTS_App_Context_t DataTransferServerContext;

volatile tBleStatus ble_status = BLE_STATUS_INVALID_PARAMS;

#define SIZE_OF_MIC_CAL_DATA   10
MicCal micCalData[SIZE_OF_MIC_CAL_DATA];
MicCal *ptrMicCalData;
void Mic_Task(void *argument){
	sensor_packet_t *packet = NULL;

	uint32_t micID = 0;
	uint32_t flags = 0;
	float32_t maxvalue;
	uint32_t maxindex;
	float32_t dominantFrequency;
	uint16_t startIdx;

	bool status;


	float startFreq;

	mic_sensor_config_t sensorSettings;

//	arm_rfft_init_q31(&fft_instance, MIC_HALF_DATA_SIZE, 0, 0);
	arm_rfft_fast_init_f32(&fft_instance, MIC_DATA_SIZE);

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct MicSensor));
	}else{
		sensorSettings.mic_sample_freq = SAI_AUDIO_FREQUENCY_48K;
		sensorSettings.sample_period_ms = 30000; // every 30 seconds
	}

	//todo: temporary code which should be removed once website is updated
//	sensorSettings.mic_sample_freq = SAI_AUDIO_FREQUENCY_48K;
//	sensorSettings.sample_period_ms = 30000; // every 30 seconds


	float fft_spacing = 48000 / 4096.0;


//	memcpy(&header.reserved[4], (uint32_t *) &fft_spacing, sizeof(fft_spacing));

	uint16_t maxMicPayloadSize = floor( (MAX_PAYLOAD_SIZE) / 4); // number of 4 byte floats
	uint32_t totalMicPayloadSize = (MIC_DATA_SIZE >> 1) - 1; // number of 4 byte floats
	uint16_t packetsPerMicSample = ceil( ((float) totalMicPayloadSize) / (maxMicPayloadSize) );

	uint32_t sample_count;

	uint32_t num_of_samples_per_cycle = ceil(((float) MIC_HALF_DATA_SIZE) / MIC_PACKET_SAMPLE_SIZE);

	hsai_BlockA1.Init.AudioFrequency = sensorSettings.mic_sample_freq;

	HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);

	/* prime the SAI channel
	 * (needs to be started to ensure first sample does not have
	 * null bytes since I2S takes a few cycles to turn on)
	 */
	HAL_SAI_Receive_DMA(&hsai_BlockA1,(uint8_t *) micData, MIC_DATA_SIZE);



	uint32_t micPktLen = 0;
	uint32_t micPktTracker = 0;

	uint32_t micCalDataTracker = 0;

	uint32_t* micDataThreadPointer;

//	HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *) micData, 256, 1);  //purposeful short timeout
//
//	if(sensorSettings.sample_period_ms > MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF){
//		micLowPowerMode = 1;
//	}else{
//		micLowPowerMode = 0;
//	}
//
//	periodicMicTimer_id = osTimerNew(triggerMicSample, osTimerPeriodic,
//				NULL, NULL);
//	osTimerStart(periodicMicTimer_id, sensorSettings.sample_period_ms);

	while(1){



		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);


		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

		  micDataThreadPointer = micDataPointer;

//		  for(int i = 0; i<MIC_DATA_SIZE; i++){
//			  micDataThreadPointer[i] = micDataThreadPointer[i] << 8;
//		  }

		  ptrMicCalData = &micCalData[micCalDataTracker % SIZE_OF_MIC_CAL_DATA];
	      micCalDataTracker += 1;

			for(int i = 0; i < MIC_HALF_DATA_SIZE; i+=MIC_PACKET_SAMPLE_SIZE){
//				while(packet != NULL){
//					packet = grabPacket();
//					if(packet == NULL){
//						osDelay(1);
//					}
//				}
//				if(packet != NULL){

					if( (i + MIC_PACKET_SAMPLE_SIZE) > MIC_HALF_DATA_SIZE){
						ptrMicCalData->length = MIC_PACKET_SAMPLE_SIZE - i;
					}else{
						ptrMicCalData->length = MIC_PACKET_SAMPLE_SIZE;
					}

					ptrMicCalData->index = micID;
					memcpy(&ptrMicCalData->values[0], &micDataThreadPointer[i], ptrMicCalData->length);
					micID++;


					DataTransferServerContext.TxData.pPayload = (uint8_t*) ptrMicCalData;
					DataTransferServerContext.TxData.Length = 4 + ptrMicCalData->length * 4; //Att_Mtu_Exchanged-10;

					ble_status = DTS_STM_UpdateChar(DATA_TRANSFER_TX_CHAR_UUID,
												(uint8_t*) &DataTransferServerContext.TxData);
					while(ble_status != BLE_STATUS_SUCCESS){
//						taskENTER_CRITICAL();
						osDelay(1);
					ble_status = DTS_STM_UpdateChar(DATA_TRANSFER_TX_CHAR_UUID,
							(uint8_t*) &DataTransferServerContext.TxData);
//					taskEXIT_CRITICAL();

					}
//					osDelay(1);

//				}

			}



		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
//			osTimerDelete(periodicMicTimer_id);
			HAL_SAI_MspDeInit(&hsai_BlockA1);
			osThreadExit();
			break;
		}
	}

	osThreadExit();
}

//static void triggerMicSample(void *argument){
//	HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);
//}
//
//void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
//	/* found that when the mic clock gets started via the interrupt call,
//	 * the first several bytes (up to several 100) can be null so a second go
//	 * is warranted.
//	 *
//	 * this is because mic takes 20ms to wakeup as per the datasheet
//	 */
//	if( (secondMicSample == 0) && (micLowPowerMode) ){
//		secondMicSample = 1;
//		HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData, MIC_DATA_SIZE);
//	}
//	else{
//		secondMicSample = 0;
//		osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
//	}
//}

/**
  * @brief Rx Transfer completed callback.
  * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
//  UNUSED(hsai);
//  audio_data_buffer_index = 1;
//  audio_data_available = 1;
//  HAL_GPIO_TogglePin(AUDIO_READY_GPIO_Port, AUDIO_READY_Pin);
	micDataPointer = &micData[MIC_HALF_DATA_SIZE];
  osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
}

/**
  * @brief Rx Transfer half completed callback.
  * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
//  UNUSED(hsai);
//  audio_data_buffer_index = 0;
//  audio_data_available = 1;
//  HAL_GPIO_TogglePin(AUDIO_READY_GPIO_Port, AUDIO_READY_Pin);
	micDataPointer = &micData[0];
  osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
}

//void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
//
//}
