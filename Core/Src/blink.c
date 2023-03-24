/**
 ******************************************************************************
 * File Name           : blink.c
 * Description        : C file for blink sensing.
 ******************************************************************************

 *
 ******************************************************************************
 */

/* includes -----------------------------------------------------------*/
#include "blink.h"
#include "gpio.h"
#include "stm32wbxx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "tim.h"
#include "adc.h"
#include "string.h"
//#include "master_thread.h"
#include "captivate_config.h"
#include "arm_math.h"
#include "math.h"
#include "packet.h"
/* typedef -----------------------------------------------------------*/

/* defines -----------------------------------------------------------*/

/* macros ------------------------------------------------------------*/

/* function prototypes -----------------------------------------------*/

/* variables -----------------------------------------------*/

/* Functions Definition ------------------------------------------------------*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

void initBlink();
void deinitBlink();

/* GLOBAL DEFINES */
//struct blinkData blinkMsgBuffer_1 = { { 0 }, 0, 0 };
uint8_t blink_buffer[1000] = { 0 };

//uint32_t blink_float_buffer[2000] = { 0 };
//uint8_t blink_buffer_neg[2000] = { 0 };

volatile uint8_t *blink_ptr;
volatile uint8_t *blink_ptr_copy;

//volatile uint8_t blink_copy[1000];

//uint32_t payload_ID = 0;
uint32_t iterator = 0;
float previousTick_ms = 0;
float tick_ms_diff = 0;

//struct LogMessage statusMessage;
uint8_t diodeState = 0;
uint8_t diodeSaturatedFlag = 0;

volatile uint32_t blinkTimestampMs = 0;
volatile uint64_t blinkTimestampUnix = 0;
volatile uint8_t firstInit = 0;


osTimerId_t blinkSingleShotTimer_id;
static uint32_t startTime;
/**
 * @brief Thread initialization.
 * @param  None
 * @retval None
 */
void BlinkTask(void *argument) {

	uint32_t evt;
	float rolling_avg = 255;
	uint16_t packetsPerHalfBuffer =  ceil( ( (float) BLINK_HALF_BUFFER_SIZE)/(BLINK_PKT_PAYLOAD_SIZE) );
	uint16_t payloadLength = 0;
	uint16_t blinkDataTracker = 0;
	uint32_t payload_ID = 0;
//	uint32_t tickCnt;
//	uint32_t blinkSampleHalfBuffer_ms = BLINK_HALF_BUFFER_SIZE * (1.0/BLINK_SAMPLE_RATE) * 1000.0;
//	uint32_t packetRemainder = BLINK_SAMPLE_RATE % BLINK_PKT_PAYLOAD_SIZE;

	firstInit = 1;

	sensor_packet_t *packet = NULL;

	uint32_t threadBlinkTimestampMs = 0;
	uint64_t threadBlinkTimestampUnix = 0;

	osDelay(500);

	blink_sensor_config_t sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(blink_sensor_config_t));
	}else{
		sensorSettings.enable_daylight_compensation = 1;
		sensorSettings.daylight_compensation_upper_thresh = INFRARED_DETECT_UPPER_THRESH;
		sensorSettings.daylight_compensation_lower_thresh = INFRARED_DETECT_LOWER_THRESH;
		sensorSettings.sample_frequency = BLINK_SAMPLE_RATE;

		sensorSettings.enable_windowing = false;
		sensorSettings.enable_windowing_sync = false;
//		sensorSettings.window_size_ms = BLINK_SAMPLE_RATE;
//		sensorSettings.window_period_ms = BLINK_SAMPLE_RATE;
	}

	float sensorSamplePeriod_ms = 1000.0/sensorSettings.sample_frequency;

	while (1) {
//		evt = osThreadFlagsWait(0x00000001 | TERMINATE_THREAD_BIT, osFlagsWaitAny, osWaitForever);
		evt = 0x00000001U;
		// if signal was received successfully, start blink task
		if ((evt & 0x00000001U) == 0x00000001U) {

			initBlink();
			startTime = HAL_GetTick();
			// message passing until told to stop
			//      note: DMA triggers callback where buffers are switched and the full one
			//      is passed by reference via queue to masterThread for packetization

			while (1) {

				// wait for data ready flag and/or stop task flags
				evt = osThreadFlagsWait(0x00000006U | TERMINATE_THREAD_BIT, osFlagsWaitAny,
						osWaitForever);
				blink_ptr_copy = blink_ptr;

				// if timer triggered, wait for event to be triggered to restart blink sampling
//				if ((evt & WINDOW_RDY_BIT) == WINDOW_RDY_BIT) {
//					initBlink();
//					startTime = HAL_GetTick();
//				}

				if ((evt & 0x00000004U) == 0x00000004U) {
//
//					if(firstInit = 1){
//						blinkTimestampMs = HAL_GetTick();
//						blinkTimestampUnix = getEpoch();
//						firstInit = 0;
//					}

					if(firstInit == 1){
						threadBlinkTimestampMs = HAL_GetTick();
						threadBlinkTimestampUnix = getEpoch();

						firstInit = 0;
					}

					blinkDataTracker = BLINK_HALF_BUFFER_SIZE;

					// because of COAP packet size restrictions, separate blink packet into chunks of size BLINK_PACKET_SIZE
					for (iterator = 0; iterator < packetsPerHalfBuffer;
							iterator++) {

						// add to packet maximum sized payload
						if(blinkDataTracker > BLINK_PKT_PAYLOAD_SIZE){
						    payloadLength = BLINK_PKT_PAYLOAD_SIZE;
						    blinkDataTracker -= BLINK_PKT_PAYLOAD_SIZE;
						}else if(blinkDataTracker != 0){
						    payloadLength = blinkDataTracker;
						    blinkDataTracker = 0;
						}else{
						    break; //should never happen
						}

						// grab available memory for packet creation
						if(osOK != osMessageQueueGet(packetAvail_QueueHandle, &packet, 0U,
							    0)){
						    //no memory available so increment payload ID and drop packet
						    payload_ID++;
						    continue;
						}


						setPacketType(packet, SENSOR_PACKET_TYPES_BLINK);

						packet->payload.blink_packet.packet_index = payload_ID;

						packet->payload.blink_packet.has_saturation_settings = sensorSettings.enable_daylight_compensation;
						packet->payload.blink_packet.saturation_settings.diode_turned_off = diodeSaturatedFlag;
						packet->payload.blink_packet.saturation_settings.diode_saturation_lower_thresh = sensorSettings.daylight_compensation_lower_thresh;
						packet->payload.blink_packet.saturation_settings.diode_saturation_upper_thresh = sensorSettings.daylight_compensation_upper_thresh;
						packet->payload.blink_packet.sample_rate = sensorSettings.sample_frequency;
						packet->payload.blink_packet.which_payload = BLINK_PACKET_PAYLOAD_BYTE_TAG;

						packet->payload.blink_packet.timestamp_unix = threadBlinkTimestampUnix;
						packet->payload.blink_packet.timestamp_ms_from_start = threadBlinkTimestampMs;

						threadBlinkTimestampMs += payloadLength * (uint64_t)sensorSamplePeriod_ms;
						threadBlinkTimestampUnix += payloadLength * (uint64_t)sensorSamplePeriod_ms;

						// write blink data
						memcpy(packet->payload.blink_packet.payload.payload_byte.sample.bytes, (uint8_t *) &(blink_ptr_copy[iterator * BLINK_PKT_PAYLOAD_SIZE]), payloadLength);

						packet->payload.blink_packet.payload.payload_byte.sample.size = payloadLength;

					    // send to BT packetizer
						queueUpPacket(packet,20);

//						// add tick cnt
//						previousTick_ms = blinkMsgBuffer_1.tick_ms;

						payload_ID++;

					}

					/* check to see if external infrared is saturating
					 *   if so, disable active diode
					 *   otherwise, leave enabled */

					// BLINK_SAMPLE_RATE == size of blink_ptr array
					if(sensorSettings.enable_daylight_compensation){
						diodeSaturatedFlag = externalInfraredDetect((uint8_t *) blink_ptr_copy, BLINK_SAMPLE_RATE, &rolling_avg,
								sensorSettings.daylight_compensation_upper_thresh,
								sensorSettings.daylight_compensation_lower_thresh );

						/* not using PWM */
						if(diodeSaturatedFlag){
							if(diodeState) turnOffDiode();
						}
						else{
							if(!diodeState) turnOnDiode();
						}
					}


					if(sensorSettings.enable_windowing){

						if(sensorSettings.window_size_ms < (HAL_GetTick() - startTime)){

							deinitBlink();

							if(sensorSettings.enable_windowing_sync){
								if(!((evt & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT)){
									evt = osThreadFlagsWait(TERMINATE_THREAD_BIT | WINDOW_SYNC_RDY_BIT,
																			osFlagsWaitAny, osWaitForever);
									if((evt & WINDOW_SYNC_RDY_BIT) == WINDOW_SYNC_RDY_BIT){
										initBlink();
									}
								}

							}else{
								int32_t waitTime = sensorSettings.window_period_ms - sensorSettings.window_size_ms;
								if(waitTime < 0) waitTime = 0;

								blinkSingleShotTimer_id = osTimerNew(initBlink, osTimerOnce, NULL, NULL);
								osTimerStart(blinkSingleShotTimer_id, waitTime);
							}

							firstInit = 1;

						}
					}
				}

				// stop timer and put thread in idle if signal was reset
				if ((evt & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {

					osTimerDelete(blinkSingleShotTimer_id);
					deinitBlink();

					// clear any flags
					osThreadFlagsClear(0x00000006U);
					vTaskDelete( NULL );

					break;
				}
			}
		}
	}
}

void initBlink(){
	startTime = HAL_GetTick();

	// start timer for ADC to sample at 1kHz
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) blink_buffer,
			sizeof(blink_buffer));

//			 start timer
	HAL_TIM_Base_Start(&htim2);

	/* using PWM */
//	HAL_TIM_Base_Start(&htim16); // modulation frequency is at 1kHz
//	if(HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) == HAL_OK){
//		diodeState = 1;
//	}
	turnOnDiode();

	// reset external infrared detection flag
	diodeSaturatedFlag = 0;

}

void deinitBlink(){
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim2);

	turnOffDiode();
}

void turnOffDiode(){


	HAL_GPIO_WritePin(BLINK_PWM_GPIO_Port, BLINK_PWM_Pin,
			GPIO_PIN_RESET);
	diodeState = 0;

//	if(HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1) == HAL_OK){
//		diodeState = 0;
//	}

//	HAL_TIM_Base_Stop(&htim16); // modulation frequency is at 1kHz


}

void turnOnDiode(){
//	HAL_GPIO_WritePin(BLINK_PWM_GPIO_Port, BLINK_PWM_Pin,
//			GPIO_PIN_SET);
//	diodeState = 1;

	HAL_GPIO_WritePin(BLINK_PWM_GPIO_Port, BLINK_PWM_Pin,
			GPIO_PIN_SET);
	diodeState = 1;

//	HAL_TIM_Base_Start(&htim16); // modulation frequency is at 1kHz
//	if(HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) == HAL_OK){
//		diodeState = 1;
//	}
}

#define INFRARED_DETECT 	1
#define NO_INFRARED_DETECT	0
float32_t sample_avg;
uint8_t detect_active = 0;
uint8_t externalInfraredDetect(uint8_t* blink_sample, uint32_t size_of_blink_ptr, float* rolling_avg,
		uint8_t upperThresh, uint8_t lowerThresh){

	//todo: convert to q format or float32 and use arm library. uint8_t can't be typecasted to 32-bit float
//	arm_mean_f32((float *) random_array, 2, &sample_avg);

	// temporary brute force average
	sample_avg = blink_sample[0] + blink_sample[100] + blink_sample[200] + blink_sample[300] + blink_sample[400]
               + blink_sample[500] + blink_sample[600] + blink_sample[700] + blink_sample[800] + blink_sample[900];
	sample_avg /= 10;

	*rolling_avg = INFRARED_DETECT_ALPHA * sample_avg + (1.0-INFRARED_DETECT_ALPHA) * (*rolling_avg);

	/* SCHMITT TRIGGER */
	if(detect_active){
		if( (*rolling_avg) > upperThresh ){
			detect_active = NO_INFRARED_DETECT;
		}
	}else{
		if( (*rolling_avg) < lowerThresh ){
			detect_active = INFRARED_DETECT;
		}
	}

	return detect_active;
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//}

//volatile uint8_t i = 0;
//void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
//	i++;
//}

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
//
//volatile uint32_t pwm_tracker_s = 0;
//volatile uint32_t pwm_tracker_diff = 0;
//
//volatile uint32_t pwm_tracker = 0;
//
//volatile uint8_t low_adc_sample = 0;
//volatile uint8_t random_sample = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	end_time = HAL_GetTick() - start_time;
////	pwm_tracker_diff = pwm_tracker - pwm_tracker_s;
//	start_time = HAL_GetTick();
//	pwm_tracker_s = pwm_tracker;
//	if(low_adc_sample){
//		HAL_ADC_Stop(&hadc1);
//		low_adc_sample = 0;
//		return;
//	}



	blink_ptr = &blink_buffer[BLINK_HALF_BUFFER_SIZE];
	osThreadFlagsSet(blinkTaskHandle, 0x00000004U);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {


	blink_ptr = blink_buffer;
	osThreadFlagsSet(blinkTaskHandle, 0x00000004U);
}

void BlinkSyncTrigger(void) {
	osThreadFlagsSet(blinkTaskHandle, WINDOW_SYNC_RDY_BIT);

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//	if(htim->Instance == TIM2){
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		HAL_ADC_Start(&hadc1);
//	}
//	pwm_tracker++;

//	low_adc_sample = 1;
//	HAL_ADC_Start(&hadc1);
}

/**
 * @brief Setting up blink sensing
 * @param  None
 * @retval None
 */

/*************************************************************
 *
 * FREERTOS WRAPPER FUNCTIONS
 *
 *************************************************************/
