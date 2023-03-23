/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "bme.h"
#include "Adafruit_BME680.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"
//#include "bsec2.h"
//#include "../Middlewares/bsec_2_2_0_0/algo/normal_version/inc/bsec_datatypes.h"
#include "bsec_datatypes.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "fram.h"

//#include "config/FieldAir_HandSanitizer/FieldAir_HandSanitizer.h"
//#include "config/Default_H2S_NonH2S/Default_H2S_NonH2S.h"
#include "config/bsec_sel_iaq_33v_3s_28d/bsec_serialized_configurations_selectivity.h"


#define BME_SAMPLE_PERIOD_MS		3000
//#define MAX_BME_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(bsecData)
#define MAX_BME_SAMPLES_PACKET		20
#define BME_WAIT_TOL			10
#define BME_SAVE_STATE_PERIOD_MS	7200000 // every 2 hours

static bme_packet_payload_t bmeData[12];

osTimerId_t periodicBMETimer_id;

Adafruit_BME680 bme;

static uint8_t bmeConfig[BSEC_MAX_PROPERTY_BLOB_SIZE];
static uint8_t bmeState[BSEC_MAX_STATE_BLOB_SIZE];


void BME_Task(void *argument) {
	sensor_packet_t *packet = NULL;
	uint32_t flags = 0;

//	bool status;

	bme_sensor_config_t sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(bme_sensor_config_t));
	}else{
		sensorSettings.sample_period_ms = 0;
	}

	uint32_t timeSinceLastStateSave = 0;

	osDelay(500);

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	while (!bme.begin(BME68X_DEFAULT_ADDRESS, &hi2c1, false)) {
		osSemaphoreRelease(messageI2C1_LockHandle);
		osDelay(100);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	}


	recoverBME_StateConfig();

	osSemaphoreRelease(messageI2C1_LockHandle);

	bme.bsecSubscribe();

	volatile uint16_t bmeIdx = 0;
	uint32_t bmeID = 0;

	int64_t timeRemaining;

	while (1) {

//		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
		if(1){

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			while(!bme.bsecRun()){
				osSemaphoreRelease(messageI2C1_LockHandle);
				timeRemaining = floor((bme.bmeConf.next_call/1000000.0) - HAL_GetTick());
				if(timeRemaining > BME_WAIT_TOL){
					osDelay( (timeRemaining-BME_WAIT_TOL) );
				}else if(timeRemaining > 1){
					osDelay(1);
				}
				osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			}
			osSemaphoreRelease(messageI2C1_LockHandle);

			for(int i = 0; i<bme.outputs.nOutputs; i++){
//				memcpy(&bmeData[bmeIdx++], &bme.outputs.output[i], sizeof(bsecData));
				bmeData[bmeIdx].timestamp_sensor = bme.outputs.output[i].time_stamp;
				bmeData[bmeIdx].timestamp_unix = getEpoch();
				bmeData[bmeIdx].timestamp_ms_from_start = HAL_GetTick();
				bmeData[bmeIdx].signal = bme.outputs.output[i].signal;
				bmeData[bmeIdx].signal_dimensions = bme.outputs.output[i].signal_dimensions;
				bmeData[bmeIdx].sensor_id = static_cast<bme680_signal_id_t>(bme.outputs.output[i].sensor_id);
				bmeData[bmeIdx++].accuracy = static_cast<bme680_accuracy_t>(bme.outputs.output[i].accuracy);
			}

			if (bmeIdx > 0) {

				packet = grabPacket();
				if (packet != NULL) {

					setPacketType(packet, SENSOR_PACKET_TYPES_BME);

					packet->payload.bme_packet.packet_index = bmeID;

					packet->payload.bme_packet.sample_period = BME_SAMPLE_PERIOD_MS;
					packet->payload.bme_packet.sensor_id = 0;

					// write data   //sensorPacket.header.payload_length
					memcpy(packet->payload.bme_packet.payload, bmeData, bmeIdx * sizeof(bme_packet_payload_t));
					packet->payload.bme_packet.payload_count = bmeIdx;

					// send to BT packetizer
					queueUpPacket(packet);

				}
				bmeID++;
				bmeIdx = 0;

			}

			if( (HAL_GetTick() - timeSinceLastStateSave) >= BME_SAVE_STATE_PERIOD_MS){
				saveBME_StateConfig();
				timeSinceLastStateSave = HAL_GetTick();
			}

		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete (periodicBMETimer_id);
			saveBME_StateConfig();
			bme.soft_reset();

			vTaskDelete( NULL );
			break;
		}
	}
}

void saveBME_StateConfig(){
	uint32_t bsecReturnLen;

	bme.bsecGetConfig(bmeConfig, &bsecReturnLen);
	bme.bsecGetState(bmeState, &bsecReturnLen);

	taskENTER_CRITICAL();
	extMemWriteData(BME_CONFIG_ADDR, bmeConfig, BME_CONFIG_SIZE);
	taskEXIT_CRITICAL();
	taskENTER_CRITICAL();
	extMemWriteData(BME_STATE_ADDR, bmeState, BME_STATE_SIZE);
	taskEXIT_CRITICAL();
}

void recoverBME_StateConfig(){
	uint8_t conditionedSystem = 0;
	extMemGetData(BME_FIRST_RUN_ADDR, &conditionedSystem, BME_FIRST_RUN_SIZE);

	if(conditionedSystem == 0){
		bme.bsecSetConfig(bsec_config_selectivity);
		saveBME_StateConfig();
		conditionedSystem = 1;
		extMemWriteData(BME_FIRST_RUN_ADDR, &conditionedSystem, BME_FIRST_RUN_SIZE);
	}else{
		extMemGetData(BME_CONFIG_ADDR, bmeConfig, BME_CONFIG_SIZE);
		extMemGetData(BME_STATE_ADDR, bmeState, BME_STATE_SIZE);

		bme.bsecSetConfig(bmeConfig);
		bme.bsecSetState(bmeState);
	}

}

//static void triggerBMESample(void *argument) {
//	osThreadFlagsSet(bmeTaskHandle, GRAB_SAMPLE_BIT);
//}
