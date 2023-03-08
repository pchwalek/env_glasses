/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "sgp.h"
#include "SensirionI2CSgp41.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"
#include "sensirion_gas_index_algorithm.h"
#include "sht.h"

#define SGP_SAMPLE_SYS_PERIOD_MS		1000 //how often do we want the system to sample
#define SEND_SGP_EVERY_X_S				1
//#define MAX_SGP_SAMPLES_PACKET	int((SEND_SGP_EVERY_X_S*1000)/SGP_SAMPLE_SYS_PERIOD_MS)
#define MAX_SGP_SAMPLES_PACKET	int(1)

//typedef struct sgpSamples {
//	uint16_t srawVoc;
//	uint32_t srawNox;
//	int32_t voc_index_value;
//	int32_t nox_index_value;
//	uint32_t timestamp;
//} sgpSample;


static void triggerSgpSample(void *argument);
static sgp_packet_payload_t sgpData[20];



//static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicSgpTimer_id;

SensirionI2CSgp41 sgp41;
GasIndexAlgorithmParams paramsNox;
GasIndexAlgorithmParams paramsVoc;


void SgpTask(void *argument) {
	sensor_packet_t *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;
	uint16_t error;

	bool status;

	sgp_sensor_config_t sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(sgp_sensor_config_t));
	}else{
		sensorSettings.sample_period_ms = SGP_SAMPLE_SYS_PERIOD_MS;
	}


    uint16_t defaultRh = 0x8000;
    uint16_t defaultT = 0x6666;

    uint16_t _Rh = 0;
	uint16_t _T = 0;

	int32_t voc_index_value, nox_index_value;

	osDelay(1000);

    GasIndexAlgorithm_init(&paramsVoc, (int32_t) GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&paramsNox, (int32_t) GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	if (!sgp41.begin(&hi2c1)) {
    	osSemaphoreRelease(messageI2C1_LockHandle);
		osDelay(100);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	}

    uint16_t serialNumber[3];
    uint8_t serialNumberSize = 3;
    error = sgp41.getSerialNumber(serialNumber, serialNumberSize);
    while (error) {
    	osSemaphoreRelease(messageI2C1_LockHandle);
		osDelay(10);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
		error = sgp41.getSerialNumber(serialNumber, serialNumberSize);
    }

    uint16_t testResult;
    error = sgp41.executeSelfTest(testResult);
    while(error) {
    	osSemaphoreRelease(messageI2C1_LockHandle);
      	osDelay(10);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
        error = sgp41.executeSelfTest(testResult);
      }
	osSemaphoreRelease(messageI2C1_LockHandle);

    if(testResult != 0xD400){
//    	osSemaphoreRelease(messageI2C1_LockHandle);
    	while(1){
    		osDelay(10000); // TODO: terminate thread instead
    	}
    }



//	header.payloadLength = MAX_SGP_SAMPLES_PACKET * sizeof(sgpSample);

	uint16_t sgpIdx = 0;
	uint32_t sgpID = 0;

	uint16_t srawVOC, srawNOX;

//	uint32_t sgpSample;

	// time in seconds needed for NOx conditioning
	uint16_t conditioning_s = 10;

	periodicSgpTimer_id = osTimerNew(triggerSgpSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicSgpTimer_id, sensorSettings.sample_period_ms);


	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

//			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
//			if(timeLeftForSample < SGP_SAMPLE_SYS_PERIOD_MS){
//				osDelay(timeLeftForSample);
//			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);


			if (conditioning_s > 0) {
				// During NOx conditioning (10s) SRAW NOx will remain 0
				if(shtTemp != -1 && shtHum != -1){
					_Rh = (65535.0 / 100) * shtHum;
					_T = (65535.0 / 175) * (shtTemp+45);
					error = sgp41.executeConditioning(_Rh, _T,  srawVOC);
				}else{
					error = sgp41.executeConditioning(defaultRh, defaultT, srawVOC);
				}
				sgpData[sgpIdx].sraw_nox = 0;
			} else {
				// Read Measurement
				if(shtTemp != -1 && shtHum != -1){
					_Rh = (65535.0 / 100) * shtHum;
					_T = (65535.0 / 175) * (shtTemp+45);
					error = sgp41.measureRawSignals(_Rh, _T, srawVOC, srawNOX);
				}else{
					error = sgp41.measureRawSignals(defaultRh, defaultT, srawVOC, srawNOX);
				}
			}
			osSemaphoreRelease(messageI2C1_LockHandle);

			sgpData[sgpIdx].sraw_voc = srawVOC;
			sgpData[sgpIdx].sraw_nox = srawNOX;

			sgpData[sgpIdx].timestamp_unix = getEpoch();
			sgpData[sgpIdx].timestamp_ms_from_start = HAL_GetTick();

			if(error){
				continue;
			}

			if (conditioning_s > 0)	{
		        conditioning_s--;
			}else{
				GasIndexAlgorithm_process(&paramsNox, sgpData[sgpIdx].sraw_nox, &sgpData[sgpIdx].nox_index_value);
				GasIndexAlgorithm_process(&paramsVoc, sgpData[sgpIdx].sraw_voc, &sgpData[sgpIdx].voc_index_value);
			}

			sgpData[sgpIdx].timestamp_unix = getEpoch();
			sgpData[sgpIdx].timestamp_ms_from_start = HAL_GetTick();

			sgpIdx++;

			if (sgpIdx >= MAX_SGP_SAMPLES_PACKET) {

			//	message.header.payload_length = MAX_LUX_SAMPLES_PACKET * sizeof(luxSample);

//				header.packetType = SGP;
//				header.packetID = sgpID;
//				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {

//					portENTER_CRITICAL();


					setPacketType(packet, SENSOR_PACKET_TYPES_SGP);

					packet->payload.sgp_packet.packet_index = sgpID;
					packet->payload.sgp_packet.sample_period=sensorSettings.sample_period_ms;

					packet->payload.sgp_packet.sensor_id = 0;

//					// reset message buffer
//					memset(&message.payload[0], 0, sizeof(message.payload));

					// write data
					memcpy(packet->payload.sgp_packet.payload, sgpData, sgpIdx * sizeof(sgp_packet_payload_t));
					packet->payload.sgp_packet.payload_count = sgpIdx;

					// encode
//					pb_ostream_t stream = pb_ostream_from_buffer(packet->payload, MAX_PAYLOAD_SIZE);
//					status = pb_encode(&stream, SENSOR_PACKET_FIELDS, &sensorPacket);
//
//					packet->header.payloadLength = stream.bytes_written;

					// send to BT packetizer
					queueUpPacket(packet);
//					portEXIT_CRITICAL();

//					memcpy(&(packet->header), &header, sizeof(PacketHeader));
//					memcpy(packet->payload, sgpData, header.payloadLength);
//					queueUpPacket(packet);
				}
				sgpID++;
				sgpIdx = 0;
			}

//			timeLeftForSample = HAL_GetTick();
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicSgpTimer_id);
			sgp41.turnHeaterOff(); // puts sgp41 in idle mode
			osThreadExit();
			break;
		}
	}
}

static void triggerSgpSample(void *argument) {
	osThreadFlagsSet(sgpTaskHandle, GRAB_SAMPLE_BIT);
}
