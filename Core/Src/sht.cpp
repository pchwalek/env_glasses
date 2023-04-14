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


//#define SHT_SAMPLE_SYS_PERIOD_MS		5000 //how often do we want the system to sample
#define SEND_SHT_EVERY_X_S				5
//#define MAX_SHT_SAMPLES_PACKET	int((SEND_SHT_EVERY_X_S*1000)/SHT_SAMPLE_SYS_PERIOD_MS)

#define SHT_SAMPLE_SYS_PERIOD_MS		500 //how often do we want the system to sample
#define MAX_SHT_SAMPLES_PACKET	2

//typedef struct shtSamples {
//	float temp;
//	float hum;
//	uint32_t timestamp;
//} shtSample;


static void triggerShtSample(void *argument);
static sht_packet_payload_t shtData[MAX_SHT_SAMPLES_PACKET];


//static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicShtTimer_id;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#ifdef SECONDARY_ENV_SENSOR_EXPANSION
Adafruit_SHT4x sht4_secondary = Adafruit_SHT4x();
static sht_packet_payload_t shtData_secondary[MAX_SHT_SAMPLES_PACKET];
#endif

float shtTemp, shtHum;

void ShtTask(void *argument) {
	sensor_packet_t *packet = NULL;
	uint32_t flags;
//	uint32_t timeLeftForSample = 0;

//	bool status;

	uint8_t errorCnt;

	uint8_t primarySHT_disable = 0;
	uint8_t secondarySHT_disable = 0;

	uint8_t skipPrimaryPacket = 1;
	uint8_t skipSecondaryPacket = 1;
	shtTemp = -1;
	shtHum = -1;

	humidity_sensor_config_t sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct HumiditySensor));
	}else{
		sensorSettings.heater_settings = SHT45_HEATER_SHT4_X_LOW_HEATER_100_MS;
		sensorSettings.precision_level = SHT45_PRECISION_SHT4_X_HIGH_PRECISION;
		sensorSettings.sample_period_ms = SHT_SAMPLE_SYS_PERIOD_MS;
	}

	errorCnt = 0;
	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	while (!sht4.begin(&hi2c1)) {
		i2c_error_check(&hi2c1);
		osSemaphoreRelease(messageI2C1_LockHandle);
		errorCnt++;
		if(errorCnt > 10){
			primarySHT_disable = 1;
			break;
		}
		osDelay(100);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	}

	sht4.setPrecision( (sht4x_precision_t) sensorSettings.precision_level);
	sht4.setHeater( (sht4x_heater_t) sensorSettings.heater_settings);

	i2c_error_check(&hi2c1);
	osSemaphoreRelease(messageI2C1_LockHandle);

#ifdef SECONDARY_ENV_SENSOR_EXPANSION
	errorCnt = 0;
	osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
	while (!sht4_secondary.begin(&hi2c3)) {
		i2c_error_check(&hi2c3);
		osSemaphoreRelease(messageI2C3_LockHandle);
		errorCnt++;
		if(errorCnt > 10){
			secondarySHT_disable = 1;
			break;
		}
		osDelay(100);
		osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
	}

	sht4_secondary.setPrecision( (sht4x_precision_t) sensorSettings.precision_level);
	sht4_secondary.setHeater( (sht4x_heater_t) sensorSettings.heater_settings);

	i2c_error_check(&hi2c3);
	osSemaphoreRelease(messageI2C3_LockHandle);

#endif

	// if both sensors failed to initialize
	if((primarySHT_disable) & (secondarySHT_disable)){
		osThreadExit();
	}

	uint16_t shtIdx = 0;
	uint32_t shtID = 0;
	uint32_t shtID_secondary = 0;

	periodicShtTimer_id = osTimerNew(triggerShtSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicShtTimer_id, sensorSettings.sample_period_ms);


	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			//			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
			//			if(timeLeftForSample < SHT_SAMPLE_SYS_PERIOD_MS){
			//				osDelay(timeLeftForSample);
			//			}
			if(!primarySHT_disable){
				osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
				if(sht4.getEvent()){
					//			if(1){
					i2c_error_check(&hi2c1);
					osSemaphoreRelease(messageI2C1_LockHandle);

					shtData[shtIdx].temperature = sht4._temperature;
					shtData[shtIdx].humidity = sht4._humidity;
					shtData[shtIdx].timestamp_ms_from_start = HAL_GetTick();
					shtData[shtIdx].timestamp_unix = getEpoch();
					shtTemp = shtData[shtIdx].temperature;
					shtHum = shtData[shtIdx].humidity;
				}else{
					i2c_error_check(&hi2c1);
					osSemaphoreRelease(messageI2C1_LockHandle);
					skipPrimaryPacket = 1;
				}

			}
#ifdef SECONDARY_ENV_SENSOR_EXPANSION
			if(!secondarySHT_disable){
				osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
				if(sht4_secondary.getEvent()){
					i2c_error_check(&hi2c3);
					osSemaphoreRelease(messageI2C3_LockHandle);

					//			if(1){
					shtData_secondary[shtIdx].temperature = sht4_secondary._temperature;
					shtData_secondary[shtIdx].humidity = sht4_secondary._humidity;
					shtData_secondary[shtIdx].timestamp_ms_from_start = HAL_GetTick();
					shtData_secondary[shtIdx].timestamp_unix = getEpoch();
					shtTemp = shtData_secondary[shtIdx].temperature;
					shtHum = shtData_secondary[shtIdx].humidity;
				}else{
					i2c_error_check(&hi2c3);
					osSemaphoreRelease(messageI2C3_LockHandle);
					skipSecondaryPacket = 1;
				}

			}
#endif

			shtIdx++;

			if (shtIdx >= MAX_SHT_SAMPLES_PACKET) {
				//				header.packetType = SHT;
				if(!primarySHT_disable && !skipPrimaryPacket){
					packet = grabPacket();
					if (packet != NULL) {

						setPacketType(packet, SENSOR_PACKET_TYPES_SHT);

						packet->payload.sht_packet.precision = static_cast<sht45_precision_t>(sensorSettings.precision_level);
						packet->payload.sht_packet.heater = static_cast<sht45_heater_t>(sensorSettings.heater_settings);
						packet->payload.sht_packet.packet_index = shtID;
						packet->payload.sht_packet.sensor_id = 0;

						// write data
						memcpy(packet->payload.sht_packet.payload, shtData, shtIdx * sizeof(sht_packet_payload_t));
						packet->payload.sht_packet.payload_count = shtIdx;

						// send to BT packetizer
						queueUpPacket(packet, sensorSettings.sample_period_ms >> 1);


						shtID++;

					}
				}

#ifdef SECONDARY_ENV_SENSOR_EXPANSION
				if(!secondarySHT_disable && !skipSecondaryPacket){
					packet = grabPacket();
					if (packet != NULL) {

						setPacketType(packet, SENSOR_PACKET_TYPES_SHT);

						packet->payload.sht_packet.precision = static_cast<sht45_precision_t>(sensorSettings.precision_level);
						packet->payload.sht_packet.heater = static_cast<sht45_heater_t>(sensorSettings.heater_settings);
						packet->payload.sht_packet.packet_index = shtID_secondary;
						packet->payload.sht_packet.sensor_id = 1;

						// write data
						memcpy(packet->payload.sht_packet.payload, shtData_secondary, shtIdx * sizeof(sht_packet_payload_t));
						packet->payload.sht_packet.payload_count = shtIdx;

						// send to BT packetizer
						queueUpPacket(packet, sensorSettings.sample_period_ms >> 1);

						shtID_secondary++;
					}
				}
#endif

				skipPrimaryPacket = 0;
				skipSecondaryPacket = 0;
				shtIdx = 0;
			}

		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicShtTimer_id);
			sht4.reset();
#ifdef SECONDARY_ENV_SENSOR_EXPANSION
			sht4_secondary.reset();
#endif
			osThreadExit();
			break;
		}
	}
}

static void triggerShtSample(void *argument) {
	osThreadFlagsSet(shtTaskHandle, GRAB_SAMPLE_BIT);
}
