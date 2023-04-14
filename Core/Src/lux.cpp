/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "lux.h"
#include "TSL2772.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"


//#define LUX_SAMPLE_SYS_PERIOD_MS		5000 //how often do we want the system to sample
#define LUX_SAMPLE_SYS_PERIOD_MS		500 //how often do we want the system to sample
#define SEND_LUX_EVERY_X_S				5
//#define MAX_LUX_SAMPLES_PACKET	(SEND_LUX_EVERY_X_S*1000)/LUX_SAMPLE_SYS_PERIOD_MS
#define MAX_LUX_SAMPLES_PACKET	10

//typedef struct luxSamples {
//	uint32_t lux;
//	uint32_t timestamp;
//} luxSample;


static void triggerLuxSample(void *argument);
static lux_packet_payload_t luxData[MAX_LUX_SAMPLES_PACKET];


//static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicLuxTimer_id;

static TSL2772 luxSensor;

static lux_sensor_config_t sensorSettings;

void LuxTask(void *argument) {
	sensor_packet_t *packet = NULL;
	uint32_t flags;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(lux_sensor_config_t));
	}else{
		sensorSettings.gain = TSL2591_GAIN_TSL2722_GAIN_8_X;
		sensorSettings.integration_time = TSL2591_INTEGRATION_TIME_TSL2722_INTEGRATIONTIME_101_MS;
		sensorSettings.sample_period_ms = 500;
	}

	osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
	while (!luxSensor.begin(TSL2772_I2CADDR, &hi2c3)) {
		osDelay(100);
	}
	luxSensor.powerOn(true);

	luxSensor.setATIME((tsl2591IntegrationTime_t) sensorSettings.integration_time);
	luxSensor.setAGAIN((tsl2591Gain_t) sensorSettings.gain);

	luxSensor.enableALS(true);
	i2c_error_check(&hi2c3);
	osSemaphoreRelease(messageI2C3_LockHandle);


	uint16_t luxIdx = 0;
	uint32_t luxID = 0;

	periodicLuxTimer_id = osTimerNew(triggerLuxSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicLuxTimer_id, sensorSettings.sample_period_ms);

	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
			luxData[luxIdx].lux = luxSensor.getLux();
			luxData[luxIdx].timestamp_unix = getEpoch();
			luxData[luxIdx].timestamp_ms_from_start = HAL_GetTick();
			i2c_error_check(&hi2c3);
			osSemaphoreRelease(messageI2C3_LockHandle);

			luxIdx++;

			if (luxIdx >= MAX_LUX_SAMPLES_PACKET) {


				packet = grabPacket();
				while(packet == NULL){
					osDelay(100);
					packet = grabPacket();
				}
				if (packet != NULL) {

					setPacketType(packet, SENSOR_PACKET_TYPES_LUX);

					packet->payload.lux_packet.packet_index = luxID;
					packet->payload.lux_packet.sample_period = sensorSettings.sample_period_ms;
					packet->payload.lux_packet.gain = static_cast<tsl2591_gain_t>(sensorSettings.gain);
					packet->payload.lux_packet.integration_time = static_cast<tsl2591_integration_time_t>(sensorSettings.integration_time);

					packet->payload.lux_packet.sensor_id = 0;

//					// write lux data
					memcpy(packet->payload.lux_packet.payload, luxData, luxIdx * sizeof(lux_packet_payload_t));
					packet->payload.lux_packet.payload_count = luxIdx;

				    // send to BT packetizer
					queueUpPacket(packet, sensorSettings.sample_period_ms);

				}
				luxID++;
				luxIdx = 0;
			}

		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicLuxTimer_id);
			luxSensor.powerOn(false);
			osThreadExit();
			break;
		}
	}
}

static void triggerLuxSample(void *argument) {
	osThreadFlagsSet(luxTaskHandle, GRAB_SAMPLE_BIT);
}
