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
#define MAX_LUX_SAMPLES_PACKET	1

//typedef struct luxSamples {
//	uint32_t lux;
//	uint32_t timestamp;
//} luxSample;


static void triggerLuxSample(void *argument);
static lux_packet_payload_t luxData[MAX_LUX_SAMPLES_PACKET];


//static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicLuxTimer_id;

TSL2772 luxSensor;

void LuxTask(void *argument) {
	sensor_packet_t *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;

	bool status;

	struct LuxSensor sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct LuxSensor));
	}else{
		sensorSettings.gain = TSL2722_GAIN_8X;
		sensorSettings.integration_time = TSL2722_INTEGRATIONTIME_101MS;
		sensorSettings.sample_period = 500;
	}

	osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
	while (!luxSensor.begin(TSL2772_I2CADDR, &hi2c3)) {
		osDelay(100);
	}
	luxSensor.powerOn(true);

	luxSensor.setATIME((tsl2591IntegrationTime_t) sensorSettings.integration_time);
	luxSensor.setAGAIN((tsl2591Gain_t) sensorSettings.gain);

	luxSensor.enableALS(true);


//	message.header.payload_length = MAX_LUX_SAMPLES_PACKET * sizeof(luxSample);


	uint16_t luxIdx = 0;
	uint32_t luxID = 0;

	uint32_t luxSample;

	osSemaphoreRelease(messageI2C3_LockHandle);
	periodicLuxTimer_id = osTimerNew(triggerLuxSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicLuxTimer_id, sensorSettings.sample_period);

	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
//
//			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
//			if(timeLeftForSample < LUX_SAMPLE_SYS_PERIOD_MS){
//				osDelay(timeLeftForSample);
//			}

			osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
			luxData[luxIdx].lux = luxSensor.getLux();
			luxData[luxIdx].timestamp_unix = getEpoch();
			luxData[luxIdx].timestamp_ms_from_start = HAL_GetTick();
			osSemaphoreRelease(messageI2C3_LockHandle);

			luxIdx++;

			if (luxIdx >= MAX_LUX_SAMPLES_PACKET) {


				packet = grabPacket();
				if (packet != NULL) {

//					portENTER_CRITICAL();

					setPacketType(packet, SENSOR_PACKET_TYPES_LUX);

					packet->payload.lux_packet.packet_index = luxID;
					packet->payload.lux_packet.sample_period = sensorSettings.sample_period;
					packet->payload.lux_packet.gain = static_cast<tsl2591_gain_t>(sensorSettings.gain);
					packet->payload.lux_packet.integration_time = static_cast<tsl2591_integration_time_t>(sensorSettings.integration_time);

					// reset message buffer
//					memset(&sensorPacket.lux_packet.payload[0], 0, sizeof(lux_packet_payload_t)*30);

					// write lux data
					memcpy(packet->payload.lux_packet.payload, luxData, luxIdx * sizeof(lux_packet_payload_t));
					packet->payload.lux_packet.payload_count = luxIdx;

//					// encode
//				    pb_ostream_t stream = pb_ostream_from_buffer(packet->payload, MAX_PAYLOAD_SIZE);
//				    status = pb_encode(&stream, SENSOR_PACKET_FIELDS, &sensorPacket);

//				    packet->header.payloadLength = stream.bytes_written;

				    // send to BT packetizer
					queueUpPacket(packet);

//					portEXIT_CRITICAL();

				}
				luxID++;
				luxIdx = 0;
			}

//			timeLeftForSample = HAL_GetTick();
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
