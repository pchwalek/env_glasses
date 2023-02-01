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

typedef struct luxSamples {
	uint32_t lux;
	uint32_t timestamp;
} luxSample;


static void triggerLuxSample(void *argument);
static luxSample luxData[MAX_LUX_SAMPLES_PACKET];


//static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicLuxTimer_id;

TSL2772 luxSensor;

static lux_packet message;
void LuxTask(void *argument) {
	SensorPacket *packet = NULL;
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

	message.has_header = true;
	message.header.packet_type = SENSOR_PACKET_TYPES_LUX;
//	message.header.payload_length = MAX_LUX_SAMPLES_PACKET * sizeof(luxSample);
	message.gain = static_cast<tsl2591_gain_t>(sensorSettings.gain);
	message.integration_time = static_cast<tsl2591_integration_time_t>(sensorSettings.integration_time);

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
			luxData[luxIdx].timestamp = HAL_GetTick();
			osSemaphoreRelease(messageI2C3_LockHandle);

			luxIdx++;

			if (luxIdx >= MAX_LUX_SAMPLES_PACKET) {


				message.header.packet_id = luxID;
				message.header.ms_from_start = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {

					packet->header.packetType = LUX;

					// reset message buffer
					memset(&message.payload[0], 0, sizeof(lux_packet_payload_t)*30);

					// write lux data
					memcpy(message.payload, luxData, MAX_LUX_SAMPLES_PACKET * sizeof(luxSample));
					message.payload_count = MAX_LUX_SAMPLES_PACKET;

					// encode
				    pb_ostream_t stream = pb_ostream_from_buffer(packet->payload, MAX_PAYLOAD_SIZE);
				    status = pb_encode(&stream, LUX_PACKET_FIELDS, &message);

				    packet->header.payloadLength = stream.bytes_written;

				    // send to BT packetizer
					queueUpPacket(packet);
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
