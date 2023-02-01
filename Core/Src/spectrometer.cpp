/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "spectrometer.h"
#include "Adafruit_AS7341.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"

#define SPEC_ATIME								100
#define SPEC_ASTEP								999
#define SPEC_SAMPLE_PERIOD_MS			(uint32_t)(((SPEC_ATIME+1)*(SPEC_ASTEP+1)*2.78)/1000)
#define SPEC_SAMPLE_PERIOD_S			(uint32_t)(((SPEC_ATIME+1)*(SPEC_ASTEP+1)*2.78)/1000000)
//#define SPEC_SAMPLE_SYS_PERIOD_MS		5000 //how often do we want the system to sample
#define SPEC_SAMPLE_SYS_PERIOD_MS		2000 //how often do we want the system to sample
#define SEND_SPEC_EVERY_X_S				5
//#define MAX_SPEC_SAMPLES_PACKET	int((SEND_SPEC_EVERY_X_S*1000)/SPEC_SAMPLE_SYS_PERIOD_MS)
#define MAX_SPEC_SAMPLES_PACKET	int(1)
#define SPEC_FLICKER_DELAY				510

typedef struct specSamples {
	uint32_t _415;
	uint32_t _445;
	uint32_t _480;
	uint32_t _515;
	uint32_t _clear_1;
	uint32_t _nir_1;
	uint32_t _555;
	uint32_t _590;
	uint32_t _630;
	uint32_t _680;
	uint32_t _clear_2;
	uint32_t _nir_2;
	uint32_t flicker;
} specSample;

typedef struct specSamplePkts{
	union SPEC_UNION {
		specSample s;
		uint32_t s_array[13];
	} data;
	uint32_t timestamp;
} specSamplePkt;

static void triggerSpectrometerSample(void *argument);

//static SPEC_UNION specData[MAX_SPEC_SAMPLES_PACKET];
static specSamplePkt specData[MAX_SPEC_SAMPLES_PACKET];

static PacketHeader header;
//osThreadId_t specTaskHandle;
osTimerId_t periodicSpecTimer_id;

Adafruit_AS7341 specSensor;

static spec_packet message;
void Spec_Task(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;
	uint32_t timeLeftForSample = 0;

	bool status;

	osDelay(500);

	struct ColorSensor sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct ColorSensor));
	}else{
		sensorSettings.integrationTime = 100;
		sensorSettings.integrationStep = 999;
		sensorSettings.gain = AS7341_GAIN_256X;
		sensorSettings.sample_period = SPEC_SAMPLE_SYS_PERIOD_MS;
	}

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	while (!specSensor.begin(SPEC_ADDR, &hi2c1, 0)) {
		osSemaphoreRelease(messageI2C1_LockHandle);
		osDelay(100);
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	}
	specSensor.setATIME(sensorSettings.integrationTime);
	specSensor.setASTEP(sensorSettings.integrationStep);
	specSensor.setGain((as7341_gain_t) sensorSettings.gain);

	message.header.payload_length = MAX_SPEC_SAMPLES_PACKET * sizeof(specSamplePkt);
	message.sample_period = sensorSettings.sample_period;
//	message.spec_send_freq = SEND_SPEC_EVERY_X_S;

	message.header.packet_type = SENSOR_PACKET_TYPES_SPECTROMETER;

	uint16_t specIdx = 0;
	uint32_t specID = 0;

//	uint32_t tempTick = 0;

	specSensor.startReading();
	osSemaphoreRelease(messageI2C1_LockHandle);
	periodicSpecTimer_id = osTimerNew(triggerSpectrometerSample, osTimerPeriodic,
			NULL, NULL);
	osTimerStart(periodicSpecTimer_id, sensorSettings.sample_period);

	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

//			timeLeftForSample = HAL_GetTick() - timeLeftForSample;
//			if(timeLeftForSample < SPEC_SAMPLE_PERIOD_MS){
//				osDelay(timeLeftForSample);
//			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			while (!specSensor.checkReadingProgress()) {
				osSemaphoreRelease(messageI2C1_LockHandle);
				osDelay(5);
				osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			}

			specData[specIdx].timestamp = HAL_GetTick();

			specSensor.getAllChannels(specData[specIdx].data.s_array);

//			specData[specIdx].data.s.flicker = specSensor.detectFlickerHz();

			osSemaphoreRelease(messageI2C1_LockHandle);

			specIdx++;

			if (specIdx >= MAX_SPEC_SAMPLES_PACKET) {

				message.header.packet_id = specID;
				message.header.ms_from_start = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {

					packet->header.packetType = SPECTROMETER;

					// reset message buffer
					memset(&message.payload[0], 0, sizeof(message.payload));

					// write data
					memcpy(message.payload, specData, message.header.payload_length);
					message.payload_count = MAX_SPEC_SAMPLES_PACKET;

					// encode
					pb_ostream_t stream = pb_ostream_from_buffer(packet->payload, MAX_PAYLOAD_SIZE);
					status = pb_encode(&stream, SPEC_PACKET_FIELDS, &message);

					packet->header.payloadLength = stream.bytes_written;

					// send to BT packetizer
					queueUpPacket(packet);


//					memcpy(&(packet->header), &header, sizeof(PacketHeader));
//					memcpy(packet->payload, specData, header.payloadLength);
//					queueUpPacket(packet);
				}
				specID++;
				specIdx = 0;
			}

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
			specSensor.startReading();
			osSemaphoreRelease(messageI2C1_LockHandle);

//			timeLeftForSample = HAL_GetTick();
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicSpecTimer_id);

			specSensor.powerEnable(false);

			osThreadExit();
			break;
		}
	}
}

static void triggerSpectrometerSample(void *argument) {
	osThreadFlagsSet(specTaskHandle, GRAB_SAMPLE_BIT);
}
