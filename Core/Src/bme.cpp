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

#define BME_SAMPLE_PERIOD_MS		1000
#define MAX_BME_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(bme_packet)

typedef struct bme_packets {
	float temperature;
	uint32_t pressure;
	float humidity;
	uint32_t gas_resistance;
	float altitude;
} bme_packet;

static void triggerBMESample(void *argument);

static bme_packet bmeData[MAX_BME_SAMPLES_PACKET];

static PacketHeader header;
//osThreadId_t bmeTaskHandle;
osTimerId_t periodicBMETimer_id;

Adafruit_BME680 bme;

void BME_Task(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;

	if (!bme.begin(BME68X_DEFAULT_ADDRESS, &hi2c1, false)) {
		osDelay(100);
			;
	}

	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms

	header.payloadLength = MAX_BME_SAMPLES_PACKET * sizeof(bme_packet);
	header.reserved[0] = BME_SAMPLE_PERIOD_MS;

	uint16_t bmeIdx = 0;
	uint32_t bmeID = 0;

	periodicBMETimer_id = osTimerNew(triggerBMESample, osTimerPeriodic, NULL,
			NULL);
	osTimerStart(periodicBMETimer_id, BME_SAMPLE_PERIOD_MS);

	while (1) {
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			while (!bme.performReading()) {
				osDelay(10);
			}

			bmeData[bmeIdx].temperature = bme.temperature;
			bmeData[bmeIdx].pressure = bme.pressure / 100.0;
			bmeData[bmeIdx].humidity = bme.humidity;
			bmeData[bmeIdx].gas_resistance = bme.gas_resistance / 1000.0;
			bmeData[bmeIdx].altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

			bmeIdx++;
			if (bmeIdx >= MAX_BME_SAMPLES_PACKET) {
				header.packetType = BME;
				header.packetID = bmeID;
				header.msFromStart = HAL_GetTick();
				packet = grabPacket();
				if (packet != NULL) {
					memcpy(&(packet->header), &header, sizeof(PacketHeader));
					memcpy(packet->payload, bmeData, header.payloadLength);
					queueUpPacket(packet);
				}
				bmeID++;
				bmeIdx = 0;
			}

		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete (periodicBMETimer_id);
			break;
		}
	}
}

static void triggerBMESample(void *argument) {
	osThreadFlagsSet(bmeTaskHandle, GRAB_SAMPLE_BIT);
}

