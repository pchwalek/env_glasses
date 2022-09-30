/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "thermopile.h"
#include "CaliPile.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"

#define THERMOPILE_SAMPLE_PERIOD_MS		1000
#define THERMOPILE_CNT								2
#define THERMOPILE_CHANNELS				5
#define MAX_THERMOPILE_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(thermopile_packet)

typedef struct thermopile_packets {
	uint8_t descriptor;
	uint32_t timestamp;
	uint16_t ambientRaw;
	uint32_t objectRaw;
	float ambientTemp;
	float objectTemp;
} thermopile_packet;

//typedef struct wristAirThermopiles {
//	thermopile_packet inner;
//	thermopile_packet outer;
//} wristAirThermopile;

void grabThermopileSamples(thermopile_packet *data, CALIPILE *tp);

//static wristAirThermopile thermopileData[MAX_THERMOPILE_SAMPLES_PACKET];
//static thermopile_packet thermopileData[MAX_THERMOPILE_SAMPLES_PACKET];
static thermopile_packet thermopileData[THERMOPILE_CHANNELS];


static PacketHeader header;

//osThreadId_t thermopileTaskHandle;
osTimerId_t periodicThermopileTimer_id;

CALIPILE tp_nose_tip;
CALIPILE tp_nose_bridge;
CALIPILE tp_temple_front;
CALIPILE tp_temple_mid;
CALIPILE tp_temple_back;

uint8_t wakeupFlag = 0;

#define THERMOPLE_NOSE_TIP_ADDR			0x0C
#define THERMOPLE_NOSE_BRIDGE_ADDR		0x0D
#define THERMOPLE_TEMPLE_FRONT_ADDR		0x0F
#define THERMOPLE_TEMPLE_MID_ADDR		0x0E
#define THERMOPLE_TEMPLE_BACK_ADDR		0x0C

#define THERMOPLE_NOSE_TIP_ID				1
#define THERMOPLE_NOSE_BRIDGE_ID			2
#define THERMOPLE_TEMPLE_FRONT_ADDR_ID		3
#define THERMOPLE_TEMPLE_MID_ADDR_ID		4
#define THERMOPLE_TEMPLE_BACK_ADDR_ID		5

void queueThermopilePkt(thermopile_packet *sample, uint16_t packetCnt);
void initThermopiles(CALIPILE *tp, uint8_t address, I2C_HandleTypeDef* i2c_handle, uint8_t descriptor);
void grabThermopileSamples(thermopile_packet *data, CALIPILE *tp);

uint16_t thermIdx;
uint32_t thermID;

void Thermopile_Task(void *argument) {
	SensorPacket *packet = NULL;
	uint32_t flags;

//	tp_nose_bridge.setup((uint8_t) THERMOPLE_NOSE_BRIDGE_ADDR, &hi2c1, THERMOPLE_NOSE_BRIDGE_ID);
//	tp_nose_bridge.wake(); 		// wakeup thermopile sensors on i2c1 bus
//	tp_temple_front.setup((uint8_t) THERMOPLE_TEMPLE_FRONT_ADDR, &hi2c3, THERMOPLE_TEMPLE_FRONT_ADDR_ID);
//	tp_temple_front.wake(); 	// wakeup thermopile sensors on i2c3 bus

	osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
	initThermopiles(&tp_nose_tip,		THERMOPLE_NOSE_TIP_ADDR,	&hi2c1,	THERMOPLE_NOSE_TIP_ID);
	initThermopiles(&tp_nose_bridge,	THERMOPLE_NOSE_BRIDGE_ADDR,	&hi2c1, THERMOPLE_NOSE_BRIDGE_ID);
	initThermopiles(&tp_temple_front,	THERMOPLE_TEMPLE_FRONT_ADDR,&hi2c3, THERMOPLE_TEMPLE_FRONT_ADDR_ID);
	initThermopiles(&tp_temple_mid,		THERMOPLE_TEMPLE_MID_ADDR,	&hi2c3, THERMOPLE_TEMPLE_MID_ADDR_ID);
	initThermopiles(&tp_temple_back,	THERMOPLE_TEMPLE_BACK_ADDR,	&hi2c3, THERMOPLE_TEMPLE_BACK_ADDR_ID);
	osSemaphoreRelease(messageI2C1_LockHandle);

	header.payloadLength = THERMOPILE_CHANNELS * sizeof(thermopile_packet);
	header.reserved[0] = THERMOPILE_SAMPLE_PERIOD_MS;
	header.reserved[1] = THERMOPILE_CNT;

	thermIdx = 0;
	thermID = 0;

	periodicThermopileTimer_id = osTimerNew(triggerThermopileSample,
			osTimerPeriodic, NULL, NULL);
	osTimerStart(periodicThermopileTimer_id, THERMOPILE_SAMPLE_PERIOD_MS);

	while (1) {

		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {

			osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);

			// sample nose
			grabThermopileSamples(&thermopileData[thermIdx++], &tp_nose_tip);
			grabThermopileSamples(&thermopileData[thermIdx++], &tp_nose_bridge);

			// sample temple
			grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_front);
			grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_mid);
			grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_back);

			queueThermopilePkt(&thermopileData[0], 5);
			thermIdx = 0;

			osSemaphoreRelease(messageI2C1_LockHandle);
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicThermopileTimer_id);
			break;
		}
	}

}

void initThermopiles(CALIPILE *tp, uint8_t address, I2C_HandleTypeDef* i2c_handle, uint8_t descriptor){

	uint16_t Tcounts = 0x83; // set threshold for over temperature interrupt, 0x83 == 67072 counts
	uint32_t flags = 0;
	//	uint8_t intStatus;


	tp->setup((uint8_t) address, i2c_handle, descriptor);

//	if(wakeupFlag==0){
//		tp->wake();
//		wakeupFlag=1;
//	}
	tp->wake();
	tp->readEEPROM(); // Verify protocol number and checksum and get calibration constants
	//  tp_outer.initMotion(tcLP1, tcLP2, LPsource, cycTime); // configure presence and motion interrupts
	tp->initTempThr(Tcounts);  // choose something ~5% above TPAMB
	// read interrupt status register(s) to unlatch interrupt before entering main loop
//	intStatus  = tp.checkIntStatus(); //dont do if not using interrupts

}

void queueThermopilePkt(thermopile_packet *sample, uint16_t packetCnt){
	SensorPacket *packet = NULL;
	thermIdx++;

//	if (thermIdx >= MAX_THERMOPILE_SAMPLES_PACKET) {
		header.packetType = THERMOPILE;
		header.packetID = thermID;
		header.msFromStart = HAL_GetTick();
		packet = grabPacket();
		if (packet != NULL) {
			memcpy(&(packet->header), &header, sizeof(PacketHeader));
			memcpy(packet->payload, sample, header.payloadLength);
			queueUpPacket(packet);
		}
		thermID++;
		thermIdx = 0;
//	}
}

void grabThermopileSamples(thermopile_packet *data, CALIPILE *tp) {
	data->descriptor = tp->descriptor;
	data->timestamp = HAL_GetTick();
	data->ambientRaw = tp->getTPAMB();
	data->objectRaw = tp->getTPOBJ();
	data->ambientTemp = tp->getTamb(data->ambientRaw);
	data->objectTemp = tp->getTobj(data->objectRaw, data->ambientTemp);
}

static void triggerThermopileSample(void *argument) {
	osThreadFlagsSet(thermopileTaskHandle, GRAB_SAMPLE_BIT);
}
