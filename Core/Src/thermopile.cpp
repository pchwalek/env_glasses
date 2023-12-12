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

//#define THERMOPILE_SAMPLE_PERIOD_MS		1000
#define THERMOPILE_SAMPLE_PERIOD_MS		500
#define THERMOPILE_CNT								2
#define THERMOPILE_CHANNELS				5
//#define MAX_THERMOPILE_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(thermopile_packet)
#define MAX_THERMOPILE_SAMPLES_PACKET  (2*THERMOPILE_CHANNELS)
//typedef struct __attribute__((packed)) thermopile_packets {
//	uint32_t descriptor;
//	uint32_t timestamp_ms_from_start;
//	uint32_t timestamp_unix;
//	uint32_t ambientRaw;
//	uint32_t objectRaw;
//	float ambientTemp;
//	float objectTemp;
//} thermopile_packet;

//typedef struct wristAirThermopiles {
//	thermopile_packet inner;
//	thermopile_packet outer;
//} wristAirThermopile;

void grabThermopileSamples(therm_packet_payload_t *data, CALIPILE *tp);

//static wristAirThermopile thermopileData[MAX_THERMOPILE_SAMPLES_PACKET];
//static thermopile_packet thermopileData[MAX_THERMOPILE_SAMPLES_PACKET];
static therm_packet_payload_t thermopileData[MAX_THERMOPILE_SAMPLES_PACKET];


//static PacketHeader header;

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

void queueThermopilePkt(therm_packet_payload_t *sample, uint16_t packetCnt);
void initThermopiles(CALIPILE *tp, uint8_t address, I2C_HandleTypeDef* i2c_handle, uint8_t descriptor);
void grabThermopileSamples(therm_packet_payload_t *data, CALIPILE *tp);
static void triggerThermopileSample(void *argument);


volatile uint16_t thermIdx;
uint32_t thermID;

thermopile_sensor_config_t sensorSettings;
void Thermopile_Task(void *argument) {
//	sensor_packet_t *packet = NULL;
	uint32_t flags;
	osThreadFlagsClear(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT);
//	bool status;


	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(thermopile_sensor_config_t));
	}else{
		sensorSettings.sample_period_ms = 500;
		sensorSettings.enable_top_of_nose = true;
		sensorSettings.enable_nose_bridge = true;
		sensorSettings.enable_front_temple = true;
		sensorSettings.enable_mid_temple = true;
		sensorSettings.enable_rear_temple = true;
	}

	uint8_t thermopile_samples_per_packet = 2 * (
										((int) (sensorSettings.enable_front_temple)) +
										((int) (sensorSettings.enable_mid_temple)) +
										((int) (sensorSettings.enable_rear_temple)) +
										((int) (sensorSettings.enable_nose_bridge)) +
										((int) (sensorSettings.enable_top_of_nose)));

	if(thermopile_samples_per_packet == 0){
		osThreadExit();
	}

	if(sensorSettings.enable_top_of_nose || sensorSettings.enable_nose_bridge){
		osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
		if(sensorSettings.enable_top_of_nose) initThermopiles(&tp_nose_tip,		THERMOPLE_NOSE_TIP_ADDR,	&hi2c1,	THERMOPILE_LOCATION_TIP_OF_NOSE);
		if(sensorSettings.enable_nose_bridge) initThermopiles(&tp_nose_bridge,	THERMOPLE_NOSE_BRIDGE_ADDR,	&hi2c1, THERMOPILE_LOCATION_NOSE_BRIDGE);
		i2c_error_check(&hi2c1);
		osSemaphoreRelease(messageI2C1_LockHandle);
	}

	if(sensorSettings.enable_front_temple || sensorSettings.enable_mid_temple || sensorSettings.enable_rear_temple){
		osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
		if(sensorSettings.enable_front_temple) initThermopiles(&tp_temple_front,	THERMOPLE_TEMPLE_FRONT_ADDR,&hi2c3, THERMOPILE_LOCATION_FRONT_TEMPLE);
		if(sensorSettings.enable_mid_temple) initThermopiles(&tp_temple_mid,		THERMOPLE_TEMPLE_MID_ADDR,	&hi2c3, THERMOPILE_LOCATION_MID_TEMPLE);
		if(sensorSettings.enable_rear_temple) initThermopiles(&tp_temple_back,	THERMOPLE_TEMPLE_BACK_ADDR,	&hi2c3, THERMOPILE_LOCATION_REAR_TEMPLE);
		i2c_error_check(&hi2c3);
		osSemaphoreRelease(messageI2C3_LockHandle);
	}

	thermIdx = 0;
	thermID = 0;

	periodicThermopileTimer_id = osTimerNew(triggerThermopileSample,
			osTimerPeriodic, NULL, NULL);
	osTimerStart(periodicThermopileTimer_id, sensorSettings.sample_period_ms);

	while (1) {

		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);

		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {


			// sample nose
			if(sensorSettings.enable_top_of_nose || sensorSettings.enable_nose_bridge){
				osSemaphoreAcquire(messageI2C1_LockHandle, osWaitForever);
				if(sensorSettings.enable_top_of_nose) grabThermopileSamples(&thermopileData[thermIdx++], &tp_nose_tip);
				if(sensorSettings.enable_nose_bridge) grabThermopileSamples(&thermopileData[thermIdx++], &tp_nose_bridge);
				i2c_error_check(&hi2c1);
				osSemaphoreRelease(messageI2C1_LockHandle);
			}

			// sample temple
			if(sensorSettings.enable_front_temple || sensorSettings.enable_mid_temple || sensorSettings.enable_rear_temple){
				osSemaphoreAcquire(messageI2C3_LockHandle, osWaitForever);
				if(sensorSettings.enable_front_temple) grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_front);
				if(sensorSettings.enable_mid_temple) grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_mid);
				if(sensorSettings.enable_rear_temple) grabThermopileSamples(&thermopileData[thermIdx++], &tp_temple_back);
				i2c_error_check(&hi2c3);
				osSemaphoreRelease(messageI2C3_LockHandle);
			}

			if(thermIdx >= thermopile_samples_per_packet){
				queueThermopilePkt(&thermopileData[0], thermIdx);
				thermID++;
				thermIdx = 0;
			}

		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicThermopileTimer_id);
			osThreadExit();
			break;
		}
	}

}

void initThermopiles(CALIPILE *tp, uint8_t address, I2C_HandleTypeDef* i2c_handle, uint8_t descriptor){

	uint16_t Tcounts = 0x83; // set threshold for over temperature interrupt, 0x83 == 67072 counts
//	uint32_t flags = 0;
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

void queueThermopilePkt(therm_packet_payload_t *sample, uint16_t packetCnt){
	sensor_packet_t *packet = NULL;
//	thermIdx+=packetCnt;

//	bool status;


//	if (thermIdx >= MAX_THERMOPILE_SAMPLES_PACKET) {
		packet = grabPacket();
		if (packet != NULL) {

			setPacketType(packet, SENSOR_PACKET_TYPES_THERMOPILE);

			packet->payload.therm_packet.packet_index = thermID;
			packet->payload.therm_packet.sample_period = sensorSettings.sample_period_ms;
			// reset message buffer
//			memset(&sensorPacket.therm_packet.payload[0], 0, sizeof(sensorPacket.therm_packet.payload));

			// write data
			memcpy(packet->payload.therm_packet.payload, sample, packetCnt * sizeof(therm_packet_payload_t));
			packet->payload.therm_packet.payload_count = packetCnt;

//			// encode
//			pb_ostream_t stream = pb_ostream_from_buffer(packet->payload, MAX_PAYLOAD_SIZE);
//			status = pb_encode(&stream, SENSOR_PACKET_FIELDS, &sensorPacket);
//
//			packet->header.payloadLength = stream.bytes_written;


			// send to BT packetizer
			queueUpPacket(packet, 200);

		}
//		thermID++;
//		thermIdx = 0;
//	}
}

float convertKelvinToCelsius(float temperature){
	return temperature - 273.15;
}

void grabThermopileSamples(therm_packet_payload_t *data, CALIPILE *tp) {
	data->descriptor = static_cast<thermopile_location_t>(tp->descriptor);
	data->timestamp_unix = getEpoch();
	data->timestamp_ms_from_start = HAL_GetTick();
	data->ambient_raw = tp->getTPAMB();
	data->object_raw = tp->getTPOBJ();
	data->ambient_temp = tp->getTamb(data->ambient_raw);
	data->object_temp = convertKelvinToCelsius(tp->getTobj(data->object_raw, data->ambient_temp));
	data->ambient_temp = convertKelvinToCelsius(data->ambient_temp);
}

static void triggerThermopileSample(void *argument) {
	osThreadFlagsSet(thermopileTaskHandle, GRAB_SAMPLE_BIT);
}
