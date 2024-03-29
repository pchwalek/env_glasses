/*
 * packet.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "app_conf.h"
#include "cmsis_os2.h"
#include "captivate_config.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "message.pb.h"

#define MAX_PACKET_LEN DATA_NOTIFICATION_MAX_PACKET_SIZE
//#define MAX_PAYLOAD_SIZE 	MAX_PACKET_LEN - sizeof(PacketHeader)
#define MAX_PAYLOAD_SIZE 	600
#define MAX_BLE_RETRIES	2

#define PACKET_SEND_SUCCESS			0
#define PACKET_UNDEFINED_ERR		10
//#define PACKET_LENGTH_EXCEEDED	2

#define MAX_PACKET_LEN		DATA_NOTIFICATION_MAX_PACKET_SIZE
//#define MAX_PACKET_QUEUE_SIZE	30

typedef enum {
	PPG_RED = 1,
	PPG_IR = 2,
	SPECTROMETER = 3,
	BME = 4,
	CO2 = 5,
	IMU = 6,
	THERMOPILE = 7,
	LUX = 8,
	LIDAR = 9,
	MIC = 10,
	SHT = 11,
	SGP = 12,
	BLINK = 13,
} PacketTypes;


#define CONTROL_LED_PKT_TYPE	1
#define SET_CLK_PKT_TYPE		2
#define CNTRL_SENSORS_PKT_TYPE	3
#define CONFIG_SENSORS_PKT_TYPE	4
#define TRIGGER_FUNC_PKT_TYPE	5

typedef struct RX_PacketHeaders {
	uint16_t packetType;
	uint16_t payloadSize;
	uint32_t epoch;
} RX_PacketHeader;

typedef struct __attribute__((packed)) FRAM_Packets{
	uint32_t memory_addr;
	uint16_t size;
} FRAM_Packet;

extern sensor_packet_t sensorPacket;

void setPacketType(sensor_packet_t* packetPtr,sensor_packet_types_t type);
sensor_packet_t* grabPacket(void);
void queueUpPacket(sensor_packet_t *packet, uint32_t timeout);
void senderThread(void *argument);
//uint8_t sendPacket_BLE(sensor_packet_t *packet);
uint8_t sendProtobufPacket_BLE(uint8_t *packet, uint16_t size);
uint8_t updateSystemConfig_BLE(system_state_t *packet);
void updateRTC(uint32_t receivedTime);
void updateRTC_MS(uint64_t receivedTime);
uint64_t getEpoch(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PACKET_H_ */
