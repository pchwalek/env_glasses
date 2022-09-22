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

#define MAX_PACKET_LEN DATA_NOTIFICATION_MAX_PACKET_SIZE
#define MAX_PAYLOAD_SIZE 	MAX_PACKET_LEN - sizeof(PacketHeader)
#define MAX_BLE_RETRIES	4

#define PACKET_SEND_SUCCESS			0
#define PACKET_UNDEFINED_ERR		10
//#define PACKET_LENGTH_EXCEEDED	2

#define MAX_PACKET_LEN		DATA_NOTIFICATION_MAX_PACKET_SIZE
//#define MAX_PACKET_QUEUE_SIZE	30


typedef enum {
		PPG_RED            						 = 1,
    PPG_IR           							 = 2,
    SPECTROMETER									 = 3,
    BME       										 = 4,
    CO2										         = 5,
		IMU														 = 6,
		THERMOPILE										 = 7,
		LUX		= 8,
		LIDAR = 9,
		MIC = 10,
		SHT = 11,
		SGP = 12,
} PacketTypes;

typedef struct PacketHeaders{
	PacketTypes packetType;
  uint16_t packetID;
  uint32_t msFromStart;
  uint32_t epoch;
  uint32_t payloadLength;
  uint32_t reserved[5];
}PacketHeader;

typedef struct SensorPackets{
  PacketHeader header;
  uint8_t payload[MAX_PAYLOAD_SIZE]; // should be MAX_PACKET_LEN - sizeof(PacketHeader)
}SensorPacket;

SensorPacket *grabPacket(void);
void queueUpPacket(SensorPacket *packet);
void senderThread(void *argument);
uint8_t sendPacket_BLE(SensorPacket *packet);


#ifdef __cplusplus
}
#endif

#endif /* INC_PACKET_H_ */
