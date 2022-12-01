/*
 * packet.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#include "packet.h"
#include "dts.h"
#include "dt_server_app.h"
#include "uuid.h"
#include "dt_server_app.h"

#include "FreeRTOS.h"
#include "task.h"


static SensorPacket packets[MAX_PACKET_QUEUE_SIZE];
SensorPacket *packetPtr[MAX_PACKET_QUEUE_SIZE];

SensorPacket* grabPacket(void) {
	SensorPacket *packet;
	// grab available memory for packet creation
	if (osOK != osMessageQueueGet(packetAvail_QueueHandle, &packet, 0U, 0)) {
		return NULL;
	}
	return packet;
}

void queueUpPacket(SensorPacket *packet) {
	// put into queue
	osMessageQueuePut(packet_QueueHandle, &packet, 0U, 0);
}

SensorPacket *packetToSend;
void senderThread(void *argument) {
	uint8_t retry;

	for (int i = 0; i < MAX_PACKET_QUEUE_SIZE; i++) {
		packetToSend = &packets[i];
		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
				osWaitForever);
	}

	while (1) {
		osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever);

		retry = 0;
//		taskENTER_CRITICAL();
		while (PACKET_SEND_SUCCESS != sendPacket_BLE(packetToSend)) {
			if (retry >= MAX_BLE_RETRIES) {
				break;
			}
			retry++;
//			osDelay(5);
		};
//		taskEXIT_CRITICAL();


		// return memory back to pool
		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
				osWaitForever);

//		osDelay(100);
//		osDelay(1);

//		osDelay(MAX_BLE_RETRIES - retry); // artificial delay to allow for the connected device to handle the latest sent packet
	}
}

static DTS_App_Context_t DataTransferServerContext;
uint8_t sendPacket_BLE(SensorPacket *packet) {

	if ((packet->header.payloadLength) > MAX_PAYLOAD_SIZE) {
		return PACKET_LENGTH_EXCEEDED;
	}

	tBleStatus status = BLE_STATUS_INVALID_PARAMS;
//	uint8_t crc_result;
//
//	/* compute CRC */
//	crc_result = APP_BLE_ComputeCRC8((uint8_t*) Notification_Data_Buffer,
//			(DATA_NOTIFICATION_MAX_PACKET_SIZE - 1));
//	Notification_Data_Buffer[DATA_NOTIFICATION_MAX_PACKET_SIZE - 1] =
//			crc_result;

	DataTransferServerContext.TxData.pPayload = (uint8_t*) packet;
	DataTransferServerContext.TxData.Length = packet->header.payloadLength
			+ sizeof(PacketHeader); //Att_Mtu_Exchanged-10;

//	if(packet->header.packetType==PPG_RED || packet->header.packetType==PPG_IR){
//		status = Generic_STM_UpdateChar(PPG_CHAR_UUID_DEF,
//	(uint8_t*) &DataTransferServerContext.TxData);
//	}

	//COMMENTED BELOW ON 9/20/2022 BUT NEED TO FIX
	status = DTS_STM_UpdateChar(DATA_TRANSFER_TX_CHAR_UUID,
			(uint8_t*) &DataTransferServerContext.TxData);

	if (status == BLE_STATUS_SUCCESS) {
		return PACKET_SEND_SUCCESS;
	} else {
		return PACKET_UNDEFINED_ERR;
	}
}
