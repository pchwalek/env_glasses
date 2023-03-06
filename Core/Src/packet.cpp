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
#include "rtc.h"

#include "app_ble.h"
#include "fram.h"

#define JULIAN_DATE_BASE     2440588   // Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)
static const uint16_t week_day[] = { 0x4263, 0xA8BD, 0x42BF, 0x4370, 0xABBF, 0xA8BF, 0x43B2 };

// https://github.com/LonelyWolf/stm32/blob/master/stm32l-dosfs/RTC.c
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint64_t RTC_ToEpochMS(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

static sensor_packet_t packets[MAX_PACKET_QUEUE_SIZE];

//static uint8_t packets[MAX_PACKET_QUEUE_SIZE][600];

sensor_packet_t *packetPtr[MAX_PACKET_QUEUE_SIZE];

sensor_packet_t* grabPacket(void) {
	sensor_packet_t *packet;
	// grab available memory for packet creation
	if (osOK != osMessageQueueGet(packetAvail_QueueHandle, &packet, 0U, 0)) {
		return NULL;
	}
	return packet;
}

void queueUpPacket(sensor_packet_t *packet) {
	// put into queue
	osMessageQueuePut(packet_QueueHandle, &packet, 0U, 0);
}

sensor_packet_t *packetToSend;
sensor_packet_t backupPacket;
CircularBuffer* backupBuffer;

sensor_packet_t sensorPacket = SENSOR_PACKET_INIT_ZERO;

void setPacketType(sensor_packet_t* packetPtr,sensor_packet_types_t type){
	packetPtr->has_header = true;

//	packetPtr->has_lux_packet = false;
//	packetPtr->has_sgp_packet = false;
//	packetPtr->has_bme_packet = false;
//	packetPtr->has_blink_packet = false;
//	packetPtr->has_sht_packet = false;
//	packetPtr->has_spec_packet = false;
//	packetPtr->has_therm_packet = false;
//	packetPtr->has_imu_packet = false;
//	packetPtr->has_mic_packet = false;


	switch(type){
		case SENSOR_PACKET_TYPES_SPECTROMETER :
			packetPtr->which_payload = SENSOR_PACKET_SPEC_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_BME :
			packetPtr->which_payload = SENSOR_PACKET_BME_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_IMU :
			packetPtr->which_payload = SENSOR_PACKET_IMU_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_THERMOPILE :
			packetPtr->which_payload = SENSOR_PACKET_THERM_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_LUX :
			packetPtr->which_payload = SENSOR_PACKET_LUX_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_MIC :
			packetPtr->which_payload = SENSOR_PACKET_MIC_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_SHT :
			packetPtr->which_payload = SENSOR_PACKET_SHT_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_SGP :
			packetPtr->which_payload = SENSOR_PACKET_SGP_PACKET_TAG;
			break;

		case SENSOR_PACKET_TYPES_BLINK :
			packetPtr->which_payload = SENSOR_PACKET_BLINK_PACKET_TAG;
			break;

		default:
			break;
	}
}


static uint8_t encoded_payload[MAX_PAYLOAD_SIZE];
pb_ostream_t stream;

void senderThread(void *argument) {
	uint8_t retry;

	backupBuffer = allocateBackupBuffer();
	uint8_t backupPkt = 0;

	uint32_t pktLength;

	bool status;



//	for (int i = 0; i < MAX_PACKET_QUEUE_SIZE; i++) {
//		packetToSend = &packets[i];
//		packetToSend->header.systemID = LL_FLASH_GetUDN();
//		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
//				osWaitForever);
//	}

	for (int i = 0; i < MAX_PACKET_QUEUE_SIZE; i++) {
		packetToSend = &packets[i];

		packetToSend->header.system_uid = LL_FLASH_GetUDN();
//		packetToSend->header.systemID = LL_FLASH_GetUDN();
		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
				osWaitForever);
	}




	while (1) {
		/* the logic below prioritizes the latest packets avialable in the queue over
		 * any packets that were stored in backup FRAM
		 */

		/* check if a packet exists in the queue without any blocking */
//		if(osOK != osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, 0)){
//			/* if no packet available in queue, check if any packet exists in FRAM
//			 * from a previous power failure */
//			if(!getPacketFromFRAM(backupBuffer, packetToSend)){
//				/* if no packet exists in FRAM, just wait forever until a thread puts a packet in the queue */
//				osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever);
//			}
//		}

		if(isBluetoothConnected()){
			if(osOK == osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, 0)){

			}
			else if(!getPacketFromFRAM(backupBuffer, &backupPacket)){
				osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever);
			}
			else{
				backupPkt = 1;
				packetToSend = &backupPacket;
				osDelay(5);
			}
		}else{
			osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever);
		}

		if(packetToSend == NULL){
			continue;
		}

		retry = 0;

		packetToSend->header.epoch = getEpoch();
		packetToSend->header.ms_from_start = HAL_GetTick();


//		uint8_t buffer[128];
//		volatile size_t message_length;HAL_GetTick();
//		bool status;
//		size_t lenOfBuff;

		if(backupPkt != 1){
			//todo: pb refactor
//			packetToSend->header.systemID = LL_FLASH_GetUDN();
//			packetToSend->header.epoch = getEpoch();
		}
		if(isBluetoothConnected()){

			stream = pb_ostream_from_buffer(encoded_payload, MAX_PAYLOAD_SIZE);
			status = pb_encode(&stream, SENSOR_PACKET_FIELDS, packetToSend);
			pktLength = stream.bytes_written;

			while (PACKET_SEND_SUCCESS != sendProtobufPacket_BLE(encoded_payload,pktLength)) {
				if (retry >= MAX_BLE_RETRIES) {
					break;
				}
				retry++;
	//			osDelay(5);
			};
//		}else{
//			/* add packet to FRAM if its not IMU or Blink */
//			if( (packetToSend->header.packetType != IMU) &&
//					(packetToSend->header.packetType != BLINK)){
//				pushPacketToFRAM(backupBuffer, packetToSend);
//			}
		}

		// return memory back to pool
		if(backupPkt != 1){
			if(packetToSend != NULL){
				osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
						osWaitForever);
			}
		}
		packetToSend = NULL;
		backupPkt = 0;

//		osDelay(100);
//		osDelay(1);

//		osDelay(MAX_BLE_RETRIES - retry); // artificial delay to allow for the connected device to handle the latest sent packet
	}
}

static DTS_App_Context_t DataTransferServerContext;
static DTS_App_Context_t DataTransferSysConfigContext;

uint8_t sendProtobufPacket_BLE(uint8_t *packet, uint16_t size) {

	if ((size) > MAX_PAYLOAD_SIZE) {
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
	DataTransferServerContext.TxData.Length = size; //Att_Mtu_Exchanged-10;

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

//uint8_t sendPacket_BLE(sensor_packet_t *packet) {
//
//	if ((packet->header.payloadLength) > MAX_PAYLOAD_SIZE) {
//		return PACKET_LENGTH_EXCEEDED;
//	}
//
//	tBleStatus status = BLE_STATUS_INVALID_PARAMS;
////	uint8_t crc_result;
////
////	/* compute CRC */
////	crc_result = APP_BLE_ComputeCRC8((uint8_t*) Notification_Data_Buffer,
////			(DATA_NOTIFICATION_MAX_PACKET_SIZE - 1));
////	Notification_Data_Buffer[DATA_NOTIFICATION_MAX_PACKET_SIZE - 1] =
////			crc_result;
//
//	DataTransferServerContext.TxData.pPayload = (uint8_t*) packet;
//	DataTransferServerContext.TxData.Length = packet->header.payloadLength
//			+ sizeof(PacketHeader); //Att_Mtu_Exchanged-10;
//
////	if(packet->header.packetType==PPG_RED || packet->header.packetType==PPG_IR){
////		status = Generic_STM_UpdateChar(PPG_CHAR_UUID_DEF,
////	(uint8_t*) &DataTransferServerContext.TxData);
////	}
//
//	//COMMENTED BELOW ON 9/20/2022 BUT NEED TO FIX
//	status = DTS_STM_UpdateChar(DATA_TRANSFER_TX_CHAR_UUID,
//			(uint8_t*) &DataTransferServerContext.TxData);
//
//	if (status == BLE_STATUS_SUCCESS) {
//		return PACKET_SEND_SUCCESS;
//	} else {
//		return PACKET_UNDEFINED_ERR;
//	}
//}

uint8_t buffer[500];
uint8_t updateSystemConfig_BLE(system_state_t *packet) {

		tBleStatus status = BLE_STATUS_INVALID_PARAMS;


        /* Create a stream that will write to our buffer. */
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        /* Now we are ready to encode the message! */
        status = pb_encode(&stream, SYSTEM_STATE_FIELDS, packet);

		DataTransferSysConfigContext.TxData.pPayload = buffer;
		DataTransferSysConfigContext.TxData.Length = stream.bytes_written;

		status = DTS_STM_UpdateChar(DATA_TRANSFER_SENSOR_CONFIG_CHAR_UUID,
				(uint8_t*) &DataTransferSysConfigContext.TxData);

		if (status == BLE_STATUS_SUCCESS) {
			return PACKET_SEND_SUCCESS;
		} else {
			return PACKET_UNDEFINED_ERR;
		}
}

void updateRTC(uint32_t receivedTime){

	// (1) convert received UNIX time to time struct
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	RTC_FromEpoch(receivedTime, &time, &date);

	// (2) set time
	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
}

void updateRTC_MS(uint64_t receivedTime){

	receivedTime = receivedTime / 1000;

	// (1) convert received UNIX time to time struct
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	RTC_FromEpoch(receivedTime, &time, &date);

	// (2) set time
	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);

}

uint64_t getEpoch(void){

	// (1) convert received UNIX time to time struct
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	// (2) get time
	return RTC_ToEpochMS(&time, &date);
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t tm;
	uint32_t t1;
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t e;
	uint32_t m;
	int16_t  year  = 0;
	int16_t  month = 0;
	int16_t  dow   = 0;
	int16_t  mday  = 0;
	int16_t  hour  = 0;
	int16_t  min   = 0;
	int16_t  sec   = 0;
	uint64_t JD    = 0;
	uint64_t JDN   = 0;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	JD  = ((epoch + 43200) / (86400 >>1 )) + (2440587 << 1) + 1;
	JDN = JD >> 1;

    tm = epoch; t1 = tm / 60; sec  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 60; min  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 24; hour = tm - (t1 * 24);

    dow   = JDN % 7;
    a     = JDN + 32044;
    b     = ((4 * a) + 3) / 146097;
    c     = a - ((146097 * b) / 4);
    d     = ((4 * c) + 3) / 1461;
    e     = c - ((1461 * d) / 4);
    m     = ((5 * e) + 2) / 153;
    mday  = e - (((153 * m) + 2) / 5) + 1;
    month = m + 3 - (12 * (m / 10));
    year  = (100 * b) + d - 4800 + (m / 10);

    date->Year    = year - 2000;
    date->Month   = month;
    date->Date    = mday;
    date->WeekDay = dow;
    time->Hours   = hour;
    time->Minutes = min;
    time->Seconds = sec;
}

uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint32_t JDN;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	// Calculate some coefficients
	a = (14 - date->Month) / 12;
	y = (date->Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date->Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
    JDN  = date->Date;
    JDN += (153 * m + 2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN += -y / 100;
    JDN += y / 400;
    JDN  = JDN - 32045;
    JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
    JDN *= 86400;                     // Days to seconds
    JDN += time->Hours * 3600;    // ... and today seconds
    JDN += time->Minutes * 60;
    JDN += time->Seconds;

	return JDN;
}

uint64_t RTC_ToEpochMS(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint64_t JDN;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	// Calculate some coefficients
	a = (14 - date->Month) / 12;
	y = (date->Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date->Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
    JDN  = date->Date;
    JDN += (153 * m + 2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN += -y / 100;
    JDN += y / 400;
    JDN  = JDN - 32045;
    JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
    JDN *= 86400;                     // Days to seconds
    JDN += time->Hours * 3600;    // ... and today seconds
    JDN += time->Minutes * 60;
    JDN += time->Seconds;

    // ms conversion
    JDN = JDN * 1000;
    JDN += 1000 - ((time->SubSeconds*1000) /  time->SecondFraction);
	return JDN;
}
