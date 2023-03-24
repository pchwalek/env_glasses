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

#include "time.h"

#define JULIAN_DATE_BASE     2440588   // Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)
//static const uint16_t week_day[] = { 0x4263, 0xA8BD, 0x42BF, 0x4370, 0xABBF, 0xA8BF, 0x43B2 };

// https://github.com/LonelyWolf/stm32/blob/master/stm32l-dosfs/RTC.c
void RTC_FromEpoch(time_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint64_t RTC_ToEpochMS(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

static sensor_packet_t packets[MAX_PACKET_QUEUE_SIZE];

//static uint8_t packets[MAX_PACKET_QUEUE_SIZE][600];

sensor_packet_t *packetPtr[MAX_PACKET_QUEUE_SIZE];

static uint32_t backup_buff_addresses[BACKUP_BUFF_SIZE];
static uint32_t buffer_index = 0;

sensor_packet_t* grabPacket(void) {
	sensor_packet_t *packet;
	// grab available memory for packet creation
	if (osOK != osMessageQueueGet(packetAvail_QueueHandle, &packet, 0U, 0)) {
		return NULL;
	}
	return packet;
}

void queueUpPacket(sensor_packet_t *packet, uint32_t timeout) {
	// put into queue
	osMessageQueuePut(packet_QueueHandle, &packet, 0U, timeout);
}

sensor_packet_t *packetToSend;
sensor_packet_t backupPacket;
//CircularBuffer* backupBuffer;
FRAM_Packets FRAM_packet, FRAM_old_sample;

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

		case SENSOR_PACKET_TYPES_MIC_LEVEL :
			packetPtr->which_payload = SENSOR_PACKET_MIC_LEVEL_PACKET_TAG;
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

//	backupBuffer = allocateBackupBuffer();
	uint8_t backupPkt = 0;

	uint32_t pktLength;

	bool status;

	uint32_t start_addr = BACKUP_START_ADDR;
	for (uint32_t i=0; i<BACKUP_BUFF_SIZE; i++) {
		   backup_buff_addresses[i] = start_addr;
		   start_addr += BUFF_PACKET_SIZE;
	}

//	for (int i = 0; i < MAX_PACKET_QUEUE_SIZE; i++) {
//		packetToSend = &packets[i];
//		packetToSend->header.systemID = LL_FLASH_GetUDN();
//		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
//				osWaitForever);
//	}

	for (int i = 0; i < MAX_PACKET_QUEUE_SIZE; i++) {
		packetToSend = &packets[i];

		memset(packetToSend, 0, sizeof(sensor_packet_t));

		packetToSend->header.system_uid = LL_FLASH_GetUDN();
//		packetToSend->header.systemID = LL_FLASH_GetUDN();
		osMessageQueuePut(packetAvail_QueueHandle, &packetToSend, 0U,
				osWaitForever);
	}


//	volatile uint64_t testVar = getEpoch();

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
			/* first check if any real-time packets are ready to be sent (with no timeout) */
			if(osOK == osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, 0)){
				packetToSend->header.epoch = getEpoch();
				packetToSend->header.ms_from_start = HAL_GetTick();
			}
			/* else check if any old packets exist in FRAM */
//			else if(!getPacketFromFRAM(backupBuffer, &backupPacket)){
			else if(osMessageQueueGetCount(FRAM_QueueHandle) > 0){
				if(osOK == osMessageQueueGet (FRAM_QueueHandle, &FRAM_packet, 0, 0)){
					extMemGetData(FRAM_packet.memory_addr, (uint8_t*) &encoded_payload, FRAM_packet.size);
					pktLength = FRAM_packet.size;
					backupPkt = 1;
				}
			}
			/* else just wait for a real-time packet to come in*/
			else{
				if(osOK == osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever)){
					packetToSend->header.epoch = getEpoch();
					packetToSend->header.ms_from_start = HAL_GetTick();
				}
			}
		}else{
			if(osOK == osMessageQueueGet(packet_QueueHandle, &packetToSend, 0U, osWaitForever)){
				packetToSend->header.epoch = getEpoch();
				packetToSend->header.ms_from_start = HAL_GetTick();
			}else{
				continue;
			}
		}

//		if((getEpoch() - testVar) > 60000){
//			testVar = getEpoch();
//		}

//		if(packetToSend == NULL){
//			continue;
//		}

		retry = 0;

		if(isBluetoothConnected()){
			if(backupPkt != 1){
				stream = pb_ostream_from_buffer(encoded_payload, MAX_PAYLOAD_SIZE);
				status = pb_encode(&stream, SENSOR_PACKET_FIELDS, packetToSend);
				pktLength = stream.bytes_written;
			}else{
				backupPkt = 0;
			}

			while (PACKET_SEND_SUCCESS != sendProtobufPacket_BLE(encoded_payload,pktLength)) {
				if (retry >= MAX_BLE_RETRIES) {
					break;
				}
				retry++;
	//			osDelay(5);
			};

			/* artificial delay in case the data rate of client device is bottlenecked */
//			osDelay(25);
		}else{
			/* add packet to FRAM if its not IMU or Blink */
			if( packetToSend->which_payload != SENSOR_PACKET_IMU_PACKET_TAG ){

				stream = pb_ostream_from_buffer(encoded_payload, MAX_PAYLOAD_SIZE);
				status = pb_encode(&stream, SENSOR_PACKET_FIELDS, packetToSend);
				pktLength = stream.bytes_written;

				FRAM_packet.memory_addr = backup_buff_addresses[buffer_index];
				FRAM_packet.size = stream.bytes_written;

				if(extMemWriteData(FRAM_packet.memory_addr, encoded_payload, FRAM_packet.size)){

					// if queue is full, pop off oldest entry
					if(osMessageQueueGetCount(FRAM_QueueHandle) == BACKUP_BUFF_SIZE){
						osMessageQueueGet (FRAM_QueueHandle, &FRAM_old_sample, 0, 0);
					}

					if( osOK == osMessageQueuePut (FRAM_QueueHandle, &FRAM_packet, 0, 0)){
						buffer_index += 1;
						buffer_index %= BACKUP_BUFF_SIZE;
					}
				}

			}

		}

		// return memory back to pool
		if(backupPkt != 1){
			if(packetToSend != NULL){
				memset(&packetToSend->payload, 0, sizeof(packetToSend->payload));

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

//void updateRTC(uint32_t receivedTime){
//
//	// (1) convert received UNIX time to time struct
//	RTC_TimeTypeDef time;
//	RTC_DateTypeDef date;
//	RTC_FromEpoch(receivedTime, &time, &date);
//
//	// (2) set time
//	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
//	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
//}

void updateRTC_MS(uint64_t receivedTime){

	receivedTime = receivedTime / 1000;

	// (1) convert received UNIX time to time struct
	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};
	RTC_FromEpoch(receivedTime, &time, &date);

	// (2) set time
	taskENTER_CRITICAL();
	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
	taskEXIT_CRITICAL();

}

uint64_t getEpoch(void){

	// (1) convert received UNIX time to time struct
	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};

	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	// (2) get time
	return RTC_ToEpochMS(&time, &date);
}

time_t timestamp;
struct tm currTime;
struct tm time_tm;
//https://community.st.com/s/question/0D50X00009XkgJESAZ/unix-epoch-timestamp-to-rtc
// Convert epoch time to Date/Time structures
void RTC_FromEpoch(time_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {


	 time_tm = *(localtime(&epoch));

	 time->TimeFormat = 0;
	 time->SubSeconds = 0;
	 time->SecondFraction = 0;
	 time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	 time->StoreOperation = RTC_STOREOPERATION_SET;

	 time->Hours = (uint8_t)time_tm.tm_hour;
	 time->Minutes = (uint8_t)time_tm.tm_min;
	 time->Seconds = (uint8_t)time_tm.tm_sec;
//	 HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);

	 if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; } // the chip goes mon tue wed thu fri sat sun
	 date->WeekDay = (uint8_t)time_tm.tm_wday;
	 date->Month = (uint8_t)time_tm.tm_mon+1; //momth 1- This is why date math is frustrating.
	 date->Date = (uint8_t)time_tm.tm_mday;
	 date->Year = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

	 /*
	 * update the RTC
	 */


//	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2); // lock it in with the backup registers
}

uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {

	currTime.tm_year = date->Year + 100;  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = date->Date;
	currTime.tm_mon  = date->Month - 1;

	currTime.tm_hour = time->Hours;
	currTime.tm_min  = time->Minutes;
	currTime.tm_sec  = time->Seconds;

	return mktime(&currTime);
}

uint64_t RTC_ToEpochMS(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {

	currTime.tm_year = date->Year + 100;  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = date->Date;
	currTime.tm_mon  = date->Month - 1;

	currTime.tm_hour = time->Hours;
	currTime.tm_min  = time->Minutes;
	currTime.tm_sec  = time->Seconds;

	uint64_t timestamp_ms = mktime(&currTime);
	return (timestamp_ms * 1000) + 1000 - ((time->SubSeconds*1000) /  time->SecondFraction);
}
