/*
 * fram.h
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

// library for CY15B108QN-40LPXI

#ifndef INC_FRAM_H_
#define INC_FRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "app_conf.h"
#include "cmsis_os2.h"
#include "captivate_config.h"
#include "bsec_datatypes.h"
#include "spi.h"
#include "circular_buffer.h"
#include "packet.h"

#define MAX_ADDR	0x0FFFFF

/* Opcodes for CY15B108QN */
#define WREN		0x06
#define WRDI		0x04
#define RDSR		0x05
#define WRSR		0x01
#define WRITE		0x02
#define READ		0x03
#define FAST_READ	0x0B
#define SSWR		0x42
#define SSRD		0x4B
#define RDID		0x9F
#define RUID		0x4C
#define WRSN		0xC2
#define RDSN		0xC3
#define DPD			0xBA
#define HBN			0xB9

/* Status Register */
#define WRSR_WriteEnable	0x01 << 1
#define WRSR_WruteProtectEn	0x01 << 7

#define BUFF_PACKET_SIZE			512
#define BACKUP_BUFF_SIZE	((uint32_t) BACKUP_SIZE) / BUFF_PACKET_SIZE

#define MAX_MEMORY_ADDR			0x0FFFFF //1,048,575 (8 Mbit)

#define START_ADDR				0x0
#define RESERVE_SIZE			200

#define SENSOR_STATE_ADDR		START_ADDR + RESERVE_SIZE
#define SENSOR_STATE_SIZE		sizeof(sensorState)
#define BME_CONFIG_ADDR			SENSOR_STATE_ADDR + SENSOR_STATE_SIZE
#define BME_CONFIG_SIZE			BSEC_MAX_PROPERTY_BLOB_SIZE
#define BME_STATE_ADDR			BME_CONFIG_ADDR + BME_CONFIG_SIZE
#define BME_STATE_SIZE			BSEC_MAX_STATE_BLOB_SIZE
#define BME_FIRST_RUN_ADDR		BME_STATE_ADDR + BSEC_MAX_STATE_BLOB_SIZE
#define BME_FIRST_RUN_SIZE		sizeof(uint8_t)

#define BACKUP_START_ADDR		BME_FIRST_RUN_ADDR + BME_FIRST_RUN_SIZE
#define BACKUP_SIZE				MAX_MEMORY_ADDR - BACKUP_START_ADDR

bool extMemInit();
bool extMemGetData(uint32_t addr, uint8_t* data, uint16_t size);
bool extMemWriteData(uint32_t addr, uint8_t* data, uint16_t size);
void extMemChipSelectPin(bool state);
bool extMemWriteProtectPin(bool state);

CircularBuffer* allocateBackupBuffer(void);
uint8_t getPacketFromFRAM(CircularBuffer* backupBuffer, SensorPacket* packet);
uint8_t pushPacketToFRAM(CircularBuffer* backupBuffer, SensorPacket* packet);

#ifdef __cplusplus
}
#endif

#endif /* INC_PACKET_H_ */
