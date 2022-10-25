/*
 * fram.cpp
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#include "fram.h"


#define SPI_HAN &hspi1

uint8_t header[4];

bool extMemWriteEnableLatch(bool state){
	uint8_t dataTX;
	extMemChipSelectPin(true);
	if(state){
		dataTX = WREN;
		HAL_SPI_Transmit(SPI_HAN, &dataTX, 1, 10);
	}else{
		dataTX = WRDI;
		HAL_SPI_Transmit(SPI_HAN, &dataTX, 1, 10);
	}
	extMemChipSelectPin(false);
	return true;
}


bool extMemInit(){
	extMemWriteEnableLatch(true);
	return true;
}

bool framWriteEnable(bool state){
	uint8_t dataTX[2] = {0};
	uint8_t dataRX[2] = {0};

	dataTX[0] = WRSR;
	HAL_SPI_TransmitReceive(SPI_HAN, dataTX, dataRX, 2, 10);

	dataTX[1] = dataRX[1] | WRSR_WriteEnable;
	HAL_SPI_Transmit(SPI_HAN, dataTX, 2, 10);
	return true;
}

bool extMemGetData(uint32_t addr, uint8_t* data, uint16_t size){

	extMemChipSelectPin(true);

	header[0] = READ;
	header[1] = (addr >> 16) & 0xFF;
	header[2] = (addr >> 8) & 0xFF;
	header[3] = (addr) & 0xFF;

//	uint8_t opCode = READ;
//	uint8_t address[3];
//	address[0] = (addr >> 16) & 0xFF;
//	address[1] = (addr >> 8) & 0xFF;
//	address[2] = (addr) & 0xFF;
//	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, &opCode, 1, 10)){
//		return false;
//	};
//	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, address, 3, 10)){
//		return false;
//	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, header, 4, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Receive(SPI_HAN, data, size, 100)){
		return false;
	};
	extMemChipSelectPin(false);

	return true;
}

bool extMemWriteData(uint32_t addr, uint8_t* data, uint16_t size){
	HAL_StatusTypeDef state;

	extMemWriteEnableLatch(true);

	extMemChipSelectPin(true);
//	uint8_t opCode = WRITE;
//	uint8_t address[3];
//	address[0] = (addr >> 16) & 0xFF;
//	address[1] = (addr >> 8) & 0xFF;
//	address[2] = (addr) & 0xFF;

	header[0] = WRITE;
	header[1] = (addr >> 16) & 0xFF;
	header[2] = (addr >> 8) & 0xFF;
	header[3] = (addr) & 0xFF;

//	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, &opCode, 1, 10)){
//		return false;
//	};
//	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, address, 3, 10)){
//		return false;
//	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, header, 4, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, data, size, 100)){
		return false;
	};
	extMemChipSelectPin(false);

	return true;
}

bool extMemChipSelectPin(bool state){
	if(state){
		HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);
	}
}

bool extMemWriteProtectPin(bool state){
	if(state){
		HAL_GPIO_WritePin(MEM_WP_GPIO_Port, MEM_WP_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(MEM_WP_GPIO_Port, MEM_WP_Pin, GPIO_PIN_RESET);
	}
}
