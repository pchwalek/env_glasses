/*
 * fram.cpp
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */

#include "fram.h"

#define SPI_HAN &hspi1

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
}


bool extMemInit(){
	extMemWriteEnableLatch(true);
}

bool framWriteEnable(bool state){
	uint8_t dataTX[2] = {0};
	uint8_t dataRX[2] = {0};

	dataTX[0] = WRSR;
	HAL_SPI_TransmitReceive(SPI_HAN, dataTX, dataRX, 2, 10);

	dataTX[1] = dataRX[1] | WRSR_WriteEnable;
	HAL_SPI_Transmit(SPI_HAN, dataTX, 2, 10);
}

bool extMemGetData(uint32_t addr, uint8_t* data, uint16_t size){
	extMemChipSelectPin(true);
	uint8_t opCode = READ;
	uint8_t address[3];
	address[0] = addr >> 16;
	address[1] = addr >> 8;
	address[2] = addr;
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, &opCode, 1, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, address, 3, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Receive(SPI_HAN, data, size, 10)){
		return false;
	};
	extMemChipSelectPin(false);
	return true;
}

bool extMemWriteData(uint32_t addr, uint8_t* data, uint16_t size){
	HAL_StatusTypeDef state;

	extMemChipSelectPin(true);
	uint8_t opCode = WRITE;
	uint8_t address[3];
	address[0] = addr >> 16;
	address[1] = addr >> 8;
	address[2] = addr;
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, &opCode, 1, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, address, 3, 10)){
		return false;
	};
	if(HAL_OK != HAL_SPI_Transmit(SPI_HAN, data, size, 10)){
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
