/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "../../Sensirion_Core/src/SensirionI2CCommunication.h"

#include <stdint.h>
#include <stdlib.h>

//#include "Arduino.h"
#include "../../Sensirion_Core/src/SensirionCrc.h"
#include "../../Sensirion_Core/src/SensirionErrors.h"
#include "../../Sensirion_Core/src/SensirionI2CRxFrame.h"
#include "../../Sensirion_Core/src/SensirionI2CTxFrame.h"

static void clearRxBuffer(I2C_HandleTypeDef *i2c_handle) {
//    while (i2cBus.available()) {
//        (void)i2cBus.read();
//    }
}

uint16_t SensirionI2CCommunication::sendFrame(uint8_t address,
                                              SensirionI2CTxFrame& frame,
											  I2C_HandleTypeDef *i2c_handle) {
//    i2cBus.beginTransmission(address);
//    size_t writtenBytes = i2cBus.write(frame._buffer, frame._index);
//    uint8_t i2c_error = i2cBus.endTransmission();

    if(HAL_OK != HAL_I2C_Master_Transmit(i2c_handle, address << 1, frame._buffer, frame._index, 10)){
    	return WriteError | I2cOtherError;
    }else{
    	return NoError;
    }

}

uint16_t SensirionI2CCommunication::receiveFrame(uint8_t address,
                                                 size_t numBytes,
                                                 SensirionI2CRxFrame& frame,
												 I2C_HandleTypeDef *i2c_handle,
                                                 CrcPolynomial poly) {
    size_t readAmount;
    size_t i = 0;

#ifdef I2C_BUFFER_LENGTH
    const uint8_t sizeBuffer =
        (static_cast<uint8_t>(I2C_BUFFER_LENGTH) / static_cast<uint8_t>(3)) * 3;
#elif defined(BUFFER_LENGTH)
    const uint8_t sizeBuffer =
        (static_cast<uint8_t>(BUFFER_LENGTH) / static_cast<uint8_t>(3)) * 3;
#else
    const uint8_t sizeBuffer = 30;
#endif

    if (numBytes % 3) {
        return ReadError | WrongNumberBytesError;
    }
    if ((numBytes / 3) * 2 > frame._bufferSize) {
        return ReadError | BufferSizeError;
    }
    if (numBytes > sizeBuffer) {
        return ReadError | InternalBufferSizeError;
    }

    uint8_t rxData[numBytes];
    if(HAL_OK != HAL_I2C_Master_Receive(i2c_handle, address << 1, rxData, static_cast<uint8_t>(numBytes), 100)){
      	return WriteError | I2cOtherError;
      }
    readAmount = numBytes;
//    else{
//      	return NoError;
//      }

    uint16_t j = 0;
    do {
        frame._buffer[i++] = rxData[j++];
        frame._buffer[i++] = rxData[j++];
        uint8_t actualCRC = rxData[j++];;
        uint8_t expectedCRC = generateCRC(&frame._buffer[i - 2], 2, poly);
        if (actualCRC != expectedCRC) {
//            clearRxBuffer(i2cBus);
            return ReadError | CRCError;
        }
        readAmount -= 3;
    } while (readAmount > 0);
    frame._numBytes = i;
    return NoError;
}
