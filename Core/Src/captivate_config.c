/*
 * captivate_config.c
 *
 *  Created on: Dec 21, 2022
 *      Author: chwalek
 */

#include "packet.h"
#include "captivate_config.h"
#include "lp5523.h"
#include "thermopile.h"
#include "spectrometer.h"
#include "lux.h"
#include "bme.h"
#include "imu.h"
#include "blink.h"
#include "packet.h"
#include "sht.h"
#include "sgp.h"
#include "mic.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "stdbool.h"
#include "fram.h"

#define ALL_SENSORS	1

void controlSpectrometer(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(specTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			specTaskHandle = osThreadNew(Spec_Task, NULL, &specTask_attributes);
		}
	}else{
		osThreadFlagsSet(specTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlBME(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(bmeTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			bmeTaskHandle = osThreadNew(BME_Task, NULL, &bmeTask_attributes);
		}
	}else{
		osThreadFlagsSet(bmeTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlIMU(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(imuTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);
		}
	}else{
		osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlThermopile(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(sgpTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			thermopileTaskHandle = osThreadNew(Thermopile_Task, NULL, &thermopileTask_attributes);
		}
	}else{
		osThreadFlagsSet(thermopileTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlLux(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(luxTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			luxTaskHandle = osThreadNew(LuxTask, NULL, &luxTask_attributes);
		}
	}else{
		osThreadFlagsSet(luxTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlMic(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(micTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			micTaskHandle = osThreadNew(Mic_Task, NULL, &micTask_attributes);
		}
	}else{
		osThreadFlagsSet(micTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlSHT(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(shtTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			shtTaskHandle = osThreadNew(ShtTask, NULL, &shtTask_attributes);
		}
	}else{
		osThreadFlagsSet(shtTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlSGP(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			sgpTaskHandle = osThreadNew(SgpTask, NULL, &sgpTask_attributes);
		}
	}else{
		osThreadFlagsSet(sgpTaskHandle, TERMINATE_THREAD_BIT);

	}
}

void controlBlink(bool state){

	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			blinkTaskHandle = osThreadNew(BlinkTask, NULL, &blinkTask_attributes);
		}
	}else{
		osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);

	}
}

void controlAllSensors(bool state){
	controlSpectrometer(state);
	controlBME(state);
	controlIMU(state);
	controlThermopile(state);
	controlLux(state);
	controlMic(state);
	controlSHT(state);
	controlSGP(state);
	controlBlink(state);;
}


void controlSensors(uint8_t* data, uint16_t numPackets){



	for(int i = 0; i<numPackets; i++){

		if( ((sensor_state.sensorSystems >> data[i*2]) & 0x1) == 0x1){
			if(data[i*2+1] == 0){
				sensor_state.sensorSystems = sensor_state.sensorSystems & !(0x00000001 << data[i*2]);
				extMemWriteData(SENSOR_STATE_ADDR, (uint8_t*) &sensor_state, SENSOR_STATE_SIZE);
			}
		}else{
			if(data[i*2+1] == 1){
				sensor_state.sensorSystems = sensor_state.sensorSystems | (0x00000001 << data[i*2]);
				extMemWriteData(SENSOR_STATE_ADDR, (uint8_t*) &sensor_state, SENSOR_STATE_SIZE);
			}
		}

		switch((PacketTypes) data[i*2]){
			case ALL_SENSORS:
				controlAllSensors(data[i*2+1]);
				break;
			case SPECTROMETER:
				controlSpectrometer(data[i*2+1]);
				break;
			case BME:
				controlBME(data[i*2+1]);
				break;
			case CO2:

				break;
			case IMU:
				controlIMU(data[i*2+1]);
				break;
			case THERMOPILE:
				controlThermopile(data[i*2+1]);
				break;
			case LUX:
				controlLux(data[i*2+1]);
				break;
			case LIDAR:

				break;
			case MIC:
				controlMic(data[i*2+1]);
				break;
			case SHT:
				controlSHT(data[i*2+1]);
				break;
			case SGP:
				controlSGP(data[i*2+1]);
				break;
			case BLINK:
				controlBlink(data[i*2+1]);
				break;
			default:
				break;
		}
	}
}



