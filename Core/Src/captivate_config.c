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
			specTaskHandle = osThreadNew(Spec_Task, &sysState.config.color, &specTask_attributes);
		}
	}else{
		osThreadFlagsSet(specTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlBME(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(bmeTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			bmeTaskHandle = osThreadNew(BME_Task, &sysState.config.bme, &bmeTask_attributes);
		}
	}else{
		osThreadFlagsSet(bmeTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlIMU(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(imuTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			imuTaskHandle = osThreadNew(IMU_Task,&sysState.config.imu, &imuTask_attributes);
		}
	}else{
		osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlThermopile(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(sgpTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			thermopileTaskHandle = osThreadNew(Thermopile_Task, &sysState.config.thermopile, &thermopileTask_attributes);
		}
	}else{
		osThreadFlagsSet(thermopileTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlLux(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(luxTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			luxTaskHandle = osThreadNew(LuxTask, &sysState.config.lux, &luxTask_attributes);
		}
	}else{
		osThreadFlagsSet(luxTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlMic(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(micTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			micTaskHandle = osThreadNew(Mic_Task, &sysState.config.mic, &micTask_attributes);
		}
	}else{
		osThreadFlagsSet(micTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlSHT(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(shtTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			shtTaskHandle = osThreadNew(ShtTask, &sysState.config.humidity, &shtTask_attributes);
		}
	}else{
		osThreadFlagsSet(shtTaskHandle, TERMINATE_THREAD_BIT);
	}
}
void controlSGP(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			sgpTaskHandle = osThreadNew(SgpTask, &sysState.config.sgp, &sgpTask_attributes);
		}
	}else{
		osThreadFlagsSet(sgpTaskHandle, TERMINATE_THREAD_BIT);

	}
}

void controlBlink(bool state){

	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			blinkTaskHandle = osThreadNew(BlinkTask, &sysState.config.blink, &blinkTask_attributes);
		}
	}else{
		osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);

	}
}

blink_sensor_config_t blinkConfigNoWindow;
void controlBlinkNoWindow(bool state){

	osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);

	memcpy(&blinkConfigNoWindow,&sysState.config.blink,sizeof(blink_sensor_config_t));
	blinkConfigNoWindow.enable_windowing = 0;

	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
			blinkTaskHandle = osThreadNew(BlinkTask, &blinkConfigNoWindow, &blinkTask_attributes);
		}
	}else{
		osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);

	}
}

//blink_sensor_config_t blinkConfigNoWindow;
//void controlBlinkNoWindow(bool state){
//
//	osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
//
//	memcpy(&blinkConfigNoWindow,&sysState.config.blink,sizeof(blink_sensor_config_t));
//	blinkConfigNoWindow.enable_windowing = 0;
//
//	if(state){
//		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
//		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
//			blinkTaskHandle = osThreadNew(BlinkTask, &blinkConfigNoWindow, &blinkTask_attributes);
//		}
//	}else{
//		osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
//
//	}
//}
//
//imu_sensor_config_t imuConfigNoWindow;
//void controlIMUNoWindow(bool state){
//	osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
//
//	memcpy(&imuConfigNoWindow,&sysState.config.imu,sizeof(imu_sensor_config_t));
//	imuConfigNoWindow.enable_windowing = 0;
//
//	if(state){
//		osThreadState_t threadState = osThreadGetState(imuTaskHandle);
//		if( (threadState == osThreadTerminated) || (threadState == osThreadError)){
//			imuTaskHandle = osThreadNew(IMU_Task,&imuConfigNoWindow, &imuTask_attributes);
//		}
//	}else{
//		osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
//	}
//}

void controlAllSensors(bool state){
	controlSpectrometer(state);
	controlBME(state);
	controlIMU(state);
	controlThermopile(state);
	controlLux(state);
	controlMic(state);
	controlSHT(state);
	controlSGP(state);
	controlBlink(state);
}

void ingestSensorConfig(system_state_t *config){
//	if(config->systemRunState == 0){
//		controlAllSensors(0);
//	}else{
//		controlSpectrometer(config->colorSensor.enable);
//		controlBME(config->gasSensor.enable);
//		controlIMU(config->inertialSensor.enable);
//		controlThermopile(config->thermopileSensor.enable);
//		controlLux(config->luxSensor.enable);
//		controlMic(config->micSensor.enable);
//		controlSHT(config->humiditySensor.enable);
//		controlSGP(config->gasSensor.enable);
//		controlBlink(config->blinkSensor.enable);;
//	}


	if(config->control.enable_all){
		controlSpectrometer(1);
		controlBME(1);
		controlIMU(1);
		controlThermopile(1);
		controlLux(1);
		controlMic(1);
		controlSHT(1);
		controlSGP(1);
		controlBlink(1);
	}else{
		controlSpectrometer(config->control.spectrometer);
		controlBME(config->control.bme688);
		controlIMU(config->control.imu);
		controlThermopile(config->control.thermopiles);
		controlLux(config->control.lux);
		controlMic(config->control.mic);
		controlSHT(config->control.sht);
		controlSGP(config->control.sgp);
		controlBlink(config->control.blink);

	}


}

//void controlSensors(uint8_t* data, uint16_t numPackets){
//
//
//
//	for(int i = 0; i<numPackets; i++){
//
//		if( ((sensor_state.sensorSystems >> data[i*2]) & 0x1) == 0x1){
//			if(data[i*2+1] == 0){
//				sensor_state.sensorSystems = sensor_state.sensorSystems & !(0x00000001 << data[i*2]);
//				extMemWriteData(SENSOR_STATE_ADDR, (uint8_t*) &sensor_state, SENSOR_STATE_SIZE);
//			}
//		}else{
//			if(data[i*2+1] == 1){
//				sensor_state.sensorSystems = sensor_state.sensorSystems | (0x00000001 << data[i*2]);
//				extMemWriteData(SENSOR_STATE_ADDR, (uint8_t*) &sensor_state, SENSOR_STATE_SIZE);
//			}
//		}
//
//		switch((PacketTypes) data[i*2]){
//			case ALL_SENSORS:
//				controlAllSensors(data[i*2+1]);
//				sensorConfig.thermopileSensor.enable = data[i*2+1];
//				sensorConfig.blinkSensor.enable = data[i*2+1];
//				sensorConfig.inertialSensor.enable = data[i*2+1];
//				sensorConfig.gasSensor.enable = data[i*2+1];
//				sensorConfig.luxSensor.enable = data[i*2+1];
//				sensorConfig.colorSensor.enable = data[i*2+1];
//				sensorConfig.micSensor.enable = data[i*2+1];
//				break;
//			case SPECTROMETER:
//				controlSpectrometer(data[i*2+1]);
//				sensorConfig.colorSensor.enable = data[i*2+1];
//				break;
//			case BME:
//				controlBME(data[i*2+1]);
//				sensorConfig.gasSensor.enable = data[i*2+1];
//				break;
//			case CO2:
//
//				break;
//			case IMU:
//				controlIMU(data[i*2+1]);
//				sensorConfig.inertialSensor.enable = data[i*2+1];
//				break;
//			case THERMOPILE:
//				controlThermopile(data[i*2+1]);
//				sensorConfig.thermopileSensor.enable = data[i*2+1];
//				break;
//			case LUX:
//				controlLux(data[i*2+1]);
//				sensorConfig.luxSensor.enable = data[i*2+1];
//				break;
//			case LIDAR:
//
//				break;
//			case MIC:
//				controlMic(data[i*2+1]);
//				sensorConfig.micSensor.enable = data[i*2+1];
//				break;
//			case SHT:
//				controlSHT(data[i*2+1]);
//				sensorConfig.humiditySensor.enable = data[i*2+1];
//				break;
//			case SGP:
//				controlSGP(data[i*2+1]);
//				sensorConfig.gasSensor.enable = data[i*2+1];
//				break;
//			case BLINK:
//				controlBlink(data[i*2+1]);
//				sensorConfig.blinkSensor.enable = data[i*2+1];
//				break;
//			default:
//				break;
//		}
//		extMemWriteData(START_ADDR + 1, (uint8_t*) &sensorConfig,
//						sizeof(struct SensorConfig));
//	}
//}



