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

#include "dts.h"

#include "i2c.h"

#define ALL_SENSORS	1
#define CONTROL_DELAY 500

void syncTimerActive(bool state, uint32_t period);
static void syncTimerTask(void);

void controlSpectrometer(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(specTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			specTaskHandle = osThreadNew(Spec_Task, &sysState.config.color, &specTask_attributes);
		}
	}else{
		osThreadFlagsSet(specTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlBME(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(bmeTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			bmeTaskHandle = osThreadNew(BME_Task, &sysState.config.bme, &bmeTask_attributes);
		}
	}else{
		osThreadFlagsSet(bmeTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlIMU(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(imuTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			imuTaskHandle = osThreadNew(IMU_Task,&sysState.config.imu, &imuTask_attributes);
		}
	}else{
		osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}

imu_sensor_config_t imuConfigNoWindow;
void controlIMUNoWindow(bool state){
	//	osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
	//
	//	osDelay(100);

	if(state){
		memcpy(&imuConfigNoWindow,&sysState.config.imu,sizeof(imu_sensor_config_t));
		imuConfigNoWindow.enable_windowing = 0;
		imuConfigNoWindow.enable_windowing_sync = 0;

		volatile osThreadState_t threadState = osThreadGetState(imuTaskHandle);

		while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
			osDelay(100);
			osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);
			threadState = osThreadGetState(imuTaskHandle);
		}

		imuTaskHandle = osThreadNew(IMU_Task,&imuConfigNoWindow, &imuTask_attributes);
	}
}
void controlThermopile(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(thermopileTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			thermopileTaskHandle = osThreadNew(Thermopile_Task, &sysState.config.thermopile, &thermopileTask_attributes);
		}
	}else{
		osThreadFlagsSet(thermopileTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlLux(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(luxTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			luxTaskHandle = osThreadNew(LuxTask, &sysState.config.lux, &luxTask_attributes);
		}
	}else{
		osThreadFlagsSet(luxTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlMic(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(micTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			micTaskHandle = osThreadNew(Mic_Task, &sysState.config.mic, &micTask_attributes);
		}
	}else{
		osThreadFlagsSet(micTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlSHT(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(shtTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			shtTaskHandle = osThreadNew(ShtTask, &sysState.config.humidity, &shtTask_attributes);
		}
	}else{
		osThreadFlagsSet(shtTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}
void controlSGP(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(sgpTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			sgpTaskHandle = osThreadNew(SgpTask, &sysState.config.sgp, &sgpTask_attributes);
		}
	}else{
		osThreadFlagsSet(sgpTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}

void controlBlink(bool state){
	if(state){
		osThreadState_t threadState = osThreadGetState(blinkTaskHandle);
		if( (threadState == osThreadTerminated) || (threadState == osThreadError) || (threadState == osThreadReady)){
			blinkTaskHandle = osThreadNew(BlinkTask, &sysState.config.blink, &blinkTask_attributes);
		}
	}else{
		osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
	}
//	osDelay(CONTROL_DELAY);
}

void waitUntilSensorsAreTerminated(){
	osThreadState_t threadState;
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(thermopileTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(blinkTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(sgpTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(shtTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(luxTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(micTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(imuTaskHandle);
		osDelay(10);
	}
	while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
		threadState = osThreadGetState(bmeTaskHandle);
		osDelay(10);
	}
}

blink_sensor_config_t blinkConfigNoWindow;
void controlBlinkNoWindow(bool state){

	//	osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
	//
	//	osDelay(100);



	if(state){
		memcpy(&blinkConfigNoWindow,&sysState.config.blink,sizeof(blink_sensor_config_t));
		blinkConfigNoWindow.enable_windowing = 0;
		blinkConfigNoWindow.enable_windowing_sync = 0;

		volatile osThreadState_t threadState = osThreadGetState(blinkTaskHandle);

		while( (threadState != osThreadTerminated) && (threadState != osThreadError) && (threadState != osThreadReady)){
			osDelay(100);
			osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
			threadState = osThreadGetState(blinkTaskHandle);
		}

		blinkTaskHandle = osThreadNew(BlinkTask,&blinkConfigNoWindow, &blinkTask_attributes);
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

	if(sysState.control.synchronize_windows){
		syncTimerActive(state, sysState.control.window_period_ms);
	}else{
		syncTimerActive(false, 0);
	}
}

void syncTimerActive(bool state, uint32_t period){
	uint32_t timerRunning = osTimerIsRunning(sensorSyncTimer_id);

	if(state && !timerRunning){
		sensorSyncTimer_id = osTimerNew(syncTimerTask, osTimerPeriodic, NULL, NULL);
		osTimerStart(sensorSyncTimer_id, period);
	}else if (!state && timerRunning){
		osTimerDelete(sensorSyncTimer_id);
	}
}

static void syncTimerTask(void){
	BlinkSyncTrigger();
	IMUSyncTrigger();
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
		controlThermopile(1);
		controlLux(1);
		controlSHT(1);
		controlSGP(1);
		controlBlink(1);
		controlMic(1);
		controlIMU(1);

		if(config->control.synchronize_windows){
			syncTimerActive(1, config->control.window_period_ms);
		}else{
			syncTimerActive(false, 0);
		}
	}else{
		controlThermopile(config->control.thermopiles);
		controlSpectrometer(config->control.spectrometer);
		controlBME(config->control.bme688);
		controlLux(config->control.lux);
		controlSHT(config->control.sht);
		controlSGP(config->control.sgp);
		controlBlink(config->control.blink);
		controlMic(config->control.mic);
		controlIMU(config->control.imu);

		if(config->control.synchronize_windows){
			syncTimerActive(config->control.imu | config->control.blink, config->control.window_period_ms);
		}else{
			syncTimerActive(false, 0);
		}
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



void BlinkCalTask(void *argument){
	//	union BlueGreenTransition blueGreenTran;

	blink_calibration_t blinkCalRX;

//	uint32_t timeTracker = 0;

	memcpy(&blinkCalRX,argument,sizeof(blink_calibration_t));


//	uint8_t step_duration_seconds;
//	uint8_t green_hold_length_seconds;
//	uint8_t transition_delay_seconds;

	/* start sensor subsystems */
	//	controlBlink(false);
	//	controlIMU(false);

	controlBlinkNoWindow(sysState.control.blink);
	controlIMUNoWindow(sysState.control.imu);

	/* delay sequence */
	osDelay(blinkCalRX.duration_ms);

	/* stop sensor subsystems and re-enable windowing, if active previously */
	BlinkCalTaskExit(NULL);

	vTaskDelete( NULL );
}

void BlinkCalTaskExit(void *argument){
	blink_calibration_t blinkCalRX;

	if(argument != NULL){
		memcpy(&blinkCalRX,argument,sizeof(blink_calibration_t));
	}
	// disable threads
	osThreadFlagsSet(blinkTaskHandle, TERMINATE_THREAD_BIT);
	osThreadFlagsSet(imuTaskHandle, TERMINATE_THREAD_BIT);

	osDelay(100); // give time for threads to exit
	uint16_t iter;

	volatile osThreadState_t threadStateBlink = osThreadGetState(blinkTaskHandle);
	volatile osThreadState_t threadStateIMU = osThreadGetState(imuTaskHandle);
	while( (threadStateBlink != osThreadTerminated) && (threadStateBlink != osThreadError) && (threadStateBlink != osThreadReady)){
		osDelay(50);
		iter++;
		if(iter > 40){
			//			vTaskDelete( NULL );
			break; // shouldnt get here
		}
		threadStateBlink = osThreadGetState(blinkTaskHandle);
	}
	while( (threadStateIMU != osThreadTerminated) && (threadStateIMU != osThreadError) && (threadStateIMU != osThreadReady)){
		osDelay(50);
		iter++;
		if(iter > 40){
			//			vTaskDelete( NULL );
			break; // shouldnt get here
		}
		threadStateIMU = osThreadGetState(imuTaskHandle);
	}

	// enable
	controlBlink(sysState.control.blink);
	controlIMU(sysState.control.imu);

	if((argument != NULL) && blinkCalRX.enable == 1){
		blinkCalTaskHandle = osThreadNew(BlinkCalTask, &blinkCalRX, &blinkCalTask_attributes);
	}

	vTaskDelete( NULL );
}

volatile uint16_t errorsFound_i2c1 = 0;
volatile uint16_t errorsFound_i2c3 = 0;

void i2c_error_check(I2C_HandleTypeDef *hi2c){

//	if(hi2c == &hi2c3){
//		if( ((hi2c->ErrorCode & 0x20) == 0x20 ) ||
//				((hi2c->ErrorCode & 0x04) == 0x04) ){
//			errorsFound_i2c3 += 1;
//		}else{
//			errorsFound_i2c3 = 0;
//		}
//
//		if(errorsFound_i2c3 == 10){
//			// attempt to send a null address message if problem is because of thermopiles
//			HAL_I2C_MspDeInit(hi2c);
//			MX_I2C3_Init();
//
//			uint8_t data = 0;
//			HAL_I2C_Mem_Write(hi2c, 0, 0x04, 1, &data, 1, 10);
//
//			errorsFound_i2c3 = 0;
//			HAL_Delay(5);
//		}
////		else if (errorsFound_i2c3 == 50){
////			uint8_t data = 0;
////			HAL_I2C_Mem_Write(hi2c, 0, 0x04, 1, &data, 1, 10);
////		}
//		else if (errorsFound_i2c3 > 30){
//			// attempt to reset system
//			__disable_irq();
//			NVIC_SystemReset();
//		}
//	}
//
//	else if(hi2c == &hi2c1){
//		if( ((hi2c->ErrorCode & 0x20) == 0x20 ) ||
//				((hi2c->ErrorCode & 0x04) == 0x04) ){
//			errorsFound_i2c1 += 1;
//		}else{
//			errorsFound_i2c1 = 0;
//		}
//
//		if(errorsFound_i2c1 == 10){
//			// attempt to send a null address message if problem is because of thermopiles
//			HAL_I2C_MspDeInit(hi2c);
//			MX_I2C1_Init();
//
//			uint8_t data = 0;
//			HAL_I2C_Mem_Write(hi2c, 0, 0x04, 1, &data, 1, 10);
//
//			errorsFound_i2c1 = 0;
//			HAL_Delay(5);
////		}else if (errorsFound_i2c3 == 50){
////			uint8_t data = 0;
////			HAL_I2C_Mem_Write(hi2c, 0, 0x04, 1, &data, 1, 10);
//		}else if (errorsFound_i2c1 > 30){
//			// attempt to reset system
//			__disable_irq();
//			NVIC_SystemReset();
//		}
//	}

	return;
}

air_spec_config_packet_t rxConfigPacket;
blue_green_transition_t blueGreenTranRX;
blink_calibration_t blinkCalRX;
red_flash_task_t redFlashRX;
union ColorComplex airspecColors;

void bleRX_Task(void *argument){
	while(1){
		osMessageQueueGet(bleRX_QueueHandle, &rxConfigPacket, 0, osWaitForever);
		volatile osThreadState_t threadState;

		if(rxConfigPacket.has_header == true){
			updateRTC_MS(rxConfigPacket.header.timestamp_unix);
		}

		switch(rxConfigPacket.which_payload) {

		case AIR_SPEC_CONFIG_PACKET_CTRL_INDIV_LED_TAG  :
			if(rxConfigPacket.payload.ctrl_indiv_led.has_left){
				airspecColors.colors_indiv.left_front_b = rxConfigPacket.payload.ctrl_indiv_led.left.forward.blue;
				airspecColors.colors_indiv.left_front_g = rxConfigPacket.payload.ctrl_indiv_led.left.forward.green;
				airspecColors.colors_indiv.left_front_r = rxConfigPacket.payload.ctrl_indiv_led.left.forward.red;

				airspecColors.colors_indiv.left_side_b = rxConfigPacket.payload.ctrl_indiv_led.left.eye.blue;
				airspecColors.colors_indiv.left_side_g = rxConfigPacket.payload.ctrl_indiv_led.left.eye.green;
				airspecColors.colors_indiv.left_side_r = rxConfigPacket.payload.ctrl_indiv_led.left.eye.red;

				airspecColors.colors_indiv.left_top_b = rxConfigPacket.payload.ctrl_indiv_led.left.top.blue;
				airspecColors.colors_indiv.left_top_g = rxConfigPacket.payload.ctrl_indiv_led.left.top.green;
				airspecColors.colors_indiv.left_top_r = rxConfigPacket.payload.ctrl_indiv_led.left.top.red;
			}
			if(rxConfigPacket.payload.ctrl_indiv_led.has_right){
				airspecColors.colors_indiv.right_front_b = rxConfigPacket.payload.ctrl_indiv_led.right.forward.blue;
				airspecColors.colors_indiv.right_front_g = rxConfigPacket.payload.ctrl_indiv_led.right.forward.green;
				airspecColors.colors_indiv.right_front_r = rxConfigPacket.payload.ctrl_indiv_led.right.forward.red;

				airspecColors.colors_indiv.right_side_b = rxConfigPacket.payload.ctrl_indiv_led.right.eye.blue;
				airspecColors.colors_indiv.right_side_g = rxConfigPacket.payload.ctrl_indiv_led.right.eye.green;
				airspecColors.colors_indiv.right_side_r = rxConfigPacket.payload.ctrl_indiv_led.right.eye.red;

				airspecColors.colors_indiv.right_top_b = rxConfigPacket.payload.ctrl_indiv_led.right.top.blue;
				airspecColors.colors_indiv.right_top_g = rxConfigPacket.payload.ctrl_indiv_led.right.top.green;
				airspecColors.colors_indiv.right_top_r = rxConfigPacket.payload.ctrl_indiv_led.right.top.red;
			}
			osMessageQueuePut(lightsComplexQueueHandle, &airspecColors, 0, 0);
			break; /* optional */

		case AIR_SPEC_CONFIG_PACKET_SENSOR_CONTROL_TAG  :
			memcpy(&sysState.control,  &rxConfigPacket.payload.sensor_control, sizeof(sensor_control_t));
			//						controlSensors(&sensorCtrl[0], rxPacketHeader.payloadSize / 2);
			if(sysState.control.synchronize_windows){
				sysState.config.imu.enable_windowing_sync = 1;
				sysState.config.imu.window_size_ms = sysState.control.window_size_ms;
				sysState.config.imu.window_period_ms = sysState.control.window_period_ms;

				sysState.config.blink.enable_windowing_sync = 1;
				sysState.config.blink.window_size_ms = sysState.control.window_size_ms;
				sysState.config.blink.window_period_ms = sysState.control.window_period_ms;
			}else{
				sysState.config.imu.enable_windowing_sync = 0;
				sysState.config.blink.enable_windowing_sync = 0;
			}

			sysState.has_config = true;
			sysState.has_control = true;

			extMemWriteData(START_ADDR + 4, (uint8_t*) &sysState,
							sizeof(system_state_t));
			updateSystemConfig_BLE(&sysState);

			// turn off all sensors
			controlAllSensors(false);
			waitUntilSensorsAreTerminated();

			// restart sensors with new config
			ingestSensorConfig(&sysState);
			break; /* optional */

		case AIR_SPEC_CONFIG_PACKET_SENSOR_CONFIG_TAG  :
			memcpy(&sysState.config,  &rxConfigPacket.payload.sensor_config, sizeof(sensor_config_t));
			//					   memcpy(&sensorCtrl[0], attribute_modified->Attr_Data + sizeof(RX_PacketHeader), rxPacketHeader.payloadSize);
			//						controlSensors(&sensorCtrl[0], rxPacketHeader.payloadSize / 2);
			if(sysState.control.synchronize_windows){
				sysState.config.imu.enable_windowing_sync = 1;
				sysState.config.imu.window_size_ms = sysState.control.window_size_ms;
				sysState.config.imu.window_period_ms = sysState.control.window_period_ms;

				sysState.config.blink.enable_windowing_sync = 1;
				sysState.config.blink.window_size_ms = sysState.control.window_size_ms;
				sysState.config.blink.window_period_ms = sysState.control.window_period_ms;
			}else{
				sysState.config.imu.enable_windowing_sync = 0;

				sysState.config.blink.enable_windowing_sync = 0;
			}

			sysState.has_config = true;
			sysState.has_control = true;

			extMemWriteData(START_ADDR + 4, (uint8_t*) &sysState,
							sizeof(system_state_t));
			updateSystemConfig_BLE(&sysState);

			// turn off all sensors
			controlAllSensors(false);
			waitUntilSensorsAreTerminated();

			// restart sensors with new config
			ingestSensorConfig(&sysState);
			break; /* optional */

		case AIR_SPEC_CONFIG_PACKET_DFU_MODE_TAG  :
			ledEnterDFUNotification();
			enterDFUMode();
			break; /* optional */

		case AIR_SPEC_CONFIG_PACKET_BLUE_GREEN_TRANSITION_TAG  :
			memcpy(&blueGreenTranRX, &rxConfigPacket.payload.blue_green_transition, sizeof(blue_green_transition_t));
			threadState = osThreadGetState(blueGreenTranTaskHandle);

			// if thread is currently running, restart it
			if((threadState == osThreadReady) || (threadState == osThreadRunning) || (threadState == osThreadBlocked)){
				osThreadTerminate(blueGreenTranTaskHandle); // terminate any existing running thread

				BlueGreenTransitionTaskExit(&blueGreenTranRX);
			}else{ // if thread is not running and needs to be started

				// check if task is still exiting (this would happen if an enable is called rapidly after a disable)
				threadState = osThreadGetState(blueGreenExitTaskHandle);
				if((threadState == osThreadReady) || (threadState == osThreadRunning) || (threadState == osThreadBlocked)){
					break;
				}
				if(blueGreenTranRX.enable == 1){
					blueGreenTranTaskHandle = osThreadNew(BlueGreenTransitionTask, &blueGreenTranRX, &blueGreenTask_attributes);
				}
			}
			break; /* optional */

		case AIR_SPEC_CONFIG_PACKET_BLINK_CALIBRATION_TAG:
			memcpy(&blinkCalRX, &rxConfigPacket.payload.blink_calibration, sizeof(blink_calibration_t));

			threadState = osThreadGetState(blinkCalTaskHandle);
			if((threadState != osThreadTerminated) &&(threadState != osThreadInactive)  && (threadState != osThreadError)){
				osThreadTerminate(blinkCalTaskHandle);
				//					blinkCalTaskExitHandle = osThreadNew(BlinkCalTaskExit, &blinkCalRX, &blinkCalTask_attributes);
				BlinkCalTaskExit(&blinkCalRX);
			}

			else if(blinkCalRX.enable == 1){
				blinkCalTaskHandle = osThreadNew(BlinkCalTask, &blinkCalRX, &blinkCalTask_attributes);
			}
			break;

		case AIR_SPEC_CONFIG_PACKET_RED_FLASH_TASK_TAG  :
			memcpy(&redFlashRX, &rxConfigPacket.payload.red_flash_task, sizeof(red_flash_task_t));
			osThreadTerminate(redFlashTaskHandle); // terminate any existing running thread
			resetLED();
			if(redFlashRX.enable == 1){
				redFlashTaskHandle = osThreadNew(RedFlashTask, &redFlashRX, &redFlashTask_attributes);
			}
			break; /* optional */

			/* you can have any number of case statements */
		default : /* Optional */
			break;
		}
	}
}

