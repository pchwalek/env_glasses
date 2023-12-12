/*
 * imu.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "imu.h"
#include "Adafruit_ICM20948.h"
#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "captivate_config.h"
#include "FreeRTOS.h"
#include "task.h"


//#define IMU_SAMPLE_PERIOD_MS		5

#define IMU_SAMPLE_PERIOD_MS		50

//#define MAX_IMU_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(imu_sample)
#define MAX_IMU_SAMPLES_PACKET	1
#define MAX_IMU_PKT_SIZE	408

//static void triggerIMUSample(void *argument);

//static imu_sample imuData[MAX_IMU_SAMPLES_PACKET];

//static PacketHeader header;
osTimerId_t periodicIMUTimer_id;

Adafruit_ICM20948 imu;
#define MAX_FIFO_CNT 4096
#define IMU_PKT_SIZE 12
static uint8_t data[MAX_FIFO_CNT];
uint8_t flag_int_enable = 0;

osTimerId_t singleShotTimer_id;
static uint32_t startTime;
static uint64_t sampleStartTime_Unix;
static uint32_t sampleStartTime_Ms;
//static uint8_t tempData[MAX_FIFO_CNT];
volatile uint32_t endTestTick, startTestTick;
volatile float sampleRateTest, counterTest = 0;

void IMU_Task(void *argument){
	sensor_packet_t *packet = NULL;
	uint32_t flags = 0;
	uint32_t flag_rdy = 0;
	osThreadFlagsClear(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT);
	//	bool status;

	osDelay(1000);

	imu_sensor_config_t sensorSettings;

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(imu_sensor_config_t));
	}else{
		sensorSettings.gyro_settings.has_cutoff = true;
		sensorSettings.gyro_settings.cutoff = IMU_GYRO_CUTOFF_ICM20_X_GYRO_FREQ_196_6_HZ;
		sensorSettings.gyro_settings.range = IMU_GYRO_RANGE_RANGE_2000_DPS;
		sensorSettings.gyro_settings.sample_rate_divisor = 1;
		sensorSettings.accel_settings.has_cutoff = true;
		sensorSettings.accel_settings.cutoff = IMU_ACCEL_CUTOFF_ICM20_X_ACCEL_FREQ_246_0_HZ;
		sensorSettings.accel_settings.range = IMU_ACCEL_RANGE_RANGE_8_G;
		sensorSettings.accel_settings.sample_rate_divisor = 1;

		sensorSettings.enable_windowing = false;
		sensorSettings.enable_windowing_sync = false;
		//		sensorSettings.window_size_ms;
		//		sensorSettings.window_period_ms;
	}

	imu.updateGyroSettings(sensorSettings.gyro_settings.has_cutoff,
			sensorSettings.gyro_settings.cutoff,
			sensorSettings.gyro_settings.range,
			sensorSettings.gyro_settings.sample_rate_divisor);
	imu.updateAccelSettings(sensorSettings.accel_settings.has_cutoff,
			sensorSettings.accel_settings.cutoff,
			sensorSettings.accel_settings.range,
			sensorSettings.accel_settings.sample_rate_divisor);


	while(!imu.begin_SPI(&hspi2,IMU_CS_GPIO_Port,IMU_CS_Pin)){
		osDelay(100);
	}

	osDelay(1);

	//  uint16_t imuIdx = 0;
	uint32_t imuID = 0;

	imu.spiDataReady = HAL_SPI_IMU_WAIT;

	//  uint8_t intStatus[4] = {0};

	uint16_t fifo_cnt;

	// sample frequency calculation
	float sampleFrequency = 1125.0 / (1+sensorSettings.accel_settings.sample_rate_divisor);
	if(sampleFrequency < (1100.0 / (1+sensorSettings.gyro_settings.sample_rate_divisor))){
		sampleFrequency = 1100.0 / (1+sensorSettings.gyro_settings.sample_rate_divisor);
	}
	float sample_time_offset_ms = 1000.0/sampleFrequency;

	uint16_t sampleTracker = 0;
	uint16_t start_idx = 0;
	//  uint16_t end_idx = 0;
	uint32_t packetTracker = 0;

	//  uint32_t lastTick = HAL_GetTick();
	//  float sampleRate = 0;

	startTime = HAL_GetTick();
	sampleStartTime_Ms = startTime;
	sampleStartTime_Unix = getEpoch();

	volatile float sampleRateTracker = 0;
	volatile uint32_t tempTimeTracker = HAL_GetTick();
	volatile uint64_t sampleCounter = 0 ;

	uint16_t failed_read_attempt = 0;

	while(1){
		//  	flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
		//  					osFlagsWaitAny, osWaitForever);
		if(1){
			osDelay(IMU_SAMPLE_PERIOD_MS);

			flags = osThreadFlagsGet();


			if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
				imu.reset();
				vTaskDelete( NULL );
			}




			imu.getFIFOcnt(&fifo_cnt);


			if(fifo_cnt != 0){





				fifo_cnt = fifo_cnt - (fifo_cnt % IMU_PKT_SIZE); // ensure reading only complete packets

				flag_int_enable = 1;
				osDelay(1);
				if( (imu.readFIFO(data, fifo_cnt) != false) && (fifo_cnt > 0)){

					sampleTracker += fifo_cnt;

					sampleCounter = sampleCounter + fifo_cnt / 12.0;


					//				lastTick = HAL_GetTick();
					start_idx = 0;


					while(sampleTracker != 0){

						if(sampleTracker >= MAX_IMU_PKT_SIZE){
							packetTracker = MAX_IMU_PKT_SIZE;
						}else{
							packetTracker = sampleTracker;
						}

						packet = grabPacket();
						while(packet == NULL){
							osDelay(10);
							packet = grabPacket();

						}

						setPacketType(packet, SENSOR_PACKET_TYPES_IMU);

						packet->payload.imu_packet.packet_index = imuID;

						packet->payload.imu_packet.has_accel_settings = true;
						packet->payload.imu_packet.accel_settings.has_cutoff = sensorSettings.accel_settings.has_cutoff;
						packet->payload.imu_packet.accel_settings.cutoff = static_cast<imu_accel_cutoff_t>(sensorSettings.accel_settings.cutoff);
						packet->payload.imu_packet.accel_settings.range =static_cast<imu_accel_range_t>( sensorSettings.accel_settings.range );
						packet->payload.imu_packet.accel_settings.sample_rate_divisor = sensorSettings.accel_settings.sample_rate_divisor;

						packet->payload.imu_packet.has_gyro_settings = true;
						packet->payload.imu_packet.gyro_settings.has_cutoff = sensorSettings.gyro_settings.has_cutoff;
						packet->payload.imu_packet.gyro_settings.cutoff = static_cast<imu_gyro_cutoff_t>(sensorSettings.gyro_settings.cutoff);
						packet->payload.imu_packet.gyro_settings.range = static_cast<imu_gyro_range_t>(sensorSettings.gyro_settings.range );
						packet->payload.imu_packet.gyro_settings.sample_rate_divisor = sensorSettings.gyro_settings.sample_rate_divisor;

						packet->payload.imu_packet.sampling_frequency = sampleFrequency;

						packet->payload.imu_packet.timestamp_unix = sampleStartTime_Unix;
						packet->payload.imu_packet.timestamp_ms_from_start = startTime;

						sampleStartTime_Unix += sample_time_offset_ms * ((float) packetTracker)/12.0;
						sampleStartTime_Ms += sample_time_offset_ms * ((float) packetTracker)/12.0;

						// write data
						memcpy(packet->payload.imu_packet.payload.sample.bytes, &data[start_idx], packetTracker);
						packet->payload.imu_packet.has_payload = true;
						packet->payload.imu_packet.payload.sample.size = packetTracker;
						counterTest += packetTracker;
						// send to BT packetizer
						queueUpPacket(packet, 10);

						sampleTracker -= packetTracker;
						start_idx += packetTracker;
						imuID++;
					}

				}
				flag_int_enable = 0;
			}else{
				failed_read_attempt+=1;
				if(failed_read_attempt >= 10){
					imu._init(); // reinit imu
					failed_read_attempt = 0;
				}
				sampleStartTime_Unix = getEpoch();
				sampleStartTime_Ms = HAL_GetTick();
			}

		}

		if(sensorSettings.enable_windowing){

			if(sensorSettings.window_size_ms < (HAL_GetTick() - startTime)){

				sampleRateTracker = ((double) sampleCounter) / (HAL_GetTick() - tempTimeTracker);
				//			endTestTick = HAL_GetTick() - startTestTick;
				//			sampleRateTest = (counterTest / 12.0) / (endTestTick / 1000.0);

				imu.reset();


				if(sensorSettings.enable_windowing_sync){
					flags = osThreadFlagsWait(TERMINATE_THREAD_BIT | WINDOW_SYNC_RDY_BIT,
							osFlagsWaitAny, osWaitForever);
				}else{
					int32_t waitTime = sensorSettings.window_period_ms - sensorSettings.window_size_ms;
					if(waitTime < 0) waitTime = 0;

					flags = osThreadFlagsWait(TERMINATE_THREAD_BIT | WINDOW_SYNC_RDY_BIT,
							osFlagsWaitAny, waitTime);
				}


				if((flags != osFlagsErrorTimeout) && ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT)) {
					vTaskDelete( NULL );
				}

				if((flags & WINDOW_SYNC_RDY_BIT) == WINDOW_SYNC_RDY_BIT){
					// this check is unnessary but leaving it in in case we want to expand further
				}



				imu.updateGyroSettings(sensorSettings.gyro_settings.has_cutoff,
						sensorSettings.gyro_settings.cutoff,
						sensorSettings.gyro_settings.range,
						sensorSettings.gyro_settings.sample_rate_divisor);
				imu.updateAccelSettings(sensorSettings.accel_settings.has_cutoff,
						sensorSettings.accel_settings.cutoff,
						sensorSettings.accel_settings.range,
						sensorSettings.accel_settings.sample_rate_divisor);




				while(!imu.begin_SPI(&hspi2,IMU_CS_GPIO_Port,IMU_CS_Pin)){
					osDelay(100);
				}

				osDelay(1);

				fifo_cnt = 0;
				imu.spiDataReady = HAL_SPI_IMU_WAIT;
				//			startTestTick = HAL_GetTick();
				//			counterTest = 0;

				startTime = HAL_GetTick();
				sampleStartTime_Ms = startTime;
				sampleStartTime_Unix = getEpoch();

				tempTimeTracker = HAL_GetTick();
				sampleRateTracker = 0;
				sampleCounter = 0;

			}


		}

		//	  else{
		//  		failed_read_attempt+=1;
		//  		if(failed_read_attempt >= 10){
		//  			imu._init(); // reinit imu
		//  			failed_read_attempt = 0;
		//  		}
		//
		//  	}

		//		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
		//			osTimerDelete(periodicIMUTimer_id);
		//		}
	}


}

//static void triggerIMUSample(void *argument) {
//	osThreadFlagsSet(imuTaskHandle, GRAB_SAMPLE_BIT);
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//	if(GPIO_Pin == IMU_INT_Pin) triggerIMUSample(NULL);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	// send flag to IMU
	if(flag_int_enable){
		osThreadFlagsSet(imuTaskHandle, IMU_DATA_RDY_BIT);
		flag_int_enable = 0;
	}

}

void IMUSyncTrigger(void) {
	osThreadFlagsSet(imuTaskHandle, WINDOW_SYNC_RDY_BIT);
}

void HAL_SPI_IMU_WAIT(uint8_t *state){
	uint32_t flag = osThreadFlagsWait(IMU_DATA_RDY_BIT,
			osFlagsWaitAny, 100);

	if(flag == osFlagsErrorTimeout){
		*state = 0;
	}else if ((flag & IMU_DATA_RDY_BIT) == IMU_DATA_RDY_BIT){
		*state = 1;
	}

	return;
}
