/*
 * mic.cpp
 *
 *  Created on: Jan 4, 2022
 *      Author: patri
 */

#include "mic.h"

#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"

#include "stm32wbxx_hal_sai.h"
#include "sai.h"

#include "arm_math.h"
#include "math.h"


// pretty OK tutorial: https://stm32f4-discovery.net/2014/10/stm32f4-fft-example/

//#define MIC_DATA_SIZE		4096 // make multiple of 2 for simplicity
#define MIC_DATA_SIZE (4096 * 2)
#define MIC_FFT_DATA_SIZE (MIC_DATA_SIZE >> 1)
#define MIC_HALF_DATA_SIZE	(MIC_DATA_SIZE >> 1) // make multiple of 2 for simplicity

/* if the microphone sample period is equal or greater than MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF,
 * the microphone will be turned off in between samples
 */
#define MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF	5000

union MicSample{
  float    float_val[MIC_DATA_SIZE];
  uint32_t uint_val[MIC_DATA_SIZE];
  int32_t  int_val[MIC_DATA_SIZE];
};

static union MicSample micData;

//float micData[MIC_DATA_SIZE];
//float micDataFloat[MIC_DATA_SIZE];

void triggerMicSample(void *argument);
void calculateSoundLevel(float *data, uint32_t size, double *dB, double *RMS);

//#define MAX_LIDAR_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(lidar_sample)

#define MAX_MIC_SAMPLES_PACKET  10

#define MIC_LEVEL_SUB_SAMPLE_CNT 10

//static PacketHeader header;
//arm_rfft_instance_q31 fft_instance;
arm_rfft_fast_instance_f32 fft_instance;
volatile uint8_t buffer_tracker;

osTimerId_t periodicMicTimer_id;

volatile uint8_t start_logging = 0;
volatile uint8_t secondMicSample = 0;
volatile uint8_t micLowPowerMode = 0;

volatile uint32_t micSampleCheck = 0;
volatile float finalDBMeasurement = 0;
volatile double short_RMS_total = 0;


static mic_sensor_config_t sensorSettings;

typedef struct micSamples {
	uint32_t lux;
	uint32_t timestamp;
} luxSample;

void Mic_Task(void *argument){
	sensor_packet_t *packet = NULL;

	uint32_t micID = 0;
	uint32_t micLevelIdx = 0;
	uint32_t flags = 0;
	float32_t maxvalue;
	uint32_t maxindex;
	float32_t dominantFrequency;
	uint16_t startIdx;
	uint32_t fft_index = 0;
	uint64_t fft_time_unix = 0;
	uint32_t fft_time_ms_from_start = 0;

	uint8_t micSubSampleCounter = 0;

	double soundLevel_dB_accumulator = 0;
	double soundLevel_RMS_accumulator = 0;

//	bool status;

	float startFreq;


//	arm_rfft_init_q31(&fft_instance, MIC_HALF_DATA_SIZE, 0, 0);
	arm_rfft_fast_init_f32(&fft_instance, MIC_FFT_DATA_SIZE);

	if(argument != NULL){
		memcpy(&sensorSettings,argument,sizeof(struct MicSensor));
	}else{
		sensorSettings.mic_sample_freq = SAI_AUDIO_FREQUENCY_48K;
		sensorSettings.sample_period_ms = 30000; // every 30 seconds
	}


	float fft_spacing = 48000 / float(MIC_FFT_DATA_SIZE);


//	memcpy(&header.reserved[4], (uint32_t *) &fft_spacing, sizeof(fft_spacing));

	uint16_t maxMicPayloadSize = floor( (420) / 4); // number of 4 byte floats
	uint32_t totalMicPayloadSize = (MIC_FFT_DATA_SIZE >> 1) - 1; // number of 4 byte floats
	uint8_t packetsPerMicSample = ceil( ((float) totalMicPayloadSize) / (maxMicPayloadSize) );

	uint32_t sample_count;

	hsai_BlockA1.Init.AudioFrequency = sensorSettings.mic_sample_freq;

	MX_SAI1_Init();

	HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);

	/* prime the SAI channel
	 * (needs to be started to ensure first sample does not have
	 * null bytes since I2S takes a few cycles to turn on)
	 */
	HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *) micData.uint_val, 256, 1);  //purposeful short timeout

	if(sensorSettings.sample_period_ms > MIC_SAMPLE_PERIOD_MS_THRESH_TO_TURN_OFF){
		micLowPowerMode = 1;
	}else{
		micLowPowerMode = 0;
		HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);
	}

	periodicMicTimer_id = osTimerNew(triggerMicSample, osTimerPeriodic,
				NULL, NULL);

	volatile osStatus_t timerStatus;
	timerStatus = osTimerStart(periodicMicTimer_id, sensorSettings.sample_period_ms / MIC_LEVEL_SUB_SAMPLE_CNT);
//	start_logging = 1;

	while(1){
		flags = osThreadFlagsWait(GRAB_SAMPLE_BIT | TERMINATE_THREAD_BIT,
				osFlagsWaitAny, osWaitForever);

//		fft_time_unix = getEpoch();
//		fft_time_ms_from_start = HAL_GetTick();


		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
			if(micLowPowerMode){
				HAL_SAI_DeInit(&hsai_BlockA1);
				HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);
			}

		/* shifting by 8 so that the 24-bit (2's complement) number
		   * is in the 24 MSBs. When using ARM functions, we will treat
		   * this uint32_t array as a Q31 format where it'll be treated
		   * as if the original values were divided by (2^24)
		   */

		  for(int i = 0; i<MIC_DATA_SIZE; i++){
			  micData.float_val[i] = ((int32_t) (micData.uint_val[i] << 8)) >> 8;
		  }



		  /* convert to float and scale by 2^24 so that we get the exact values
		   * as intended from the microphone. However, because scale doesn't matter
		   * for FFT, we can theoretically skip the below two steps and do a Q31
		   * FFT to save <10ms in computation if not looking for calibrated data.
		   */
//		  arm_q31_to_float ((q31_t *) &micData[0],
//						&micDataFloat[0], MIC_DATA_SIZE); //~4ms (16MHz, 4096 data size)

//		  arm_scale_f32 (&micDataFloat[0], 16777216, &micDataFloat[0], MIC_DATA_SIZE); //~4ms (16MHz, 4096 data size)

//		  for(int i = 0; i<MIC_DATA_SIZE; i++){
//			  micData.float_val[i] = micData.int_val[i] >> 8;
//		  }

		  /* calculate noise level */
		  /* note: this function adds onto the two accumulator variables and doesn't replace their value */
		  calculateSoundLevel(&micData.float_val[0], MIC_DATA_SIZE, &soundLevel_dB_accumulator, &soundLevel_RMS_accumulator);

		  micSubSampleCounter++;

		  if(micSubSampleCounter >= MIC_LEVEL_SUB_SAMPLE_CNT){

			  soundLevel_dB_accumulator /= micSubSampleCounter;
			  soundLevel_RMS_accumulator /= micSubSampleCounter;

			  micSubSampleCounter = 0;

			  /* WARNING: this function modifies dataMicF */
			  arm_rfft_fast_f32(&fft_instance, micData.float_val, micData.float_val, 0); //~22ms (16MHz, 4096 data size)

			  arm_cmplx_mag_f32(micData.float_val, micData.float_val, MIC_FFT_DATA_SIZE >> 1); //~6ms (16MHz, 2048 data size)

			  arm_max_f32(&micData.float_val[1], (MIC_FFT_DATA_SIZE >> 1) - 1, &maxvalue, &maxindex); //~2ms (16MHz, 2048 data size)

			  dominantFrequency = maxindex * fft_spacing;


			  	/* packetize sound level data */
			  packet = grabPacket();
				while( packet == NULL ){
					osDelay(2);
					packet = grabPacket();
				}

				setPacketType(packet, SENSOR_PACKET_TYPES_MIC_LEVEL);


				packet->payload.mic_level_packet.mic_sample_freq = sensorSettings.mic_sample_freq;

				packet->payload.mic_level_packet.num_of_samples_used = MIC_LEVEL_SUB_SAMPLE_CNT;
				packet->payload.mic_level_packet.packet_index = micLevelIdx;
				packet->payload.mic_level_packet.sample_length =  MIC_DATA_SIZE;
				packet->payload.mic_level_packet.sample_period =  sensorSettings.sample_period_ms / MIC_LEVEL_SUB_SAMPLE_CNT;
				packet->payload.mic_level_packet.weighting = MIC_WEIGHTING_A_WEIGHT;

				// write data
				packet->payload.mic_level_packet.payload[0].sound_rms = soundLevel_RMS_accumulator;
				packet->payload.mic_level_packet.payload[0].sound_spl_db = soundLevel_dB_accumulator;
				packet->payload.mic_level_packet.payload[0].timestamp_ms_from_start = HAL_GetTick();
				packet->payload.mic_level_packet.payload[0].timestamp_unix = getEpoch();
				packet->payload.mic_level_packet.payload_count = 1;

				packet->payload.mic_packet.payload.sample_count = sample_count;

				queueUpPacket(packet, 100);
				micLevelIdx++;


				/* packetize FFT data */
				for(int i = 0; i < packetsPerMicSample; i++){

					packet = grabPacket();
					while( packet == NULL ){
						osDelay(2);
						packet = grabPacket();
					}

						setPacketType(packet, SENSOR_PACKET_TYPES_MIC);


						packet->payload.mic_packet.packet_index = micID;


						packet->payload.mic_packet.timestamp_unix = getEpoch();
						packet->payload.mic_packet.timestamp_ms_from_start = HAL_GetTick();

						packet->payload.mic_packet.frequency_spacing = fft_spacing;
						packet->payload.mic_packet.mic_sample_freq = sensorSettings.mic_sample_freq;
						packet->payload.mic_packet.sample_period = sensorSettings.sample_period_ms;
						packet->payload.mic_packet.packets_per_fft = packetsPerMicSample; // total number of packets required to send full FFT
						packet->payload.mic_packet.samples_per_fft = totalMicPayloadSize; // total number of packets required to send full FFT


						startIdx = maxMicPayloadSize * i;

						packet->payload.mic_packet.fft_index = startIdx + 1;

						if( (startIdx + maxMicPayloadSize) > totalMicPayloadSize){
							sample_count = (totalMicPayloadSize - startIdx);
	//						message.sample_count = totalMicPayloadSize - startIdx;
						}else{
							sample_count = maxMicPayloadSize;
	//						message.sample_count = maxMicPayloadSize;
						}

						startFreq = (startIdx + 1) * fft_spacing;
						packet->payload.mic_packet.start_frequency = startFreq;

						// write data
						memcpy(packet->payload.mic_packet.payload.sample, (uint8_t *) &micData.float_val[startIdx + 1], sample_count * 4);
						packet->payload.mic_packet.has_payload = true;
						packet->payload.mic_packet.payload.sample_count = sample_count;


						// send to BT packetizer
						queueUpPacket(packet, 20);


					micID++;

				}
				fft_index++;
			  }
		}

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete(periodicMicTimer_id);
			HAL_SAI_Abort(&hsai_BlockA1);
			HAL_SAI_DeInit(&hsai_BlockA1);
			HAL_SAI_MspDeInit(&hsai_BlockA1);
			osThreadExit();
//			start_logging = 0;
			break;
		}
	}

	osThreadExit();
}

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
//SOS_IIR_Filter ICS43434 = {
//  gain: 0.477326418836803,
//  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
//   {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
//   {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
//  }
//};

arm_biquad_cascade_df2T_instance_f32 equalizerIIR;
float equalizerGain =  0.477326418836803;
const float equalizerCoef[10] = {1.0, 0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128,
								   1.0, -1.98905931743624453, 0.98908924206960169, 1.99755331853906037, -0.99755481510122113};
float equalizerIIR_state[4] = {0,0,0,0};

//const float equalizerCoef[10] = {0,   0.4773 ,  0.2320      ,     1.0000  , 0.0344,
//		0 ,  1.0000  , 0.4838          , 1.0000 ,  0.0324,
//		0 ,  1.0000 , -0.9945         , 1.0000 , -0.9988,
//		0 ,  1.0000 , -0.9945        ,   1.0000 , -0.9988};




////
//// A-weighting IIR Filter, Fs = 48KHz
//// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
//// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
//// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
//SOS_IIR_Filter A_weighting = {
//  gain: 0.169994948147430,
//  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
//    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
//    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
//    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
//  }
//};
//
////
//// C-weighting IIR Filter, Fs = 48KHz
//// Designed by invfreqz curve-fitting, see respective .m file
//// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
//// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
//SOS_IIR_Filter C_weighting = {
//  gain: -0.491647169337140,
//  sos: {
//    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
//    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
//    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
//  }
//};
arm_biquad_cascade_df2T_instance_f32 weightingIIR;
/* C-weighting */
//float weightingGain =  -0.491647169337140;
//const float weightingCoef[15] = {1.0, 1.4604385758204708, 0.5275070373815286, 1.9946144559930252, -0.9946217070140883,
//				    1.0, 0.2376222404939509, 0.0140411206016894, -1.3396585608422749, -0.4421457807694559,
//				    1.0, -2.0000000000000000, 1.0000000000000000, 0.3775800047420818, -0.0356365756680430};
//float weightingIIR_state[4] = {0,0,0,0};



/* A-weighting */
//float weightingGain =  0.169994948147430;
float weightingGain =  0.2343;
//const float weightingCoef[15] = {1.0, -2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926,
//				    1.0, +4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332,
//				    1.0, -0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989};
float weightingIIR_state[4] = {0,0,0,0};
const float weightingCoef[15] = {1.0000  ,  2.0000 ,   1.0000    ,  0.2246  ,  -0.0126,
		1.0000 ,  -2.0002 ,   1.0002  ,  1.8939 ,   -0.8952,
		1.0000 ,  -1.9998 ,   0.9998  ,  1.9946 ,   -0.9946};


#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);
uint32_t Leq_samples = 0;
double Leq_sum_sqr = 0;
double Leq_dB = 0;

#define SAMPLE_T          int32_t


// adapted from https://github.com/ikostoski/esp32-i2s-slm/blob/master/esp32-i2s-slm.ino
void calculateSoundLevel(float *data, uint32_t size, double *dB, double *RMS){
	// Sum of squares of mic samples, after Equalizer filter
	float sum_sqr_SPL;
	// Sum of squares of weighted mic samples
	float sum_sqr_weighted;

	/* initialize filters */
	arm_biquad_cascade_df2T_init_f32(&equalizerIIR, 2,
			equalizerCoef, equalizerIIR_state );

	arm_biquad_cascade_df2T_init_f32(&weightingIIR, 3,
				weightingCoef, weightingIIR_state );

	/* equalize */
	arm_biquad_cascade_df2T_f32(&equalizerIIR, data, data, size);
	arm_scale_f32 (data, equalizerGain, data, size);
	arm_power_f32 (data, size, &sum_sqr_SPL);

	/* apply weighting */
	arm_biquad_cascade_df2T_f32(&weightingIIR, data, data, size);
	arm_scale_f32 (data, weightingGain, data, size);
	arm_power_f32 (data, size, &sum_sqr_weighted);

    /* Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference */
    double short_RMS = sqrt(double(sum_sqr_weighted) / size);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    short_RMS_total += short_RMS;

    *dB += short_SPL_dB;
    *RMS += short_RMS;

//    // Accumulate Leq sum
//    Leq_sum_sqr += q.sum_sqr_weighted;
//    Leq_samples += SAMPLES_SHORT;

//    // When we gather enough samples, calculate new Leq value
//    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
//      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
//      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
//      Leq_sum_sqr = 0;
//      Leq_samples = 0;
//
//      // Serial output, customize (or remove) as needed
//      Serial.printf("%.1f\n", Leq_dB);
//
//      // Debug only
//      //Serial.printf("%u processing ticks\n", q.proc_ticks);
//    }
}

void triggerMicSample(void *argument){
	portENTER_CRITICAL();
	HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData.uint_val, MIC_DATA_SIZE);
	portEXIT_CRITICAL();
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	/* found that when the mic clock gets started via the interrupt call,
	 * the first several bytes (up to several 100) can be null so a second go
	 * is warranted.
	 *
	 * this is because mic takes 20ms to wakeup as per the datasheet
	 */
//	if(start_logging){
		if( (secondMicSample == 0) && (micLowPowerMode) ){
			secondMicSample = 1;
			HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t *) micData.uint_val, MIC_DATA_SIZE);
		}
		else{
			secondMicSample = 0;
			osThreadFlagsSet(micTaskHandle, GRAB_SAMPLE_BIT);
		}
//	}
}

//void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
//
//}
