/**
 ******************************************************************************
 * File Name          : config.h
 * Description        : configuration file
 ******************************************************************************

 *
 ******************************************************************************
 */
#ifndef CAPTIVATE_CONFIG_H
#define CAPTIVATE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------*/
#include "cmsis_os2.h"
#include "app_conf.h"
/* typedef -----------------------------------------------------------*/

/* defines -----------------------------------------------------------*/

// only define the below variables if wanting to send a packet at a period of NETWORK_TEST_PERIOD ms
//#define NETWORK_TEST			1
//#define NETWORK_TEST_PERIOD		1000

// uncomment if programming the STM32 USB dongle
#define CUSTOM_NETWORK_STACK	1
#define DYNAMIC_MODE	1
//#define DONGLE_CODE					1
//#define OTA_ENABLED					1
//#define BORDER_ROUTER_NODE			1
//#define BORDER_ROUTER_NODE_TRANSMITTER	1
//#define LED_TEST	1
#define TEST_RUN_ON_START 1
//#define TEST_TOUCH 1
#ifdef DONGLE_CODE
#define NODE_TYPE				"dongle"  // max 11 bytes
#define NODE_DESCRIPTION  		"don_1"	  // max 11 bytes
#else
#define NODE_TYPE				"captivates"  // max 11 bytes
#define NODE_DESCRIPTION  		"cap_1"	  // max 11 bytes
#endif

#ifdef BORDER_ROUTER_NODE
#define SEND_TOGLOG_START		1
#define SEND_TOGLOG_STOP		2
#define SET_BLINK_ON  		    2
#endif

//// enable sensing subsystems
#define BLINK_SENSING_ENABLE		0
#define TEMP_SENSING_ENABLE			0
#define POS_SENSING_ENABLE			0
#define	INERTIA_SENSING_ENABLE		0

#define DISABLE_SENSING				0

/* SYSTEM GLOBAL CONFIGURATION */
#define PACKET_SEND_PERIOD			100

#define C_LIGHT_RESSOURCE						"light"
#define C_LIGHTS_SIMPLE_RESSOURCE				"lightS"
#define C_LIGHTS_COMPLEX_RESSOURCE				"lightC"
#define C_BORER_TIME_RESSOURCE					"borderTime"
#define C_BORDER_PACKET_RESSOURCE				"borderLog"
#define C_NODE_INFO_RESSOURCE					"nodeInfo"
#define C_TOGGLE_LOGGING_RESSOURCE				"togLog"

#ifdef OTA_ENABLED
#define C_RESSOURCE_FUOTA_PROVISIONING     "FUOTA_PROVISIONING"
#define C_RESSOURCE_FUOTA_PARAMETERS       "FUOTA_PARAMETERS"
#define C_RESSOURCE_FUOTA_SEND             "FUOTA_SEND"

#define FUOTA_NUMBER_WORDS_64BITS                50
#define FUOTA_PAYLOAD_SIZE                FUOTA_NUMBER_WORDS_64BITS * 8
#endif

#define ENABLE_LOG								1
#define DISABLE_LOG								0
#define SENSOR_ENABLE							1
#define LIGHT_LAB_DEMO							2
#define TARE_NOW								3
#define SEND_VIA_BLE							4

/* RADIO SPECIFIC */
#define TRANSMIT_POWER							6 //in dbm
//#define TRANSMIT_POWER							-3 //in dbm

#define CHILD_SUPERVISION_INTERVAL				2 // default is 129 (how often a router broadcasts to its child to ensure its alive)
#define CHILD_SUPERVISION_TIMEOUT				3	// default is 190 (when child trying to find a new router)

/* WATCHDOG TIMER */
#define WATCHDOG_PERIOD							5000 // 5 seconds

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

/* VIVE SPECIFIC CONFIGURATION */
#define VIVE_SAMPLE_PERIOD			2000
#define	GET_3D_LOC_TIMEOUT			1500
//#define VIVE_THREAD_INFINITE_TIMEOUT	1

/* TOUCH SPECIFIC CONFIGURATION */
#define ALPHA_WEIGHT 	0.01    // updated_va = new_val * (ALPHA_WEIGHT) + old_val * (1-ALPHA_WEIGHT)
#define CALIBRATION_SAMPLES 100 // how many samples to take before starting sensing
#define THRESHOLD_TOLERANCE	60 // how much below the dynamic threshold before classifying as a "touch"

//#define TSCx_TS1_MINTHRESHOLD			0
//#define TSCx_TS1_MAXTHRESHOLD			7400
//#define TSCx_TS2_MINTHRESHOLD			0
//#define TSCx_TS2_MAXTHRESHOLD			8800

/* IMU SPECIFIC CONFIGURATION */
#define ROT_VEC_PERIOD				100
#define ACT_CLASS_PERIOD			500
#define STEP_CNT_PERIOD				500
#define STABILITY_CLASS_PERIOD		500
#define IMU_CALIBRATION				1
#define ACC_GYRO_PACKET_SIZE		14
#define ACC_GYRO_QUEUE_SIZE			5
#define MAX_THREE_AXIS_PAYLOAD_ENTRIES	25
//#define INERTIAL_ACC_GYRO_EN		1 // comment out if wanting just the rotation matrix and normal use

/* BLINK SPECIFIC CONFIGURATION */
#define BLINK_SAMPLE_RATE		1000
#define BLINK_HALF_BUFFER_SIZE	1000
#define BLINK_PACKET_SIZE		400
#define BLINK_ITERATOR_COUNT 	BLINK_HALF_BUFFER_SIZE / BLINK_PACKET_SIZE
#define INFRARED_DETECT_ALPHA 	0.5
#define INFRARED_DETECT_LOWER_THRESH 	7
#define INFRARED_DETECT_UPPER_THRESH 	235


/* TP SPECIFIC CONFIGURATION */
#define MAX_THERMAL_ENTRIES	4

 /* PACKET DESCRIPTORS */
#define BLINK_DATA 		5
#define THERMAL_DATA		6
#define ACC_DATA		7
#define MAG_DATA		8
#define GYRO_DATA		9

//#define MAX_PACKET_QUEUE_SIZE 	10

#define PACKET_SEND_SUCCESS	0
#define PACKET_LENGTH_EXCEEDED	1
#define PACKET_UNDEFINED_ERR	10
//#define MAX_PACKET_LEN		DATA_NOTIFICATION_MAX_PACKET_SIZE
#define MAX_PACKET_QUEUE_SIZE	10
/* macros ------------------------------------------------------------*/

/* function prototypes -----------------------------------------------*/

/* freertos types  -----------------------------------------------*/


extern osThreadId_t blinkTaskHandle;
extern osMessageQueueId_t blinkMsgQueueHandle;

extern osThreadId_t frontLightsTaskHandle;
extern osMessageQueueId_t lightsSimpleQueueHandle;

extern osThreadId_t frontLightsComplexTaskHandle;
extern osMessageQueueId_t lightsComplexQueueHandle;

extern osMessageQueueId_t togLoggingQueueHandle;
extern osThreadId_t masterTaskHandle;

extern osThreadId_t blueGreenTranTaskHandle;
extern const osThreadAttr_t blueGreenTask_attributes;

extern osThreadId_t redFlashTaskHandle;
extern const osThreadAttr_t redFlashTask_attributes;

extern osThreadId_t specTaskHandle;
extern const osThreadAttr_t specTask_attributes;

extern osThreadId_t thermopileTaskHandle;
extern const osThreadAttr_t thermopileTask_attributes;

extern osThreadId_t luxTaskHandle;
extern const osThreadAttr_t luxTask_attributes;

extern osThreadId_t bmeTaskHandle;
extern const osThreadAttr_t bmeTask_attributes;

extern osThreadId_t shtTaskHandle;
extern const osThreadAttr_t shtTask_attributes;

extern osThreadId_t sgpTaskHandle;
extern const osThreadAttr_t sgpTask_attributes;

extern osThreadId_t imuTaskHandle;
extern const osThreadAttr_t imuTask_attributes;

extern osThreadId_t blinkTaskHandle;
extern const osThreadAttr_t blinkTask_attributes;

extern osThreadId_t micTaskHandle;
extern const osThreadAttr_t micTask_attributes;

extern osSemaphoreId_t messageI2C1_LockHandle;
extern osSemaphoreId_t messageI2C3_LockHandle;

extern osThreadId_t inertialTaskHandle;
extern osMessageQueueId_t inertialSensingQueueHandle;

extern osMessageQueueId_t activitySampleQueueHandle;
extern osMessageQueueId_t rotationSampleQueueHandle;
extern osMessageQueueId_t accSampleQueueHandle;
extern osMessageQueueId_t gyroSampleQueueHandle;

extern osThreadId_t interProcTaskHandle;
extern osMessageQueueId_t interProcessorMsgQueueHandle;

extern osThreadId_t msgPassingUSB_THandle;
extern osMessageQueueId_t msgPasssingUSB_QueueHandle;

extern osThreadId_t pulseTaskHandle;

extern osTimerId_t viveTimerHandle;

extern osMessageQueueId_t statusQueueHandle;

extern osSemaphoreId_t locNotifyHandle;

extern osSemaphoreId_t locCompleteHandle;

extern osSemaphoreId_t lightingLabDemoEndHandle;

extern osMessageQueueId_t packet_QueueHandle;
extern osMessageQueueId_t packetAvail_QueueHandle;

extern osThreadId_t senderTaskHandle;

struct LuxSensor{
	uint16_t sample_period;
} ;
struct GasSensor{
	uint16_t sample_period;
} ;
struct InertialSensor{
	uint16_t sample_period;
} ;
struct ColorSensor{
	uint16_t sample_period;
} ;
struct ThermopileSensor{
	uint16_t sample_period;
} ;
struct BlinkSensor{
	uint16_t sample_period;
} ;

// cannot be more than ~490 Bytes
struct SensorConfig{
	uint8_t systemRunState;
	struct ThermopileSensor thermopileSensor;
	struct BlinkSensor blinkSensor;
	struct InertialSensor inertialSensor;
	struct GasSensor gasSensor;
	struct LuxSensor luxSensor;
	struct ColorSensor colorSensor;
};

extern struct SensorConfig sensorConfig;

extern void startSensorThreads(void);

#ifdef NETWORK_TEST
extern osSemaphoreId_t networkTestTaskHandle;
#endif
/* variables -----------------------------------------------*/
struct SystemStatus {
	int blinkThread :1;
	int inertialThread :1;
	int interProcThread :1;
	int frontLightsThread :1;
};

struct LogMessage {
	uint8_t status;
	uint8_t logStatus;
	uint8_t blinkEnabled;
	uint8_t tempEnabled;
	uint8_t intertialEnabled;
	uint8_t positionEnabled;

};


typedef struct SensorState {
	uint32_t sensorSystems;
	/* TODO: add additional configuration options below */
} sensorState;

extern sensorState sensor_state;

struct USB_msgPass {
	uint8_t len;
	uint8_t buf[10];
};

void controlSensors(uint8_t* data, uint16_t numPackets);

//typedef struct PacketHeaders{
//  uint16_t packetType;
//  uint16_t packetID;
//  uint32_t msFromStart;
//  uint32_t epoch;
//  uint32_t payloadLength;
//  uint32_t reserved[5];
//}PacketHeader;
//
//
//#define MAX_PAYLOAD_SIZE 	MAX_PACKET_LEN - sizeof(PacketHeader)
//typedef struct CaptivatePackets{
//  PacketHeader header;
//  uint8_t payload[MAX_PAYLOAD_SIZE]; // should be MAX_PACKET_LEN - sizeof(PacketHeader)
//}CaptivatePacket;

/* Functions Definition ------------------------------------------------------*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief Thread initialization.
 * @param  None
 * @retval None
 */

/*************************************************************
 *
 * FREERTOS WRAPPER FUNCTIONS
 *
 *************************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
