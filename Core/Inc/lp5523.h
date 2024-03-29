/**
 ******************************************************************************
 * File Name          : template.h
 * Description        : Header for Lights.
 ******************************************************************************

 *
 ******************************************************************************
 */
#ifndef _LP5523_H
#define _LP5523_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------*/
#include "stdint.h"
#include "cmsis_os.h"
//#include "captivate_config.h"
//#include "main.h"

/* typedef -----------------------------------------------------------*/

typedef union ColorComplex {
	struct colors_indiv {
		uint8_t left_front_g;
		uint8_t left_front_b;
		uint8_t left_top_b;
		uint8_t left_top_g;
		uint8_t left_side_b;
		uint8_t left_side_g;
		uint8_t left_front_r;
		uint8_t left_top_r;
		uint8_t left_side_r;

		uint8_t right_front_g;
		uint8_t right_front_b;
		uint8_t right_top_b;
		uint8_t right_top_g;
		uint8_t right_side_b;
		uint8_t right_side_g;
		uint8_t right_front_r;
		uint8_t right_top_r;
		uint8_t right_side_r;
	} colors_indiv;
	uint8_t color[18];
};

typedef union BlueGreenTransition {
	struct val {
		uint8_t start_bit;
		uint8_t blue_min_intensity;
		uint8_t blue_max_intensity;
		uint8_t green_max_intensity;
		uint8_t step_size;
		uint8_t step_duration_milliseconds;
		uint32_t green_hold_length_seconds;
		uint32_t transition_delay_seconds;
	} val;
	uint8_t array[6];
};

typedef union RedFlash {
	struct val_flash {
		uint8_t start_bit;
		uint8_t intensity;
		uint8_t delay_duration;
		uint32_t total_duration;
	} val_flash;
	uint8_t array[7];
};

/* defines -----------------------------------------------------------*/

/** FREERTOS QUEUES **/
#define MAX_LIGHT_SIMPLE_QUEUE_SIZE	3 // 5 entries
#define NUM_OF_LEDS_PER_DEVICE		9

/** Default Address **/
#define LIS3DH_DEFAULT_ADDRESS 		0x35

/** LEFT SIDE **/
#define LIS3DH_LEFT_ADDRESS 		0x34
#define LED_LEFT_FRONT_B			0
#define LED_LEFT_FRONT_G			1
#define LED_LEFT_TOP_B				2
#define LED_LEFT_TOP_G				3
#define LED_LEFT_SIDE_B				4
#define LED_LEFT_SIDE_G				5
#define LED_LEFT_FRONT_R			6
#define LED_LEFT_TOP_R				7
#define LED_LEFT_SIDE_R				8

/** RIGHT SIDE **/
#define LIS3DH_RIGHT_ADDRESS 		0x35
#define LED_RIGHT_FRONT_B			0
#define LED_RIGHT_FRONT_G			1
#define LED_RIGHT_TOP_B				2
#define LED_RIGHT_TOP_G				3
#define LED_RIGHT_SIDE_B			4
#define LED_RIGHT_SIDE_G			5
#define LED_RIGHT_FRONT_R			6
#define LED_RIGHT_TOP_R				7
#define LED_RIGHT_SIDE_R			8

/* LIS3DH SPECIFIC REGISTERS */
#define LIS3DH_EN_CNTRL1_REG		0x00
#define 	LP5525_CHIP_EN			0x40
#define LIS3DH_ENGINE_CTRL1_REG		0x01
#define LIS3DH_DIRECT_RAT_MSB_REG	0x02
#define LIS3DH_DIRECT_RAT_LSB_REG	0x03
#define LIS3DH_CONTRL_MSB_REG		0x04
#define LIS3DH_CONTRL_LSB_REG		0x05

#define LIS3DH_D1_CNTRL_REG			0x06
#define LIS3DH_D2_CNTRL_REG			0x07
#define LIS3DH_D3_CNTRL_REG			0x08
#define LIS3DH_D4_CNTRL_REG			0x09
#define LIS3DH_D5_CNTRL_REG			0x0A
#define LIS3DH_D6_CNTRL_REG			0x0B
#define LIS3DH_D7_CNTRL_REG			0x0C
#define LIS3DH_D8_CNTRL_REG			0x0D
#define LIS3DH_D9_CNTRL_REG			0x0E
#define 	LOG_EN						0x20
#define 	LIN_EN						0x00

#define LIS3DH_D1_PWM_REG			0x16
#define LIS3DH_D2_PWM_REG			0x17
#define LIS3DH_D3_PWM_REG			0x18
#define LIS3DH_D4_PWM_REG			0x19
#define LIS3DH_D5_PWM_REG			0x1A
#define LIS3DH_D6_PWM_REG			0x1B
#define LIS3DH_D7_PWM_REG			0x1C
#define LIS3DH_D8_PWM_REG			0x1D
#define LIS3DH_D9_PWM_REG			0x1E

#define LIS3DH_D1_CURRENT_CTRL_REG	0x26
#define LIS3DH_D2_CURRENT_CTRL_REG	0x27
#define LIS3DH_D3_CURRENT_CTRL_REG	0x28
#define LIS3DH_D4_CURRENT_CTRL_REG	0x29
#define LIS3DH_D5_CURRENT_CTRL_REG	0x2A
#define LIS3DH_D6_CURRENT_CTRL_REG	0x2B
#define LIS3DH_D7_CURRENT_CTRL_REG	0x2C
#define LIS3DH_D8_CURRENT_CTRL_REG	0x2D
#define LIS3DH_D9_CURRENT_CTRL_REG	0x2E

#define LIS3DH_MISC_REG				0x36
#define 	CP_MODE_AUTO				0x0C
#define 	EN_AUTO_INC					0x40
#define 	POWERSAVE_EN				0x20
#define 	INT_CLK_EN					0x03

#define LED_START_SEQ_INTERVAL		20//mseconds
//#define LED_START_SEQ_INTERVAL		800 //mseconds


#ifdef DONGLE_CODE
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_4
#define LED1_GPIO_PORT                          GPIOA
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOA_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_0
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_1
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)         do { if ((__INDEX__) == LED1) LED1_GPIO_CLK_ENABLE(); else \
                                                if ((__INDEX__) == LED2) LED2_GPIO_CLK_ENABLE(); else \
                                                if ((__INDEX__) == LED3) LED3_GPIO_CLK_ENABLE();} while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)         do { if ((__INDEX__) == LED1) LED1_GPIO_CLK_DISABLE(); else \
                                                if ((__INDEX__) == LED2) LED2_GPIO_CLK_DISABLE(); else \
                                                if ((__INDEX__) == LED3) LED3_GPIO_CLK_DISABLE();} while(0)
#endif

/* typedefs ------------------------------------------------------------*/
#ifdef DONGLE_CODE
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  /* Color led aliases */
  LED_BLUE   = LED1,
  LED_GREEN  = LED2,
  LED_RED    = LED3
}Led_TypeDef;
#endif

/* function prototypes -----------------------------------------------*/
#ifdef DONGLE_CODE
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
#endif

/* variables -----------------------------------------------*/

/* variables that hold received messages */
//uint32_t lightsSimpleMessage;

/* Functions Definition ------------------------------------------------------*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
void ThreadFrontLightsTask(void *argument);
void ThreadFrontLightsComplexTask(void *argument);


void setup_LP5523(uint8_t ADDR);

void FrontLightsSet(union ColorComplex *setColors);

//void ledDisconnectNotification(void);
void ledDisconnectNotification(void *argument);
//void ledConnectNotification(void);
void ledStartupSequence(void);
void ledAllRed(void);
void resetColor(union ColorComplex * colorComplex);
void ledEnterDFUNotification(void);
void BlueGreenTransitionTask(void *argument);
void RedFlashTask(void *argument);
void resetLED(void);
void BlueGreenTransitionTaskExit(void *argument);

void ledCheck(void *argument);
void colorTransitionDemo(void *argument);
void shiftLED(union ColorComplex * colorComplex, uint8_t val);
//class LP5523
//{
//public:
//  LP5523(uint16_t DevAddress = LIS3DH_DEFAULT_ADDRESS);
//
//  bool begin(void);
//
////  uint8_t getDeviceID();
////  bool haveNewData();
////
////  void read();
////  int16_t readADC(uint8_t a);
////
////  void setRange(lis3dh_range_t range);
////  lis3dh_range_t getRange(void);
////
////  void setDataRate(lis3dh_dataRate_t dataRate);
////  lis3dh_dataRate_t getDataRate(void);
////
////  bool getEvent(sensors_event_t *event);
////  void getSensor(sensor_t *sensor);
////
////  void setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit = 10,
////                uint8_t timelatency = 20, uint8_t timewindow = 255);
////  uint8_t getClick(void);
////
////  int16_t x; /**< x axis value */
////  int16_t y; /**< y axis value */
////  int16_t z; /**< z axis value */
////
////  float x_g; /**< x_g axis value (calculated by selected range) */
////  float y_g; /**< y_g axis value (calculated by selected range) */
////  float z_g; /**< z_g axis value (calculated by selected scale) */
//
//  uint16_t deviceAddress;
//
////protected:
////  uint8_t spixfer(uint8_t x = 0xFF);
////  void writeRegister8(uint8_t reg, uint8_t value);
////  uint8_t readRegister8(uint8_t reg);
////
////private:
////  TwoWire *I2Cinterface;
////  SPIClass *SPIinterface;
////
////  uint8_t _wai;
////
////  int8_t _cs, _mosi, _miso, _sck;
////
////  int8_t _i2caddr;
////
////  int32_t _sensorID;
//};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
