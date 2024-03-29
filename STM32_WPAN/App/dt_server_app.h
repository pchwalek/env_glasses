/**
 ******************************************************************************
 * @file    dt_server_app.h
 * @author  MCD Application Team
 * @brief   Header for dt_server_app.c module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DTS_SERVER_APP_H
#define __DTS_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "captivate_config.h"

#define BUTTON_PRESSED GPIO_PIN_RESET

/* Exported typedefs ---------------------------------------------*/
typedef GPIO_PinState BUTTON_STATE;
typedef void (*IO_RECEIVE_DATA_USER_CALLBACK_TYPE)(uint8_t *rx_data,
		uint16_t data_size);

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "app_conf.h"
//#include "master_thread.h"
#include "dts.h"
#include "packet.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//osThreadId_t DataWriteProcessId;
//osThreadId_t DataTransferProcessId;
//osThreadId_t Button_SW1_ProcessId;
//osThreadId_t Button_SW2_ProcessId;
//osThreadId_t Button_SW3_ProcessId;

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DTS_App_Init(void);
void DTS_App_KeyButtonAction(void);
void DTS_App_KeyButton2Action(void);
void DTS_App_KeyButton3Action(void);
void DTS_App_TxPoolAvailableNotification(void);

void SendData( void * argument );
//uint8_t sendCaptivatePacket_BLE(sensor_packet_t *packet);
void senderThread(void *argument);
void BLE_App_Delay_DataThroughput(void * argument);
//void SendDataBLE( struct LogPacket *sensorPacket );

typedef enum {
	DTS_APP_FLOW_OFF, DTS_APP_FLOW_ON
} DTS_App_Flow_Status_t;

typedef enum {
	DTS_APP_TRANSFER_REQ_OFF, DTS_APP_TRANSFER_REQ_ON
} DTS_App_Transfer_Req_Status_t;

typedef struct {
	DTS_STM_Payload_t TxData;
	DTS_App_Transfer_Req_Status_t NotificationTransferReq;
	DTS_App_Transfer_Req_Status_t ButtonTransferReq;
	DTS_App_Flow_Status_t DtFlowStatus;
} DTS_App_Context_t;

#ifdef __cplusplus
	}
#endif

#endif /*__DTS_SERVER_APP_H */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
