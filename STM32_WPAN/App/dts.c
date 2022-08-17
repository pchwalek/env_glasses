/**
 ******************************************************************************
 * @file    dts.c
 * @author  MCD Application Team
 * @brief   Data Transfer Service (Custom)
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


/* Includes ------------------------------------------------------------------*/
#include "ble_common.h"
#include "ble.h"
#include "dbg_trace.h"
#include "dts.h"   

#include "main.h"
#include "stdbool.h"
//#include "deca_device_api.h"

//#include "niq.h"
//#include "app_config.h"

#define UUID_128_SUPPORTED 1

#if (UUID_128_SUPPORTED == 1)
#define DT_UUID_LENGTH  UUID_TYPE_128
#else
#define DT_UUID_LENGTH  UUID_TYPE_16
#endif

#if (UUID_128_SUPPORTED == 1)
const uint8_t GENERIC_TX_CHAR_UUID[16] =
{ 0x19, 0xed, 0x82, 0xae,
  0xed, 0x21, 0x4c, 0x9d,
  0x41, 0x45, 0x22, 0x8e,
  0x81, 0xFE, 0x00, 0x00};
#else
const uint8_t DT_REQ_CHAR_UUID[2] = { 0x81, 0xFE };
#endif

#if (UUID_128_SUPPORTED == 1)
const uint8_t GENERIC_RX_CHAR_UUID[16] =
{ 0x19, 0xed, 0x82, 0xae,
  0xed, 0x21, 0x4c, 0x9d,
  0x41, 0x45, 0x22, 0x8e,
  0x82, 0xFE, 0x00, 0x00};
#else
const uint8_t DT_REQ_CHAR2_UUID[2] = { 0x82, 0xFE };
#endif

#if (UUID_128_SUPPORTED == 1)
const uint8_t PPG_CHAR_UUID[16] =
{ 0x19, 0xed, 0x82, 0xae,
  0xed, 0x21, 0x4c, 0x9d,
  0x41, 0x45, 0x22, 0x8e,
  0x83, 0xFE, 0x00, 0x00};
#else
const uint8_t DT_REQ_CHAR3_UUID[2] = { 0x83, 0xFE };
#endif

#if (UUID_128_SUPPORTED == 1)
const uint8_t GENERIC_REQ_SERV_UUID[16] =
{ 0x19, 0xed, 0x82, 0xae,
  0xed, 0x21, 0x4c, 0x9d,
  0x41, 0x45, 0x22, 0x8e,
  0x80, 0xFE, 0x00, 0x00};

#else
const uint8_t DT_REQ_SERV_UUID[2] = { 0x80, 0xFE };
#endif

//const uint8_t DT_REQ_CHAR_UWB_UUID[16] = { 0x19, 0xed, 0x82, 0xae, 0xed, 0x21,
//		0x4c, 0x9d, 0x41, 0x45, 0x21, 0x8e, 0x84, 0xFE, 0x00, 0x00 };
//
//const uint8_t DT_REQ_CHAR_SYS_UUID[16] = { 0x19, 0xed, 0x82, 0xae, 0xed,
//		0x21, 0x4c, 0x9d, 0x41, 0x44, 0x22, 0x8e, 0x85, 0xFE, 0x00, 0x00 };
//
//const uint8_t DT_REQ_CHAR_TIME_UUID[16] = { 0x19, 0xed, 0x82, 0xae, 0xed, 0x21,
//		0x4c, 0x9d, 0x41, 0x45, 0x24, 0x8e, 0x86, 0xFE, 0x00, 0x00 };
//
//const uint8_t DT_REQ_CHAR_RX_UUID[16] = { 0x19, 0xed, 0x82, 0xae, 0xed, 0x21,
//		0x4c, 0x9d, 0x41, 0x45, 0x24, 0x8b, 0x16, 0xFE, 0x00, 0x00 };
//
//const uint8_t DT_REQ_CHAR_TX_UUID[16] = { 0x19, 0xed, 0x82, 0xae, 0xed, 0x21,
//		0x4c, 0x9d, 0x41, 0x45, 0x24, 0x8c, 0x16, 0xFE, 0x00, 0x00 };


/* Private typedef -----------------------------------------------------------*/


typedef enum
{
  DTS_STM_NOTIFICATION_MASK = (1 << 0),
  DTS_STM_INDICATION_MASK = (1 << 1),
} ClientCharConfMask_t;



/* Private defines -----------------------------------------------------------*/
#define DATA_TRANSFER_NOTIFICATION_LEN_MAX                                 (512)

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static SVCCTL_EvtAckStatus_t Generic_Event_Handler( void *pckt );
static GenericSvcContext_t aGenericContext;

//static void send_ble_data(uint8_t * buffer, uint16_t data_len);
extern uint16_t Att_Mtu_Exchanged;


/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Generic_Event_Handler( void *Event )
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt * event_pckt;
  evt_blue_aci * blue_evt;
  aci_gatt_attribute_modified_event_rp0 * attribute_modified;
  aci_att_exchange_mtu_resp_event_rp0 * exchange_mtu_resp;
  aci_gatt_write_permit_req_event_rp0 * write_permit_req ;

  DTS_STM_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *) (((hci_uart_pckt*) Event)->data);
  APP_DBG_MSG("DTS_Event_Handler\n");
  switch (event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*) event_pckt->data;

      switch (blue_evt->ecode)
      {
        case EVT_BLUE_ATT_EXCHANGE_MTU_RESP:
//        	enableBlueLED_PWM(true, 150);
          APP_DBG_MSG("EVT_BLUE_ATT_EXCHANGE_MTU_RESP \n");
          exchange_mtu_resp = (aci_att_exchange_mtu_resp_event_rp0 *)blue_evt->data;
          APP_DBG_MSG("MTU_size = %d \n",exchange_mtu_resp->Server_RX_MTU );
          Att_Mtu_Exchanged = exchange_mtu_resp->Server_RX_MTU;
//          enableGreenLED_PWM(true, 150);

          break;
        /* server */
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
        	attribute_modified =
        						(aci_gatt_attribute_modified_event_rp0*) blue_evt->data;
//        				if (attribute_modified->Attr_Handle
//        						== (aGenericContext.DataTransferTxCharHdle + 2)) {
//        					/**
//        					 * Notify to application to start measurement
//        					 */
//        					if (attribute_modified->Attr_Data[0]
//        							& DTS_STM_NOTIFICATION_MASK) {
//        						APP_DBG_MSG("notification enabled\n");
//        						Notification.Evt_Opcode = DTS_STM__NOTIFICATION_ENABLED;
//        						Generic_Notification(&Notification);
//        					} else {
//        						APP_DBG_MSG("notification disabled\n");
//        						Notification.Evt_Opcode = DTS_STM_NOTIFICATION_DISABLED;
//        						Generic_Notification(&Notification);
//        					}
//        				}
//
//        				if (attribute_modified->Attr_Handle
//        						== (aGenericContext.DataTransferRxCharHdle + 2)) {
//        					return_value = SVCCTL_EvtAckFlowEnable;
//
//        					Notification.Evt_Opcode = DTS_STM_DATA_RECEIVED;
//        					Notification.DataTransfered.Length =
//        							attribute_modified->Attr_Data_Length;
//        					Generic_Notification(&Notification);
//        				}

//        				// if UWB info was requested
//        				if (attribute_modified->Attr_Handle
//        						== (aGenericontext.DataTransferUWBConfigHdle + 1)) {
//        	#ifdef NUCLEO_LED_ACTIVE
//        	        	  	  	  	  BSP_LED_Toggle(LED_BLUE);
//        	#endif
////        					memcpy(&receivedColor, attribute_modified->Attr_Data,
////        							sizeof(receivedColor));
////        					//FrontLightsSet(&receivedColor);
////        					osMessageQueuePut(lightsComplexQueueHandle, &receivedColor, 0, 0);
//
//        				}
//
//        				// if system config was modified
//        				if (attribute_modified->Attr_Handle
//        						== (aGenericontext.DataTransferRxHdle + 1)) {
//        	#ifdef NUCLEO_LED_ACTIVE
//        	        	  BSP_LED_Toggle(LED_GREEN);
//        	#endif
//
////        					osMessageQueuePut(togLoggingQueueHandle, &receivedCntrlPacket,
////        							0U, 0U);
//
//        				}
//
//        				// if time was set
//        				if (attribute_modified->Attr_Handle
//        						== (aGenericontext.DataTransferTimeHdle + 1)) {
//        	#ifdef NUCLEO_LED_ACTIVE
//        							  BSP_LED_Toggle(LED_GREEN);
//        	#endif
////        					memcpy(&receivedEpoch, attribute_modified->Attr_Data,
////        							sizeof(receivedEpoch));
//
////        					receivedEpoch = receivedEpoch / 1000;
////        					updateRTC(receivedEpoch);
//
//        				}
        				break;
        			}

        case EVT_BLUE_GATT_TX_POOL_AVAILABLE:
//          Resume_Notification();
          break; 
          
      case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
        APP_DBG_MSG("write permit req\r\n");
        write_permit_req = (aci_gatt_write_permit_req_event_rp0 *) blue_evt->data;
        aci_gatt_write_resp( write_permit_req->Connection_Handle, write_permit_req->Attribute_Handle, 0, 0, write_permit_req->Data_Length, write_permit_req->Data);        
        break;

        default:
          break;
      }
    }
      break; /* HCI_EVT_VENDOR_SPECIFIC */

    default:
      break;
  }

  return (return_value);
}/* end SVCCTL_EvtAckStatus_t */



/**
 * @brief  Feature Characteristic update
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
# define MAX_PACKET_LENGTH	243 // https://www.compel.ru/wordpress/wp-content/uploads/2019/12/en.dm00598033.pdf

static tBleStatus TX_Update_Char(DTS_STM_Payload_t *pDataValue) {
	tBleStatus ret;

	/**
	 *  Notification Data Transfer Packet
	 */
	if(pDataValue->Length <= MAX_PACKET_LENGTH){
	  ret = aci_gatt_update_char_value(aGenericContext.GenericSvcHdle,
	  		aGenericContext.GenericTxCharHdle, 0, /* charValOffset */
			  pDataValue->Length, /* charValueLen */
			  (uint8_t*) pDataValue->pPayload);
	}
	else if(pDataValue->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){

	    uint16_t packetLen = pDataValue->Length;
	    uint16_t offset = 0;

	    while(packetLen > MAX_PACKET_LENGTH){
	      aci_gatt_update_char_value_ext (0,
				       aGenericContext.GenericSvcHdle,
							 aGenericContext.GenericTxCharHdle,
					0x00, //dont notify
					pDataValue->Length,
					offset,
					MAX_PACKET_LENGTH,
					((uint8_t*) pDataValue->pPayload) + offset);
	      offset += MAX_PACKET_LENGTH;
	      packetLen -= MAX_PACKET_LENGTH;
	    }

	    ret = aci_gatt_update_char_value_ext (0,
	    		aGenericContext.GenericSvcHdle,
					aGenericContext.GenericTxCharHdle,
	    	0x01, //notify
		pDataValue->Length,
		offset,
		packetLen,
	    	((uint8_t*) pDataValue->pPayload) + offset);
	}

	return ret;
}/* end TX_Update_Char() */

static tBleStatus PPG_Update_Char(DTS_STM_Payload_t *pDataValue) {
	tBleStatus ret;

	/**
	 *  Notification Data Transfer Packet
	 */
	if(pDataValue->Length <= MAX_PACKET_LENGTH){
	  ret = aci_gatt_update_char_value(aGenericContext.GenericSvcHdle,
	  		aGenericContext.PPG_CharHdle, 0, /* charValOffset */
			  pDataValue->Length, /* charValueLen */
			  (uint8_t*) pDataValue->pPayload);
	}
	else if(pDataValue->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){

	    uint16_t packetLen = pDataValue->Length;
	    uint16_t offset = 0;

	    while(packetLen > MAX_PACKET_LENGTH){
	      aci_gatt_update_char_value_ext (0,
				       aGenericContext.GenericSvcHdle,
							 aGenericContext.PPG_CharHdle,
					0x00, //dont notify
					pDataValue->Length,
					offset,
					MAX_PACKET_LENGTH,
					((uint8_t*) pDataValue->pPayload) + offset);
	      offset += MAX_PACKET_LENGTH;
	      packetLen -= MAX_PACKET_LENGTH;
	    }

	    ret = aci_gatt_update_char_value_ext (0,
	    		aGenericContext.GenericSvcHdle,
					aGenericContext.GenericTxCharHdle,
	    	0x01, //notify
		pDataValue->Length,
		offset,
		packetLen,
	    	((uint8_t*) pDataValue->pPayload) + offset);
	}

	return ret;
}/* end PPG_Update_Char() */
/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void Generic_STM_Init( void )
{
  tBleStatus hciCmdResult = BLE_STATUS_FAILED;

  /**
   *	Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Generic_Event_Handler);

  /* DT service and characteristics */
  hciCmdResult = aci_gatt_add_service(DT_UUID_LENGTH, (Service_UUID_t *) GENERIC_REQ_SERV_UUID,
  PRIMARY_SERVICE,
                                      1+3*10, &(aGenericContext.GenericSvcHdle));
  if (hciCmdResult != 0)
  {
    APP_DBG_MSG("error add service 0x%x\n", hciCmdResult);
//    enableRedLED_PWM(true, 200);
  }

  /**
		 *  Add Data Transfer TX Characteristic (characteristic that is used to send data)
		 */
		hciCmdResult = aci_gatt_add_char(aGenericContext.GenericSvcHdle,
		DT_UUID_LENGTH, (Char_UUID_t*) GENERIC_TX_CHAR_UUID,
		DATA_TRANSFER_NOTIFICATION_LEN_MAX,
		CHAR_PROP_NOTIFY,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
		10, /* encryKeySize */
		1, /* isVariable */
		&(aGenericContext.GenericTxCharHdle));
		if (hciCmdResult != 0) {
			APP_DBG_MSG("error add char Tx 0x%x\n", hciCmdResult);
			enableRedLED_PWM(true, 200);
	#ifdef NUCLEO_LED_ACTIVE
			BSP_LED_On(LED_RED);
	#endif
		}

		/**
		 *  Add Data Transfer RX Characteristic (not intended to be used in the end)
		 */
		hciCmdResult = aci_gatt_add_char(aGenericContext.GenericSvcHdle,
		DT_UUID_LENGTH, (Char_UUID_t*) GENERIC_RX_CHAR_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
		CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
				10, /* encryKeySize */
				1, /* isVariable */
				&(aGenericContext.GenericRxCharHdle));
		if (hciCmdResult != 0) {
			APP_DBG_MSG("error add char Tx\n");
			enableRedLED_PWM(true, 200);
	#ifdef NUCLEO_LED_ACTIVE
	    BSP_LED_On(LED_RED);
	#endif
		}

//		/**
//		 *  Add Data Transfer RX Characteristic (not intended to be used in the end)
//		 */
//		hciCmdResult = aci_gatt_add_char(aGenericContext.GenericSvcHdle,
//		DT_UUID_LENGTH, (Char_UUID_t*) PPG_CHAR_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
//		CHAR_PROP_NOTIFY,
//		ATTR_PERMISSION_NONE,
//		GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
//				10, /* encryKeySize */
//				1, /* isVariable */
//				&(aGenericContext.PPG_CharHdle));
//		if (hciCmdResult != 0) {
//			APP_DBG_MSG("error add char Tx\n");
//			enableRedLED_PWM(true, 150);
//
//	#ifdef NUCLEO_LED_ACTIVE
//	    BSP_LED_On(LED_RED);
//	#endif
//		}

  return;
}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * 
 */
tBleStatus Generic_STM_UpdateChar( uint16_t UUID , uint8_t *pPayload )
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;
  switch (UUID)
  {
//  	case PPG_CHAR_UUID_DEF:
//			result = TX_Update_Char((DTS_STM_Payload_t*) pPayload);
//			break;
    case DATA_TRANSFER_TX_CHAR_UUID:
      result = TX_Update_Char((DTS_STM_Payload_t*) pPayload);
      break;

    default:
      break;
  }
  return result;
}/* end DTS_STM_UpdateChar() */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
