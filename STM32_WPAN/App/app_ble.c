/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/app_ble.c
 * Description        : Application file for BLE Middleware.
 *
 *****************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "cmsis_os.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "dis_app.h"
//#include "hrs_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dt_client_app.h"
#include "dt_server_app.h"
#include "dts.h"
#include "lp5523.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];

}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

  /**
   * ID of the Advertising Timeout
   */
  uint8_t Advertising_mgr_timer_Id;

}BleApplicationContext_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */
char hexToAscii(uint8_t val);

char name[18];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef CUSTOM_BT_PARAMETERS
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;

static const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME , 'S', 'T', 'M', '3', '2', 'W', 'B'};
uint8_t  manuf_data[14] = {
    sizeof(manuf_data)-1, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0x01/*SKD version */,
    0x00 /* Generic*/,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */

};

/* USER CODE BEGIN PV */
#else
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] = {
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40) };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
 *   Identity root key used to derive LTK and CSRK
 */
static const uint8_t BLE_CFG_IR_VALUE[16] =
				CFG_BLE_IRK;

				/**
				 * Encryption root key used to derive LTK and CSRK
				 */
				static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;

				/**
				 * These are the two tags used to manage a power failure during OTA
				 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
				 * The MagicKeywordvalue is checked in the ble_ota application
				 */
				PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29;
				PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

				PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
				PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin,
AdvIntervalMax;


//static const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'A','i','r','S','p','e','c' };
uint8_t manuf_data[14] = { sizeof(manuf_data) - 1,
AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x01/*SKD version */, 0x00 /* Generic*/,
		0x00 /* GROUP A Feature  */, 0x00 /* GROUP A Feature */,
		0x00 /* GROUP B Feature */, 0x00 /* GROUP B Feature */, 0x00, /* BLE MAC start -MSB */
		0x00, 0x00, 0x00, 0x00, 0x00, /* BLE MAC stop */

};
#endif
osThreadId_t LinkConfigProcessId;
osThreadId_t AdvCancelProcessId;
osThreadId_t AdvReqProcessId;


/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/
osMutexId_t MtxHciId;
osSemaphoreId_t SemHciId;
osThreadId_t AdvUpdateProcessId;
osThreadId_t HciUserEvtProcessId;

const osThreadAttr_t AdvUpdateProcess_attr = {
    .name = CFG_ADV_UPDATE_PROCESS_NAME,
    .attr_bits = CFG_ADV_UPDATE_PROCESS_ATTR_BITS,
    .cb_mem = CFG_ADV_UPDATE_PROCESS_CB_MEM,
    .cb_size = CFG_ADV_UPDATE_PROCESS_CB_SIZE,
    .stack_mem = CFG_ADV_UPDATE_PROCESS_STACK_MEM,
    .priority = CFG_ADV_UPDATE_PROCESS_PRIORITY,
    .stack_size = CFG_ADV_UPDATE_PROCESS_STACK_SIZE
};

const osThreadAttr_t HciUserEvtProcess_attr = {
    .name = CFG_HCI_USER_EVT_PROCESS_NAME,
    .attr_bits = CFG_HCI_USER_EVT_PROCESS_ATTR_BITS,
    .cb_mem = CFG_HCI_USER_EVT_PROCESS_CB_MEM,
    .cb_size = CFG_HCI_USER_EVT_PROCESS_CB_SIZE,
    .stack_mem = CFG_HCI_USER_EVT_PROCESS_STACK_MEM,
    .priority = CFG_HCI_USER_EVT_PROCESS_PRIORITY,
    .stack_size = CFG_HCI_USER_EVT_PROCESS_STACK_SIZE
};

/* Private function prototypes -----------------------------------------------*/
static void HciUserEvtProcess(void *argument);
static void BLE_UserEvtRx( void * pPayload );
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress( void );
static void Adv_Request( APP_BLE_ConnStatus_t New_Status );
static void Add_Advertisment_Service_UUID( uint16_t servUUID );
//static void Add_Advertisment_Service_UUID_128( uint8_t* servUUID );
static void Adv_Mgr( void );
static void AdvUpdateProcess(void *argument);
static void Adv_Update( void );

/* USER CODE BEGIN PFP */
const osThreadAttr_t LinkConfigProcess_attr = { .name =
CFG_TP_LINK_CONFIG_PROCESS_NAME, .attr_bits =
CFG_TP_GENERIC_PROCESS_ATTR_BITS, .cb_mem =
CFG_TP_GENERIC_PROCESS_CB_MEM, .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
		.stack_mem =
		CFG_TP_GENERIC_PROCESS_STACK_MEM, .priority =
		CFG_TP_GENERIC_PROCESS_PRIORITY, .stack_size =
		CFG_TP_GENERIC_PROCESS_STACK_SIZE * 2 };
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init( void )
{
/* USER CODE BEGIN APP_BLE_Init_1 */
//#ifndef CUSTOM_BT_PARAMETERS
/* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_SLAVE_SCA,
     CFG_BLE_MASTER_SCA,
     CFG_BLE_LSE_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG}
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  HciUserEvtProcessId = osThreadNew(HciUserEvtProcess, NULL, &HciUserEvtProcess_attr);

  /**
   * Starts the BLE Stack on CPU2
   */
  if (SHCI_C2_BLE_Init( &ble_init_cmd_packet ) != SHCI_Success)
  {
    Error_Handler();
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
  /**
   * From here, all initialization are BLE application specific
   */
  AdvUpdateProcessId = osThreadNew(AdvUpdateProcess, NULL, &AdvUpdateProcess_attr);

  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
  manuf_data[sizeof(manuf_data)-8] = CFG_FEATURE_OTA_REBOOT;
#endif


/* USER CODE BEGIN APP_BLE_Init_3 */


/* USER CODE END APP_BLE_Init_3 */

  /**
   * Create timer to handle the connection state machine
   */

  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;

//  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_128_BIT_SERV_UUID;
//  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;

    Add_Advertisment_Service_UUID(DATA_TRANSFER_SERVICE_UUID);
//  Add_Advertisment_Service_UUID_128(DT_REQ_CHAR_UUID);


  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
  * Start to Advertise to be connected by Collector
   */
   Adv_Request(APP_BLE_FAST_ADV);

/* USER CODE BEGIN APP_BLE_Init_2 */
  startInitThread();

/* USER CODE END APP_BLE_Init_2 */
  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  evt_blecore_aci *blecore_evt;
  hci_le_phy_update_complete_event_rp0 *evt_le_phy_update_complete;
  uint8_t TX_PHY, RX_PHY;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;

  /* USER CODE BEGIN SVCCTL_App_Notification */
//#ifndef CUSTOM_BT_PARAMETERS
  /* USER CODE END SVCCTL_App_Notification */

  switch (event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      hci_disconnection_complete_event_rp0 *disconnection_complete_event;
      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) event_pckt->data;

      if (disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

        APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH CLIENT \n");
      }

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);


      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */
  		/* restart advertising */
//		osThreadTerminate(blueGreenTranTaskHandle); // terminate any existing running thread

//      osThreadState_t threadState = osThreadGetState(blueGreenTranTaskHandle);
//		if((threadState == osThreadReady) || (threadState == osThreadRunning) || (threadState == osThreadBlocked)){
//			osThreadTerminate(blueGreenTranTaskHandle); // terminate any existing running thread
//
//	//							osThreadId_t blueGreenExitTaskHandle;
//	//							const osThreadAttr_t blueGreenExitTask_attributes
//			blueGreenExitTaskHandle = osThreadNew(BlueGreenTransitionTaskExit, NULL, &blueGreenExitTask_attributes);
//
//	//							BlueGreenTransitionTaskExit();
//		}



//      osThreadTerminate(redFlashTaskHandle); // terminate any existing running thread

      osThreadFlagsSet(ledDisconnectTaskHandle, DISCONNECT_BLE_BIT);

//      HAL_Delay(4000);
//      NVIC_SystemReset();


#ifndef DYNAMIC_MODE
  		Adv_Request(APP_BLE_FAST_ADV);
  #else
		osThreadFlagsSet(AdvUpdateProcessId, 1);
		//		Adv_Request(APP_BLE_LP_ADV);
#endif

      /* USER CODE END EVT_DISCONN_COMPLETE */
    }

    break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
    {
      meta_evt = (evt_le_meta_event*) event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("\r\n\r** CONNECTION UPDATE EVENT WITH CLIENT \n");

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;
        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE \n");
          evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)meta_evt->data;
          if (evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE, status ok \n");
          }
          else
          {
            APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE, status nok \n");
          }

          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,&TX_PHY,&RX_PHY);
          if (ret == BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("Read_PHY success \n");

            if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
            {
              APP_DBG_MSG("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
            }
            else
            {
              APP_DBG_MSG("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
            }
          }
          else
          {
            APP_DBG_MSG("Read conf not succeess \n");
          }
          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_complete_event_rp0 *connection_complete_event;

          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;

          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n", connection_complete_event->Connection_Handle);
          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */
//          ledConnectNotification();
          osThreadFlagsSet(ledDisconnectTaskHandle, CONNECT_BLE_BIT);

//          mutex = 1;
          BLE_SVC_L2CAP_Conn_Update_7_5();
          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
        }
        break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

        /* USER CODE BEGIN META_EVT */

        /* USER CODE END META_EVT */

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }
    }
    break; /* HCI_LE_META_EVT_CODE */

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blecore_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */
        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
        APP_DBG_MSG("\r\n\r** ACI_GAP_PROC_COMPLETE_VSEVT_CODE \n");
        /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

        /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */

      /* USER CODE BEGIN BLUE_EVT */

      /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

      default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
    return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/

void bluetoothStartAdvertising(void){
	  /**
	  * Start to Advertise to be connected by Collector
	   */
	   Adv_Request(APP_BLE_FAST_ADV);
}

uint8_t isBluetoothConnected(void){
	if((BleApplicationContext.Device_Connection_Status == APP_BLE_CONNECTED_CLIENT) ||
			(BleApplicationContext.Device_Connection_Status == APP_BLE_CONNECTED_SERVER)){
		return 1;
	}else{
		return 0;
	}
}


uint8_t APP_BLE_ComputeCRC8(uint8_t *DataPtr, uint8_t Datalen) {
	uint8_t i, j;
	const uint8_t PolynomeCRC = 0x97;
	uint8_t CRC8 = 0x00;

	for (i = 0; i < Datalen; i++) {
		CRC8 ^= DataPtr[i];
		for (j = 0; j < 8; j++) {
			if ((CRC8 & 0x80) != 0) {
				CRC8 = (uint8_t) ((CRC8 << 1) ^ PolynomeCRC);
			} else {
				CRC8 <<= 1;
			}
		}
	}
	return (CRC8);
}

uint8_t index_con_int, mutex;
void BLE_SVC_L2CAP_Conn_Update_7_5(void) {
	/* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_1 */

	/* USER CODE END BLE_SVC_L2CAP_Conn_Update_1 */
//	if (mutex == 1) {
//		mutex = 0;
		uint16_t interval_min = CONN_P(7.5);
		uint16_t interval_max = CONN_P(7.5);
		uint16_t slave_latency = L2CAP_SLAVE_LATENCY;
		uint16_t timeout_multiplier = L2CAP_TIMEOUT_MULTIPLIER;
		tBleStatus result;

		result = aci_l2cap_connection_parameter_update_req(
				BleApplicationContext.BleApplicationContext_legacy.connectionHandle,
				interval_min, interval_max, slave_latency, timeout_multiplier);
		if (result == BLE_STATUS_SUCCESS) {
#if(CFG_DEBUG_APP_TRACE != 0)
			APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Successfully \r\n\r");
#endif
		} else {
#if(CFG_DEBUG_APP_TRACE != 0)
			APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Failed \r\n\r");
#endif
		}
//	}
	/* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_2 */

	/* USER CODE END BLE_SVC_L2CAP_Conn_Update_2 */
	return;
}

void LinkConfiguration(void *argument) {
	UNUSED(argument);
	tBleStatus status;

	while (1) {
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

#if (CFG_BLE_CENTRAL != 0)
		uint8_t tx_phy;
		uint8_t rx_phy;
#endif

		/**
		 * The client will start ATT configuration after the link is fully configured
		 * Setup PHY
		 * Setup Data Length
		 * Setup Pairing
		 */
#if (((CFG_TX_PHY == 2) || (CFG_RX_PHY == 2)) && (CFG_BLE_CENTRAL != 0))
		GapProcReq(GAP_PROC_SET_PHY);
#endif

#if (CFG_BLE_CENTRAL != 0)
		APP_DBG_MSG("Reading_PHY\n");
		status = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,&tx_phy,&rx_phy);
		if (status != BLE_STATUS_SUCCESS)
		{
			APP_DBG_MSG("Read phy cmd failure: 0x%x \n", status);
		}
		else
		{
			APP_DBG_MSG("TX PHY = %d\n", tx_phy);
			APP_DBG_MSG("RX PHY = %d\n", rx_phy);
		}
#endif

		APP_DBG_MSG("set data length \n");
//		  BSP_LED_On(LED_BLUE);
		status = hci_le_set_data_length(
				BleApplicationContext.BleApplicationContext_legacy.connectionHandle,
				251, 2120);
		if (status != BLE_STATUS_SUCCESS) {
//				  BSP_LED_On(LED_RED);
			APP_DBG_MSG("set data length command error \n");
		}

#if ((CFG_ENCRYPTION_ENABLE != 0) && (CFG_BLE_CENTRAL != 0))
		GapProcReq(GAP_PROC_PAIRING);
#endif

		DTC_App_LinkReadyNotification(
				BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
	}
}

/* USER CODE END FD*/
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  MtxHciId = osMutexNew( NULL );
  SemHciId = osSemaphoreNew( 1, 0, NULL ); /*< Create the semaphore and make it busy at initialization */

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE };

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);

#if (CFG_BLE_ADDRESS_TYPE == PUBLIC_ADDR)
  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = bd_addr[5];
  manuf_data[ sizeof(manuf_data)-5] = bd_addr[4];
  manuf_data[ sizeof(manuf_data)-4] = bd_addr[3];
  manuf_data[ sizeof(manuf_data)-3] = bd_addr[2];
  manuf_data[ sizeof(manuf_data)-2] = bd_addr[1];
  manuf_data[ sizeof(manuf_data)-1] = bd_addr[0];
#endif

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET,
    CONFIG_DATA_IR_LEN,
                            (uint8_t*) BLE_CFG_IR_VALUE);

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET,
                              CONFIG_DATA_ER_LEN,
                              (uint8_t*) BLE_CFG_ER_VALUE);

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#elif (CFG_BLE_ADDRESS_TYPE == RANDOM_ADDR)
  /* Get RNG semaphore */
  while( LL_HSEM_1StepLock( HSEM, CFG_HW_RNG_SEMID ) );

  /* Enable RNG */
  __HAL_RNG_ENABLE(&hrng);

  /* Enable HSI48 oscillator */
  LL_RCC_HSI48_Enable();
  /* Wait until HSI48 is ready */
  while( ! LL_RCC_HSI48_IsReady( ) );

  if (HAL_RNG_GenerateRandomNumber(&hrng, &srd_bd_addr[1]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  if (HAL_RNG_GenerateRandomNumber(&hrng, &srd_bd_addr[0]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */

  /* Disable HSI48 oscillator */
  LL_RCC_HSI48_Disable();

  /* Disable RNG */
  __HAL_RNG_DISABLE(&hrng);

  /* Release RNG semaphore */
  LL_HSEM_ReleaseLock( HSEM, CFG_HW_RNG_SEMID, 0 );
#endif

#if (CFG_BLE_ADDRESS_TYPE == STATIC_RANDOM_ADDR)
  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = srd_bd_addr[1] >> 8 ;
  manuf_data[ sizeof(manuf_data)-5] = srd_bd_addr[1];
  manuf_data[ sizeof(manuf_data)-4] = srd_bd_addr[0] >> 24;
  manuf_data[ sizeof(manuf_data)-3] = srd_bd_addr[0] >> 16;
  manuf_data[ sizeof(manuf_data)-2] = srd_bd_addr[0] >> 8;
  manuf_data[ sizeof(manuf_data)-1] = srd_bd_addr[0];

  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );
#endif

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
  aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  uint32_t UID = LL_FLASH_GetUDN();

  if (role > 0)
  {
//    const char *name = "STM32WB";
//	  const char *name2 = "AIRSPEC_10201213";
	const char nameTemp[] = {'A','i','r','S','p','e','c','_',
			  hexToAscii(UID >> 28),
			  hexToAscii(UID >> 24),
			  hexToAscii(UID >> 20),
			  hexToAscii(UID >> 16),
			  hexToAscii(UID >> 12),
			  hexToAscii(UID >> 8),
			  hexToAscii(UID >> 4),
			  hexToAscii(UID)};

	strcpy(name,nameTemp);
	name[16] = 0;
	name[17] = 0;
//	tBleStatus state;
    aci_gap_init(role,
#if ((CFG_BLE_ADDRESS_TYPE == RESOLVABLE_PRIVATE_ADDR) || (CFG_BLE_ADDRESS_TYPE == NON_RESOLVABLE_PRIVATE_ADDR))
                 2,
#else
                 0,
#endif
//                 APPBLE_GAP_DEVICE_NAME_LENGTH,
				 16,
                 &gap_service_handle,
                 &gap_dev_name_char_handle,
                 &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }
  /**
   * Initialize Default PHY
   */
  hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                         CFG_SC_SUPPORT,
                                         CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         CFG_BLE_ADDRESS_TYPE
                                         );

  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }
}

char hexToAscii(uint8_t val){
	// only look at first 4 bits
	val = val & (0x0F);
	if(val<10) return val+48;
	else return val+87;
}

static void Adv_Request(APP_BLE_ConnStatus_t New_Status)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

//  if (New_Status == APP_BLE_FAST_ADV)
//  {
//    Min_Inter = AdvIntervalMin;
//    Max_Inter = AdvIntervalMax;
//  }
//  else
//  {
//    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
//    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
//  }
//
  Min_Inter = AdvIntervalMin;
  Max_Inter = AdvIntervalMax;

  uint32_t UID = LL_FLASH_GetUDN();

  const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'A','i','r','S','p','e','c','_',
		  hexToAscii(UID >> 28),
		  hexToAscii(UID >> 24),
		  hexToAscii(UID >> 20),
		  hexToAscii(UID >> 16),
		  hexToAscii(UID >> 12),
		  hexToAscii(UID >> 8),
		  hexToAscii(UID >> 4),
		  hexToAscii(UID)};

    /**
     * Stop the timer, it will be restarted for a new shot
     * It does not hurt if the timer was not running
     */
    HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

    APP_DBG_MSG("First index in %d state \n", BleApplicationContext.Device_Connection_Status);

    if ((New_Status == APP_BLE_LP_ADV)
        && ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
            || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
    {
      /* Connection in ADVERTISE mode have to stop the current advertising */
      ret = aci_gap_set_non_discoverable();
      if (ret == BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("Successfully Stopped Advertising \n");
      }
      else
      {
        APP_DBG_MSG("Stop Advertising Failed , result: %d \n", ret);
      }
    }

    BleApplicationContext.Device_Connection_Status = New_Status;
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_discoverable(
        ADV_IND,
        Min_Inter,
        Max_Inter,
        CFG_BLE_ADDRESS_TYPE,
        NO_WHITE_LIST_USE, /* use white list */
        sizeof(local_name),
        (uint8_t*) &local_name,
        BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
        BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
        0,
        0);

    /* Update Advertising data */
    /* the below is left from example code since this could be a place to do something if
     * a client doesn't connect to the glasses within "INITIAL_ADV_TIMEOUT" time
     */
    ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);
    if (ret == BLE_STATUS_SUCCESS)
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
//        APP_DBG_MSG("Successfully Start Fast Advertising \n" );
        /* Start Timer to STOP ADV - TIMEOUT */
        HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
      }
      else
      {
//        APP_DBG_MSG("Successfully Start Low Power Advertising \n");
        HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
      }
    }
    else
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
        APP_DBG_MSG("Start Fast Advertising Failed , result: %d \n", ret);
//        NVIC_SystemReset();

      }
      else
      {
        APP_DBG_MSG("Start Low Power Advertising Failed , result: %d \n", ret);
//        NVIC_SystemReset();
      }
    }

  return;
}

const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

/**
 * Public Address with the ST company ID
 * bit[47:24] : 24bits (OUI) equal to the company ID
 * bit[23:16] : Device ID.
 * bit[15:0] : The last 16bits from the UDN
 * Note: In order to use the Public Address in a final product, a dedicated
 * 24bits company ID (OUI) shall be bought.
 */
    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)device_id;
    bd_addr_udn[3] = (uint8_t)(company_id & 0x000000FF);
    bd_addr_udn[4] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x00FF0000) >> 16 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }
  }

  return bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 *SPECIFIC FUNCTIONS
 *
 *************************************************************/
//static void Add_Advertisment_Service_UUID_128( uint8_t* servUUID )
//{
//  memcpy(BleApplicationContext.BleApplicationContext_legacy.advtServUUID+1, servUUID, 16);
////  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
////      (uint8_t) (servUUID & 0xFF);
////  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
////  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
////      (uint8_t) (servUUID >> 8) & 0xFF;
////  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
//  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen+=16;
//
//  return;
//}

static void Add_Advertisment_Service_UUID( uint16_t servUUID )
{
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID & 0xFF);
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID >> 8) & 0xFF;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;

  return;
}

static void Adv_Mgr( void )
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  osThreadFlagsSet( AdvUpdateProcessId, 1 );

  return;
}

static void AdvUpdateProcess(void *argument)
{
  UNUSED(argument);

  for(;;)
  {
    osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);
    Adv_Update( );
  }
}

static void Adv_Update( void )
{
  Adv_Request(APP_BLE_LP_ADV);

  return;
}

static void HciUserEvtProcess(void *argument)
{
  UNUSED(argument);

  for(;;)
  {
    osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);
    hci_user_evt_proc( );
  }
}

/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UNUSED(pdata);
  osThreadFlagsSet( HciUserEvtProcessId, 1 );
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UNUSED(flag);
  osSemaphoreRelease( SemHciId );
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UNUSED(timeout);
  osSemaphoreAcquire( SemHciId, osWaitForever );
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  switch (status)
  {
    case HCI_TL_CmdBusy:
      osMutexAcquire( MtxHciId, osWaitForever );
      break;

    case HCI_TL_CmdAvailable:
      osMutexRelease( MtxHciId );
      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
void SVCCTL_InitCustomSvc(void) {
	DTS_STM_Init();
}

/* USER CODE END FD_WRAP_FUNCTIONS */
