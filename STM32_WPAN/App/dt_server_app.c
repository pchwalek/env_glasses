/**
 ******************************************************************************
 * @file    dt_server_app.c
 * @author  MCD Application Team
 * @brief   data throughput server Application
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
#include "app_common.h"
#include "dbg_trace.h"

#include "ble.h"
#include "app_ble.h"

#include "dt_server_app.h"
#include "dt_client_app.h"
#include "dts.h"

//#include "stm32_seq.h"
#include "stm32_lpm.h"

#include "ble_common.h"
//#include "ble_clock.h"

#include "cmsis_os.h"






/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DTS_App_Context_t DataTransferServerContext;
static uint8_t Notification_Data_Buffer[DATA_NOTIFICATION_MAX_PACKET_SIZE]; /* DATA_NOTIFICATION_MAX_PACKET_SIZE data + CRC */
uint32_t DataReceived;

//const osThreadAttr_t DataWriteProcess_attr = {
//    .name = CFG_TP_DW_PROCESS_NAME,
//    .attr_bits = CFG_TP_GENERIC_PROCESS_ATTR_BITS,
//    .cb_mem = CFG_TP_GENERIC_PROCESS_CB_MEM,
//    .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
//    .stack_mem = CFG_TP_GENERIC_PROCESS_STACK_MEM,
//    .priority = CFG_TP_GENERIC_PROCESS_PRIORITY,
//    .stack_size = CFG_TP_GENERIC_PROCESS_STACK_SIZE
//};
//
//const osThreadAttr_t DataTransferProcess_attr = {
//    .name = CFG_TP_DT_PROCESS_NAME,
//    .attr_bits = CFG_TP_GENERIC_PROCESS_ATTR_BITS,
//    .cb_mem = CFG_TP_GENERIC_PROCESS_CB_MEM,
//    .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
//    .stack_mem = CFG_TP_GENERIC_PROCESS_STACK_MEM,
//    .priority = CFG_TP_GENERIC_PROCESS_PRIORITY,
//    .stack_size = CFG_TP_GENERIC_PROCESS_STACK_SIZE
//};
//
//const osThreadAttr_t Button_SW1_Process_attr = {
//    .name = CFG_TP_SW1_PROCESS_NAME,
//    .attr_bits = CFG_TP_GENERIC_PROCESS_ATTR_BITS,
//    .cb_mem = CFG_TP_GENERIC_PROCESS_CB_MEM,
//    .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
//    .stack_mem = CFG_TP_GENERIC_PROCESS_STACK_MEM,
//    .priority = CFG_TP_GENERIC_PROCESS_PRIORITY,
//    .stack_size = CFG_TP_GENERIC_PROCESS_STACK_SIZE
//};
//
//const osThreadAttr_t Button_SW2_Process_attr = {
//    .name = CFG_TP_SW2_PROCESS_NAME,
//    .attr_bits = CFG_TP_GENERIC_PROCESS_ATTR_BITS,
//    .cb_mem = CFG_TP_GENERIC_PROCESS_CB_MEM,
//    .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
//    .stack_mem = CFG_TP_GENERIC_PROCESS_STACK_MEM,
//    .priority = CFG_TP_GENERIC_PROCESS_PRIORITY,
//    .stack_size = CFG_TP_GENERIC_PROCESS_STACK_SIZE
//};
//
//const osThreadAttr_t Button_SW3_Process_attr = {
//    .name = CFG_TP_SW3_PROCESS_NAME,
//    .attr_bits = CFG_TP_GENERIC_PROCESS_ATTR_BITS,
//    .cb_mem = CFG_TP_GENERIC_PROCESS_CB_MEM,
//    .cb_size = CFG_TP_GENERIC_PROCESS_CB_SIZE,
//    .stack_mem = CFG_TP_GENERIC_PROCESS_STACK_MEM,
//    .priority = CFG_TP_GENERIC_PROCESS_PRIORITY,
//    .stack_size = CFG_TP_GENERIC_PROCESS_STACK_SIZE
//};

/* Global variables ----------------------------------------------------------*/

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
//static void ButtonTriggerReceived(void);
//static void DT_App_Button2_Trigger_Received( void );
//static void DT_App_Button3_Trigger_Received( void );
//static void SendData(void);
//static void BLE_App_Delay_DataThroughput( void );
//void ButtonTriggerReceived( void * argument );
//void DT_App_Button2_Trigger_Received( void * argument );
//void DT_App_Button3_Trigger_Received( void * argument );
extern uint16_t Att_Mtu_Exchanged;
extern uint8_t TimerDataThroughputWrite_Id;

#define DEFAULT_TS_MEASUREMENT_INTERVAL   (1000000/CFG_TS_TICK_VAL)  /**< 1s */
#define DELAY_1s  (1*DEFAULT_TS_MEASUREMENT_INTERVAL)
#define TIMEUNIT  1

#define BOUNCE_THRESHOLD                20U
#define LONG_PRESS_THRESHOLD            1000U


/*************************************************************
 *
 * PUBLIC FUNCTIONS
 *
 *************************************************************/
void DTS_App_Init(void)
{
  uint8_t i;

//  UTIL_SEQ_RegTask( 1<<CFG_TASK_BUTTON_ID, UTIL_SEQ_RFU, ButtonTriggerReceived);
//  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW2_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, DT_App_Button2_Trigger_Received);
//  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW3_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, DT_App_Button3_Trigger_Received);
//  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_TRANSFER_UPDATE_ID, UTIL_SEQ_RFU, SendData);
//  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_WRITE_ID, UTIL_SEQ_RFU, BLE_App_Delay_DataThroughput);

//  DataWriteProcessId= osThreadNew(BLE_App_Delay_DataThroughput, NULL, &DataWriteProcess_attr);
//  DataTransferProcessId= osThreadNew(SendData, NULL, &DataTransferProcess_attr);
//  Button_SW1_ProcessId= osThreadNew(ButtonTriggerReceived, NULL, &Button_SW1_Process_attr);
//  Button_SW2_ProcessId= osThreadNew(DT_App_Button2_Trigger_Received, NULL, &Button_SW2_Process_attr);
//  Button_SW3_ProcessId= osThreadNew(DT_App_Button3_Trigger_Received, NULL, &Button_SW3_Process_attr);


  /**
   * Initialize data buffer
   */
  for (i=0 ; i<(DATA_NOTIFICATION_MAX_PACKET_SIZE-1) ; i++)
  {
    Notification_Data_Buffer[i] = i;
  }

  DataTransferServerContext.NotificationTransferReq = DTS_APP_TRANSFER_REQ_OFF;
//  DataTransferServerContext.ButtonTransferReq = DTS_APP_TRANSFER_REQ_OFF;
  DataTransferServerContext.ButtonTransferReq = DTS_APP_TRANSFER_REQ_ON;
  DataTransferServerContext.DtFlowStatus = DTS_APP_FLOW_ON;
}

//void DTS_App_KeyButtonAction( void )
//{
////  UTIL_SEQ_SetTask(1 << CFG_TASK_BUTTON_ID, CFG_SCH_PRIO_0);
//  osThreadFlagsSet( Button_SW1_ProcessId, 1 );
//}
//
//void DTS_App_KeyButton2Action( void )
//{
////    UTIL_SEQ_SetTask(1 << CFG_TASK_SW2_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
//    osThreadFlagsSet( Button_SW2_ProcessId, 1 );
//}
//
//void DTS_App_KeyButton3Action( void )
//{
////    UTIL_SEQ_SetTask(1 << CFG_TASK_SW3_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
//	  osThreadFlagsSet( Button_SW2_ProcessId, 1 );
//}
//
//void DTS_App_TxPoolAvailableNotification(void)
//{
//  DataTransferServerContext.DtFlowStatus = DTS_APP_FLOW_ON;
////  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_UPDATE_ID, CFG_SCH_PRIO_0);
//  osThreadFlagsSet( DataTransferProcessId, 1 );
//
//  return;
//}

/*************************************************************
 *
 * CALLBACK FUNCTIONS
 *
 *************************************************************/
void Generic_Notification( DTS_STM_App_Notification_evt_t *pNotification )
{
  switch (pNotification->Evt_Opcode)
  {
    case DTS_STM__NOTIFICATION_ENABLED:
      DataTransferServerContext.NotificationTransferReq = DTS_APP_TRANSFER_REQ_ON;
//      UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_UPDATE_ID, CFG_SCH_PRIO_0);
//      osThreadFlagsSet( DataTransferProcessId, 1 );
      break;

    case DTS_STM_NOTIFICATION_DISABLED:
      DataTransferServerContext.NotificationTransferReq = DTS_APP_TRANSFER_REQ_OFF;
      break;
      
    case DTC_NOTIFICATION_ENABLED:
      BLE_SVC_L2CAP_Conn_Update_7_5();

      break;
      
    case DTC_NOTIFICATION_DISABLED:
      //DataTransferServerContext.NotificationClientTransferFlag = 0x00;
      APP_DBG_MSG("write data notification disabled \n");
      break;
      
    case DTS_STM_DATA_RECEIVED:
      if (DataReceived == 0)
      {
        /* start timer */
          APP_DBG_MSG("START TIMER (REMOVED)\n");

        DataReceived += pNotification->DataTransfered.Length;
//        HW_TS_Start(TimerDataThroughputWrite_Id, DELAY_1s);
      }
      else
      {
        DataReceived += pNotification->DataTransfered.Length;
      }
      break;

    case DTS_STM_GATT_TX_POOL_AVAILABLE:
      DataTransferServerContext.DtFlowStatus = DTS_APP_FLOW_ON;
//      UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_UPDATE_ID, CFG_SCH_PRIO_0);
//      osThreadFlagsSet( DataTransferProcessId, 1 );
      break;

    default:
      break;
  }

  return;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
//void Resume_Notification(void)
//{
//  DataTransferServerContext.DtFlowStatus = DTS_APP_FLOW_ON;
//}
//void ButtonTriggerReceived( void * argument )
//{
//	UNUSED(argument);
//	  for(;;)
//	  {
//	    osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);
//  if(DataTransferServerContext.ButtonTransferReq != DTS_APP_TRANSFER_REQ_OFF)
//  {
////    BSP_LED_Off(LED_BLUE);
//    DataTransferServerContext.ButtonTransferReq = DTS_APP_TRANSFER_REQ_OFF;
//  }
//  else
//  {
////    BSP_LED_On(LED_BLUE);
//    DataTransferServerContext.ButtonTransferReq = DTS_APP_TRANSFER_REQ_ON;
////    UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_UPDATE_ID, CFG_SCH_PRIO_0);
//    osThreadFlagsSet( DataTransferProcessId, 1 );
//  }
//	  }
////  return;
//}

//void DT_App_Button2_Trigger_Received( void * argument )
//{
//	UNUSED(argument);
//	  for(;;)
//	  {
//	    osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);
//  APP_DBG_MSG("change PHY \n");
//  BLE_SVC_GAP_Change_PHY();
//  return;
//	  }
//}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
