/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>

//String sysName = "EnvGlass";
char sysName[] = "AirSpec";

//static BLEUUID serviceUUID("0000FE80-8E22-4541-9D4C-21EDAE82ED19");
//static BLEUUID serviceUUID("FE80");
//BLEClientService dataService = BLEService("FE80");
BLEClientService dataService("FE80");
//BLEClientCharacteristic dataChar = BLEClientCharacteristic("FE81");
BLEClientCharacteristic dataChar("FE81");

#define GATT_MTU_SIZE_DEFAULT 103 // before this was 23

//0000fe80-0000-1000-8000-00805f9b34fb
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
//static BLEUUID    charUUID("0000FE81-8E22-4541-9D4C-21EDAE82ED19");
//static BLEUUID charUUID("FE81");

void setup() 
{
  Serial.begin(1000000);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central Scan Example");
  Serial.println("--------------------------------\n");

  // Config the connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);  
//  Bluefruit.configCentralConn(100, 6, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  dataService.begin();
//  dataChar.setMaxLen(512);
//  dataChar.setFixedLen(32);//(SIZE_CPS_MEAS_DATA);
//  dataChar.setMaxLen(32);
  dataChar.setNotifyCallback(data_notify_callback);
  dataChar.begin();

  // Start Central Scan
  Bluefruit.setConnLedInterval(250);

    // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Scanner.filterUuid(dataService.uuid);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);

//  Bluefruit.Scanner.filterMSD(0x2A29);

  Bluefruit.Scanner.start(0);

  Serial.println("Scanning ...");
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

//  Bluefruit.Scanner.start(0); //start on disconnect is activated in setup function
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  Serial.print("Max MTU: ");
//  Serial.println(Bluefruit.Gap.getMaxMtu(conn_handle));
  // If HRM is not found, disconnect and return
  if ( !dataService.discover(conn_handle) )
  {
    Serial.println("Found NONE");

    // disconnect since we couldn't find HRM service
    Bluefruit.disconnect(conn_handle);

    return;
  }

  // Once HRM service is found, we continue to discover its characteristic
  Serial.println("Found it");

  if ( !dataChar.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    Serial.println("not found !!!");  
    Serial.println("Measurement characteristic is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Found it x2");
  
//  Bluefruit.disconnect(conn_handle);
  
//  Serial.print("Discovering BLE Uart Service ... ");
//  if ( clientUart.discover(conn_handle) )
//  {
//    Serial.println("Found it");
//
//    Serial.println("Enable TXD's notify");
//    clientUart.enableTXD();
//
//    Serial.println("Ready to receive from peripheral");
//  }else
//  {
//    Serial.println("Found NONE");
//    
//    // disconnect since we couldn't find bleuart service
//    Bluefruit.disconnect(conn_handle);
//  }  

  // Reaching here means we are ready to go, let's enable notification on measurement chr
  if ( dataChar.enableNotify() )
  {
    Serial.println("Ready to receive HRM Measurement value");
  }else
  {
    Serial.println("Couldn't enable notify for HRM Measurement. Increase DEBUG LEVEL for troubleshooting");
  }
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{


  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForUuid(report, "FE80") )
  {
    uint8_t len = 0;
    uint8_t buffer[32];
  
    Serial.println("AirSpec Found");

    Serial.println("Timestamp Addr              Rssi Data");
  
    Serial.printf("%09d ", millis());
    
    // MAC is in little endian --> print reverse
    Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
    Serial.print(" ");
  
    Serial.print(report->rssi);
    Serial.print("  ");
  
    Serial.printBuffer(report->data.p_data, report->data.len, '-');
    Serial.println();

        /* Shortened Local Name */
    if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
    {
      Serial.printf("%14s %s\n", "SHORT NAME", buffer);
      memset(buffer, 0, sizeof(buffer));
    }
  
    /* Complete Local Name */
    if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
    {
      buffer[7] = 0; // not sure why but the library reads "AirSpec?" so setting last bit to null
      Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
      if(Serial.print(strcmp((char*) buffer, sysName))){
        Bluefruit.Central.connect(report); // connect if name of device is "AirSpec"
      }
      memset(buffer, 0, sizeof(buffer));

    }
 
//    /* Check for Manufacturer Specific Data */
//    len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
//    if (len)
//    {
//      Serial.printf("%14s ", "MAN SPEC DATA");
//      Serial.printBuffer(buffer, len, '-');
//      Serial.println();
//      memset(buffer, 0, sizeof(buffer));
//    }  

    // Connect to device with bleuart service in advertising
//    Bluefruit.Central.connect(report);
  }

//  Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void loop() 
{
  // nothing to do
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be hrmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void data_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml
  // Measurement contains of control byte0 and measurement (8 or 16 bit) + optional field
  // if byte0's bit0 is 0 --> measurement is 8 bit, otherwise 16 bit.

//  Serial.println("Data Measurement");
//  Serial.println(data);
  
    char hexBuf[3];
    Serial.println(len);
    for(int i = 0; i<len; i++){
      sprintf(hexBuf, "%02x", *(data+i));
      Serial.print(hexBuf);
    }
    Serial.println();
//
//  if ( data[0] & bit(0) )
//  {
//    uint16_t value;
//    memcpy(&value, data+1, 2);
//
//    Serial.println(value);
//  }
//  else
//  {
//    Serial.println(data[1]);
//  }
}
