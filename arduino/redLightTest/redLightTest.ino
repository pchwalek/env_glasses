/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
//#include "BLEScan.h"
#include "message.pb.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

char sysName[] = "AirSpec";
// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000fe80-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("0000fe82-0000-1000-8000-00805f9b34fb");
// static air_spec_config_packet_t airspecMessage

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static air_spec_config_packet_t airspecMessage;

static uint8_t buffer[150];

void sendAirSpecRedFlash(uint32_t duration_ms, uint32_t frequency, uint8_t enable_speaker){
    Serial.print("Duration (ms): ");
    Serial.println(duration_ms);
    Serial.print("Frequency (Hz): ");
    Serial.println(frequency);
    Serial.print("Speaker Enabled (ms): ");
    Serial.println(enable_speaker);

    airspecMessage.has_header = false;
    airspecMessage.which_payload = AIR_SPEC_CONFIG_PACKET_RED_FLASH_TASK_TAG;
    airspecMessage.payload.red_flash_task.enable = 1;
    airspecMessage.payload.red_flash_task.red_max_intensity = 255;
    airspecMessage.payload.red_flash_task.red_min_intensity = 0;
    airspecMessage.payload.red_flash_task.frequency = frequency;
    airspecMessage.payload.red_flash_task.duration_ms = duration_ms;
    airspecMessage.payload.red_flash_task.enable_speaker = enable_speaker;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, AIR_SPEC_CONFIG_PACKET_FIELDS, &airspecMessage);

    if (connected) {
    // String newValue = "Time since boot: " + String(millis()/1000);

    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue((uint8_t *) &buffer, stream.bytes_written, false);
    }
    Serial.println("sending!");
}

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");


    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.

const char s[2] = ",";
static char inputStringBuff[50];
// This is the Arduino main loop function.
void loop() {

  if (Serial.available()) {   
    String inputstr = Serial.readString();  //read until timeout
    inputstr.toCharArray(inputStringBuff, inputstr.length());
    inputstr.trim();       // If anything comes in Serial (USB),
        /* get the first token */
    String token1 = strtok(inputStringBuff, s);
    String token2 = strtok(NULL, s);
    String token3 = strtok(NULL, s);
    /* walk through other tokens */
    // while( token != NULL ) {
    //     token = strtok(NULL, s);
    // 

    sendAirSpecRedFlash(token1.toInt(),token2.toInt(),token3.toInt());
  }

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    // String newValue = "Time since boot: " + String(millis()/1000);
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop