#include <SD.h>
#include <SPI.h>

#define FET_1  2
#define FET_2  28
#define FET_3  12
#define FET_4  8
#define FET_5  4
#define FET_6  5

//#define CTRL_1  2
//#define CTRL_2  28
//#define CTRL_3  12
//#define CTRL_4  8
//#define CTRL_5  4
//#define CTRL_6  5

#define SENSE_1_0 A0
#define SENSE_1_1 A1
#define SENSE_2_0 A2
#define SENSE_2_1 A3
#define SENSE_3_0 A4
#define SENSE_3_1 A5
#define SENSE_4_0 A6
#define SENSE_4_1 A7
#define SENSE_5_0 A8
#define SENSE_5_1 A9
#define SENSE_6_0 A14
#define SENSE_6_1 A15

//#define FET_ADC_OFF_THRESH          1140 //~0.92V (V_BATT ~= 3.2V)
//#define FET_ADC_SAFETY_UPPER_THRESH 4046 //~3.26V

#define FET_ADC_OFF_THRESH          0.92 //~0.92V (V_BATT ~= 3.2V)
#define FET_ADC_SAFETY_UPPER_THRESH 3.26 //~3.26V

const int chipSelect = BUILTIN_SDCARD;

struct BattStruct{             
  float  voltage_0;
  float  voltage_1;
  uint32_t  resVal;
  float     current;
  uint8_t   testOver;
  uint32_t  timeSample;
};

BattStruct batt_1, batt_2, batt_3, batt_4, batt_5, batt_6;

// Create an IntervalTimer object 
IntervalTimer myTimer;

String fileName;
char fileNameChar[20];

void setup() {
  pinMode(FET_1, OUTPUT);
  pinMode(FET_2, OUTPUT);
  pinMode(FET_3, OUTPUT);
  pinMode(FET_4, OUTPUT);
  pinMode(FET_5, OUTPUT);
  pinMode(FET_6, OUTPUT);

  // enable all FETS
  digitalWrite(FET_1, HIGH); 
  digitalWrite(FET_2, HIGH); 
  digitalWrite(FET_3, HIGH); 
  digitalWrite(FET_4, HIGH); 
  digitalWrite(FET_5, HIGH); 
  digitalWrite(FET_6, HIGH); 
  
  batt_1.resVal=56;
  batt_2.resVal=56;
  batt_3.resVal=56;
  batt_4.resVal=56;
  batt_5.resVal=56;
  batt_6.resVal=56;

  batt_1.testOver=0;
  batt_2.testOver=0;
  batt_3.testOver=0;
  batt_4.testOver=0;
  batt_5.testOver=0;
  batt_6.testOver=0;

  analogReadResolution(12);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  
  Serial.println("card initialized.");

  // figure out fileName
  fileName = "datalog_1.csv";
  int i = 1;
  fileName.toCharArray(fileNameChar, fileName.length()+1);
  while(SD.exists(fileNameChar)){
    i++;
    fileName = "datalog_" + String(i) + ".csv";
    fileName.toCharArray(fileNameChar, fileName.length()+1);
  }

  Serial.print("logging file name:\t");
  Serial.println(fileNameChar);
  
  delay(10000);

  myTimer.begin(sampleBatteries, 100000);  // 1 Hz

}

void loop() {

}

void sampleBatteries(){
  Serial.print("Batt_1\t");
  sampleFunc(&batt_1, SENSE_1_0, SENSE_1_1, FET_1);
  Serial.print("Batt_2\t");
  sampleFunc(&batt_2, SENSE_2_0, SENSE_2_1, FET_2);
  Serial.print("Batt_3\t");
  sampleFunc(&batt_3, SENSE_3_0, SENSE_3_1, FET_3);
  Serial.print("Batt_4\t");
  sampleFunc(&batt_4, SENSE_4_0, SENSE_4_1, FET_4);
  Serial.print("Batt_5\t");
  sampleFunc(&batt_5, SENSE_5_0, SENSE_5_1, FET_5);
  Serial.print("Batt_6\t");
  sampleFunc(&batt_6, SENSE_6_0, SENSE_6_1, FET_6);

  saveBatteryVals();
  Serial.println("===================================");
}

String serializeBatteryVals(BattStruct battery){
  String dataString = "";
  dataString += String(battery.voltage_0,5);
  dataString += ",";
  dataString += String(battery.voltage_1,5);
  dataString += ",";
  dataString += String(battery.resVal);
  dataString += ",";
  dataString += String(battery.current,5);
  dataString += ",";
  dataString += String(battery.timeSample);
  dataString += ",";
  dataString += String(battery.testOver);

  return dataString;
}


void saveBatteryVals(void){
  // open the file.
  File dataFile = SD.open(fileNameChar, FILE_WRITE);

  String dataString = "";
  dataString += serializeBatteryVals(batt_1);
  dataString += ",";
  dataString += serializeBatteryVals(batt_2);
  dataString += ",";
  dataString += serializeBatteryVals(batt_3);
  dataString += ",";
  dataString += serializeBatteryVals(batt_4);
  dataString += ",";
  dataString += serializeBatteryVals(batt_5);
  dataString += ",";
  dataString += serializeBatteryVals(batt_6);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
//    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

void sampleFunc(BattStruct *battery, uint8_t ADC_PIN_0, uint8_t ADC_PIN_1, uint8_t FET_PIN){
  battery->timeSample = millis();
  battery->voltage_0 = 3.300*(analogRead(ADC_PIN_0)/4096.0);
  battery->voltage_1 = 3.300*(analogRead(ADC_PIN_1)/4096.0);

  battery->current = ((float) (battery->voltage_1 - battery->voltage_0))/battery->resVal;

  Serial.print("Voltage Sample: ");
  Serial.print(battery->voltage_0,5);
  Serial.print("\t");
  Serial.print(battery->voltage_1,5);
  Serial.print("\t");
  Serial.print("Current:");
  Serial.print(battery->current,5);
  Serial.println(" mA"); 
  
  if(battery->testOver != 1){
    if(battery->voltage_1 < FET_ADC_OFF_THRESH){
      // turn off FET to protect the battery
      digitalWrite(FET_PIN, LOW); 
      battery->testOver = 1;
      Serial.println("TEST OVER. Undervoltage condition reached.");
    }else if(battery->voltage_1 >= FET_ADC_SAFETY_UPPER_THRESH){
      digitalWrite(FET_PIN, HIGH); 
    }
  }else{
    // if FET is on (battery is discharging)
    if(digitalRead(FET_PIN) == HIGH){
      if(battery->voltage_1 < FET_ADC_OFF_THRESH){
        // turn off FET to protect the battery
        digitalWrite(FET_PIN, LOW); 
      }
    }else{
      if(battery->voltage_1 >= FET_ADC_SAFETY_UPPER_THRESH){
        // turn on FET to protect GPIO pin
        digitalWrite(FET_PIN, HIGH); 
      }
    }
  }
}
