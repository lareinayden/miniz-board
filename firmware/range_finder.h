#pragma once
#include "TCA9548A.h"
#include <vl53l4cx_class.h>
#include <Wire.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

class RangeFinder {
public:
  byte channel;
  TCA9548A *mux;

  VL53L4CX *sensor_vl53l4cx_sat;

  volatile byte state;

  RangeFinder(byte channel, TCA9548A *mux) {
    this->channel = channel;
    this->mux = mux;
    //this->interruptPin = interruptPin;

    this->state = LOW;
    this->sensor_vl53l4cx_sat = new VL53L4CX(&Wire, A1);
  }

  ~RangeFinder() {
    delete this->sensor_vl53l4cx_sat;
  }

  void setup() {
    //pinMode(shutdownPin, OUTPUT);
    //attachInterrupt(digitalPinToInterrupt(interruptPin), VL53LOXISR,
    //                CHANGE);\

    Serial.println("Getting mux...");

    AcquireMux();

    Serial.println("got mux!");

    Serial.println("trying to begin()");
    
    // if lox->begin failes its becasue it was a warm boot and the VL53LOX is in
    // continues mesurement mode we can use an IO pin to reset the device in case
    // we get stuck in this mode
    // if (!sensor_vl53l4cx_sat->InitSensor(0x12);) {
    //   Serial.println(F("Failed to boot VL53L4CX"));
    //   while (true) delay(100);
    // }
    
    pinMode(LedPin, OUTPUT);
    Wire.begin();
    sensor_vl53l4cx_sat->begin();
    sensor_vl53l4cx_sat->VL53L4CX_Off();
    sensor_vl53l4cx_sat->InitSensor(0x12);  // Initialize with I2C address 0x12
    sensor_vl53l4cx_sat->VL53L4CX_StartMeasurement();
    Serial.println("StartMeasurement... ");


    // Set Interrupt Treashholds
    // Low reading set to 50mm  High Set to 100mm
    FixPoint1616_t LowThreashHold = (50 * 65536.0);
    FixPoint1616_t HighThreashHold = (100 * 65536.0);
    Serial.println("Set Interrupt Threasholds... ");
    //lox->setInterruptThresholds(LowThreashHold, HighThreashHold, true);

    // Enable Continous Measurement Mode
    // Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");
    //lox->setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);

    //lox->startMeasurement();

    // lox->configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
    // lox->startRangeContinuous(10);

    ReleaseMux();
  }

  int measureRange() {
    AcquireMux();

    // uint16_t range = lox->readRangeResult();

    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0;
    char report[64];
    int status;
    uint16_t range = 0;

    do {
      status = sensor_vl53l4cx_sat->VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LedPin, HIGH);

    if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l4cx_sat->VL53L4CX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      Serial.println(report);
      for (int j = 0; j < no_of_object_found; j++) {
        if (j != 0) {
          Serial.print("\r\n                               ");
        }
        Serial.print("status=");
        Serial.print(pMultiRangingData->RangeData[j].RangeStatus);
        range = pMultiRangingData->RangeData[j].RangeMilliMeter;
        Serial.print(", D=");
        Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
        Serial.print("mm");
      }
      if (status == 0) {
        status = sensor_vl53l4cx_sat->VL53L4CX_ClearInterruptAndStartMeasurement();
      }
    }

    digitalWrite(LedPin, LOW);
    ReleaseMux();

    return (int) range;
  }

private:
  void AcquireMux() {
    this->mux->openChannel(channel);
  }

  void ReleaseMux() {
    this->mux->closeChannel(channel);
  }
};