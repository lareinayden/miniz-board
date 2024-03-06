#pragma once
#include "TCA9548A.h"
#include "Adafruit_VL53L0X.h"

class RangeFinder {
public:
  byte channel;
  TCA9548A *mux;

  Adafruit_VL53L0X *lox;

  volatile byte state;

  RangeFinder(byte channel, TCA9548A *mux) {
    this->channel = channel;
    this->mux = mux;
    //this->interruptPin = interruptPin;

    this->state = LOW;
    this->lox = new Adafruit_VL53L0X();
  }

  ~RangeFinder() {
    delete this->lox;
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
    if (!lox->begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
      while (true) delay(100);
    }

    Serial.println("booted chip!");

  // Second Parameter options are VL53L0X_GPIOFUNCTIONALITY_OFF,
    // VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
    // VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
    // VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
    // VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY

    Serial.println("Set GPIO Config so if range is lower the LowThreshold "
                  "trigger Gpio Pin ");
    /*lox->setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                      VL53L0X_INTERRUPTPOLARITY_LOW);*/

    // Set Interrupt Treashholds
    // Low reading set to 50mm  High Set to 100mm
    FixPoint1616_t LowThreashHold = (50 * 65536.0);
    FixPoint1616_t HighThreashHold = (100 * 65536.0);
    Serial.println("Set Interrupt Threasholds... ");
    //lox->setInterruptThresholds(LowThreashHold, HighThreashHold, true);

    // Enable Continous Measurement Mode
    Serial.println("Set Mode VL53L0X_DEVICEMODE_CONTINUOUS_RANGING... ");
    //lox->setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);

    Serial.println("StartMeasurement... ");
    //lox->startMeasurement();

    lox->startRangeContinuous(10);

    ReleaseMux();
  }

  int measureRange() {
    AcquireMux();

    uint16_t range = lox->readRangeResult();

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