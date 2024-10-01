#include "range_finder.h"
#include "range_finder_block.hpp"
#include <Wire.h>

static TCA9548A I2CMux;

static const byte XSHUT=2;

static RangeFinder rightSensor(0, &I2CMux);
static RangeFinder frontSensor(1, &I2CMux);
static RangeFinder leftSensor(2, &I2CMux);
static RangeFinder backSensor(3, &I2CMux);

void setup_range_finders() {
    Serial.begin(115200);
    pinMode(XSHUT, OUTPUT);
    digitalWrite(XSHUT, HIGH);

    Serial.println("Set XShut to High");

    delay(500);
    
    Serial.println("Adafruit VL53L0X XShut set Low to Force HW Reset");
    digitalWrite(XSHUT, LOW);
    delay(3000);
    digitalWrite(XSHUT, HIGH);
    Serial.println("Adafruit VL53L0X XShut set high to Allow Boot");

    delay(500);
    
    I2CMux.begin(Wire);
    I2CMux.closeAll();

    rightSensor.setup();
    frontSensor.setup();
    leftSensor.setup();
    backSensor.setup();

    Serial.println("SETUP RANGE FINDERS DONE!");
}

range_finder_readings_t sample_range_finders() {
    range_finder_readings_t lidar_data{};

    lidar_data.front = (uint16_t) frontSensor.measureRange();
    lidar_data.right = (uint16_t) rightSensor.measureRange();
    lidar_data.left = (uint16_t) leftSensor.measureRange();
    lidar_data.back = (uint16_t) backSensor.measureRange();

    return lidar_data;
}

static range_finder_readings_t latest_measurement{};

void run_range_finder_timer() {
    range_finder_readings_t result = sample_range_finders();
    latest_measurement = result;
}

range_finder_readings_t get_latest_range_finder_sample() {
    range_finder_readings_t reading = latest_measurement;

    return reading;
}