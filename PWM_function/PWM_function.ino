#include "pwm.h"
#include <Arduino.h>

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 2; // change to 9
// right
int steer_fwd_pin = 3; // change to 10

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

const int pulse_pin = 11;
volatile unsigned long pulse_begin_ts = 0;
volatile unsigned long pulse_reading = 0;

float fmap(float val, float a, float b, float c, float d){
    return c + (val-a)*(d-c)/(b-a);
}

void setup(){
    pinMode(pulse_pin, INPUT);
    Serial.begin(115200);
    PWM::setup();
    attachInterrupt(digitalPinToInterrupt(pulse_pin), readPulse, CHANGE);
}

void readPulse(){
    if (digitalRead(pulse_pin)){
        pulse_begin_ts = micros();
    } else {
        pulse_reading = micros()-pulse_begin_ts;
    }
    float val = fmap(pulse_reading, 1000, 2000, 0.0, 1.0);
    PWM::set(5,val);
    PWM::set(6,val);
    PWM::set(9,val);
    PWM::set(10,val);
}

void loop(){
    Serial.println(pulse_reading);
    delay(100);
}