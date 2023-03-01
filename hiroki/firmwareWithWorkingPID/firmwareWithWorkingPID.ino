///////////////////////////////////////
//SAMDTimerInterrupt Setup First Half//
///////////////////////////////////////

#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

/*
#ifndef pinVariableName
  #define pinVariableName       pinNumber
#endif
*/

#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

///////////////////////////
//End of Setup First Half//
///////////////////////////

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define PI 3.14159265

#include "network.h"
#include "packet.h"
#include "status_light.h"

////////////////////////////////////////
//SAMDTimerInterrupt Setup Second Half//
////////////////////////////////////////

#define HW_TIMER_INTERVAL_MS      10

// Depending on the board, you can select SAMD21 Hardware Timer from TC3, TC4, TC5, TCC, TCC1 or TCC2
// SAMD51 Hardware Timer only TC3

// Init SAMD timer TIMER_TC3 
SAMDTimer ITimer(TIMER_TC3); // using TIMER_TC3

#if (TIMER_INTERRUPT_USING_SAMD21)
// Init SAMD timer TIMER_TCC
//SAMDTimer ITimer(TIMER_TC4);
//SAMDTimer ITimer(TIMER_TC5);
//SAMDTimer ITimer(TIMER_TCC);
//SAMDTimer ITimer(TIMER_TCC1);
//SAMDTimer ITimer(TIMER_TCC2);
#endif

// Init SAMD_ISR_Timer
// Each SAMD_ISR_Timer can service 16 different ISR-based timers
SAMD_ISR_Timer ISR_Timer;

#define TIMER_INTERVAL_1S             1000L // change to period of choice
#define TIMER_INTERVAL_2S             2000L // 
#define TIMER_INTERVAL_5S             5000L // 

void TimerHandler(void)
{
  ISR_Timer.run();  
}

// In SAMD, avoid doing something fancy in ISR, for example complex Serial.print with String() argument
// The pure simple Serial.prints here are just for demonstration and testing. Must be eliminate in working environment
// Or you can get this run-time error / crash

void doingSomething2()
{
  //another function can go here
}

void doingSomething3()
{
  //another function can go here
}

////////////////////////////
//End of Setup Second Half//
////////////////////////////

// TODO use SAMD timer interrupt to achieve higher control frequency

unsigned int localPort = 2390;
unsigned long last_packet_ts = 0;

// board pin layout
int encoder_s_pin = 14;

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 2;
// right
int steer_fwd_pin = 3;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

// electronics calibration
// [-1,1]
volatile float throttle = 0.0;
// left positive, radians
volatile float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle_rad = 26.1/180.0*PI;
float full_right_angle_rad = -26.1/180.0*PI;
float full_left_encoder_value = 630.0;
float full_right_encoder_value = 410.0;
float steering_deadzone_rad = 1.0/180.0*PI;
unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;

bool flag_failsafe = false;
StatusLed led;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  pinMode(encoder_s_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);
  pinMode(drive_rev_pin, OUTPUT);
  pinMode(drive_fwd_pin, OUTPUT);
  led.init();
  led.off();
  /*
  setupWifi();
  Udp.begin(localPort);
  */

  ////////////////////////////////////
  ////SAMDTimerInterrupt in setup()///
  ///////////////////////////////////

  while (!Serial && millis() < 5000);

  delay(100);

  // configure pin in output mode
  /*
  pinMode(pinVariableName,  OUTPUT);
  */

  // Interval in millisecs
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary
  // You can use up to 16 timer for each ISR_Timer
  ISR_Timer.setInterval(TIMER_INTERVAL_1S,  PIDControl);
  // ISR_Timer.setInterval(TIMER_INTERVAL_2S,  doingSomething2);
  // ISR_Timer.setInterval(TIMER_INTERVAL_5S,  doingSomething3);
  
}



void blinkTwice(){
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);

}

void PIDControl()
{ 
  //digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  digitalWrite(LED_BUILTIN,HIGH);
  float raw_encoder = analogRead(encoder_s_pin);
  steering_measured = fmap(raw_encoder, full_left_encoder_value, full_right_encoder_value, full_left_angle_rad,full_right_angle_rad);
  steering_measured = constrain(steering_measured, full_right_angle_rad, full_left_angle_rad);
  
  float err = steering - steering_measured;
  steering_requested = steering;
  /*
  Serial.print("raw: ");
  Serial.print(raw_encoder);
  Serial.print(" goal: ");
  Serial.print(steering/PI*180.0,5);
  Serial.print(" actual: ");
  Serial.print(steering_measured/PI*180.0,5);
  Serial.println();
  */
  // filter
  float dt = (float)(millis() - last_packet_ts)/1000.0;
  float freq = 10.0;
  float alfa = (2*PI*dt*freq)/(2*PI*dt*freq + 1.0);
  err = (1.0-alfa)*last_err + alfa*err;
  steering_integral += err * dt;
  steering_integral = constrain(steering_integral, -steering_integral_limit, steering_integral_limit);
  float output = err * param_steering_P + steering_integral * param_steering_I + (err - last_err)/dt * param_steering_D;

/*
  if (abs(err) < steering_deadzone_rad){
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  } else if (output > 0){
    analogWrite(steer_rev_pin, constrain(err*param_steering_P, 0,255));
    analogWrite(steer_fwd_pin, 0);
  } else if (output < 0){
    analogWrite(steer_fwd_pin, constrain(-err*param_steering_P, 0,255));
    analogWrite(steer_rev_pin, 0);
  }
  */

  Serial.println(output);

  last_pid_ts = millis();
  last_err = err;
}

void setupWifi(){
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  led.blink();
  
  printCurrentNet();
  printWifiData();
}

void loop() {
  return;
  led.update();
//  Serial.println(millis() - loop_time);
//  loop_time = millis();
  int packet_size = Udp.parsePacket();
  
  // process incoming packet
  if (packet_size) {
    if (packet_size != PACKET_SIZE){ Serial.println("err packet size"); }

    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE){ Serial.println("err reading packet size"); }
    //Serial.println("parsing packet");
    parsePacket();
    last_packet_ts = millis();
    flag_failsafe = false;
    led.on();
    // response is handled by packet parser
  }

  if (millis() - last_packet_ts > 100 && !flag_failsafe){
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    //Serial.print(millis());
    //Serial.println(" failsafe");
    led.blink();
  } 

  actuateControls();
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// using global variable throttle and steering
// set pwm for servo and throttle
void actuateControls(){
  if (flag_failsafe){
    // brake mode for all
    // NOTE digitalWrite no longer works
    analogWrite(drive_fwd_pin, 0);
    analogWrite(drive_rev_pin, 0);
    analogWrite(steer_fwd_pin, 255);
    analogWrite(steer_rev_pin, 255);
    return;
  }

  if (abs(throttle) < throttle_deadzone){
    analogWrite(drive_fwd_pin, 0);
    analogWrite(drive_rev_pin, 0);
  } else {
    int throttle_value = (int)fmap(throttle, -1.0, 1.0, -255.0, 255.0);
    throttle_value = constrain(throttle_value, -255, 255);
    if (throttle_value > 0) {
      analogWrite(drive_fwd_pin, throttle_value);
      analogWrite(drive_rev_pin, 0);
    } else {
      analogWrite(drive_rev_pin, -throttle_value);
      analogWrite(drive_fwd_pin, 0);
    }
  }
}
