#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define PI 3.14159265

//TODO add parameter server, enable remote setting
// TODO complete protocol
// TODO report mode change and status
// TODO more intuitive indicator light
// TODO add filter to encoder reading
// TODO put PWM frequency outside audible range
// read and send message asynchrously

// network setting
char ssid[] = "TP-LINK_F4D4";
char pass[] = "15291356";
int status = WL_IDLE_STATUS;
WiFiUDP Udp;
#define PACKET_SIZE 64
unsigned int localPort = 2390;
char in_buffer[PACKET_SIZE];
char out_buffer[PACKET_SIZE];
unsigned long last_packet_ts = 0;
uint32_t current_seq_no = 0;
typedef struct {
  uint32_t seq_no;
  uint32_t ts;
  uint8_t dest_addr;
  uint8_t src_addr;
  uint8_t type;
  uint8_t sub_type;
  union {
    // general
    char payload[52];
    // type 1.x, command packet
    struct {
      float throttle; // [-1,1]
      float steering; // in rad
    };
    // type 2.0, Sensor Update
    struct {
      float steering_requested; // in rad
      float steering_measured;  // in rad
    };
    // type 3.0, Parameter Packet
    struct {
      bool sensor_update; // 4 bytes due to alignment
      float steering_P;
      float steering_I;
      float steering_D;
    };
  };
} Packet;
// params
bool  param_sensor_update = true;
float param_steering_P = 100.0;
float param_steering_I = 0.0;
float param_steering_D = 0.0;

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
float throttle = 0.0;
// left positive, radians
float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle_rad = 26.1/180.0*PI;
float full_right_angle_rad = -26.1/180.0*PI;
float full_left_encoder_value = 630.0;
float full_right_encoder_value = 410.0;
float steering_deadzone_rad = 2.5/180.0*PI;

bool flag_failsafe = false;

// sensors
// steering_requested differs from steering in that it is synchronized to steering_measured
float steering_requested;
float steering_measured;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  setupWifi();
  blinkTwice();
  Udp.begin(localPort);
}

void blinkTwice(){
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(encoder_s_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);
  pinMode(drive_rev_pin, OUTPUT);
  pinMode(drive_fwd_pin, OUTPUT);
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);

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
  
  printCurrentNet();
  printWifiData();
}

void loop() {
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

    // response is handled by packet parser
  }

  if (millis() - last_packet_ts > 100){
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    //Serial.print(millis());
    //Serial.println(" failsafe");
  }

  actuateControls();
}

// parse packet in in_buffer
// set appropriate global variables
void parsePacket(){
  Packet *p = (Packet *) in_buffer;

  /*
  Serial.print("seq_no: ");
  Serial.println(p->seq_no);
  Serial.print("ts: ");
  Serial.println(p->ts);
  Serial.print("dest_addr: ");
  Serial.println(p->dest_addr);
  Serial.print("src_addr: ");
  Serial.println(p->src_addr);
  Serial.print("type: ");
  Serial.println(p->type);
  Serial.print("subtype: ");
  Serial.println(p->sub_type);
  */
  switch (p->type){
    case 0:
      parsePingPacket();
      break;
    case 1:
      parseCommandPacket();
      break;
    case 2:
      parseSensorPacket();
      break;
    case 3:
      parseParamPacket();
      break;
   default:
      // should never reach here
      break;
  }
}

void parsePingPacket(){
  //Packet *p = (Packet *) in_buffer;
  buildPingResponsePacket();
  sendResponsePacket();
}
// send sensor response
void parseSensorPacket(){
  buildSensorResponsePacket();
  sendResponsePacket();
}
void parseCommandPacket(){
  /*
  Serial.print("throttle: ");
  Serial.println(p->throttle,5);
  Serial.print("steering: ");
  Serial.println(p->steering,5);
  */
  Packet *p = (Packet *) in_buffer;
  steering = p->steering;
  throttle = p->throttle;

  buildSensorResponsePacket();
  sendResponsePacket();
}

void parseParamPacket(){
  Packet *p = (Packet *) in_buffer;
  if (p->sub_type == 3){
    param_sensor_update = p->sensor_update;
    param_steering_P = p->steering_P;
    param_steering_I = p->steering_I;
    param_steering_D = p->steering_D;
    Serial.print("setting new param: ");
    Serial.print(" sensor_update: ");
    Serial.print(param_sensor_update);
    Serial.print(" steering_P: ");
    Serial.print(param_steering_P);
    Serial.println();
  }
  // all subtype get parameter printout 3.0
  p = (Packet *) out_buffer;
  p->sensor_update = param_sensor_update;
  p->steering_P = param_steering_P;
  p->steering_I = param_steering_I;
  p->steering_D = param_steering_D;
  buildHeader(3,0);
  sendResponsePacket();
}

void buildPingResponsePacket(){
  buildHeader(0,1);
  Packet *p = (Packet *) out_buffer;
}

void buildSensorResponsePacket(){
  buildHeader(2,0);
  Packet *p = (Packet *) out_buffer;
  p->steering_requested = steering_requested;
  p->steering_measured = steering_measured;
}

void buildHeader(uint8_t type, uint8_t subtype){
  Packet *p = (Packet *) out_buffer;
  p->seq_no = current_seq_no++;
  p->ts = micros();
  p->dest_addr = 0;
  p->src_addr = 1;
  p->type = type;
  p->sub_type = subtype;
}

int sendResponsePacket(){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(out_buffer,PACKET_SIZE);
  /*
  if (retval){
    Serial.println("sent success");
  } else {
    Serial.println("sent FAIL");
  }
  */
  return Udp.endPacket();
}

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // you're connected now, so print out the data:
  Serial.println("Module connected to the network:");
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println("----- entering loop -----");
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
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

  float raw_encoder = analogRead(encoder_s_pin);
  steering_measured = fmap(raw_encoder, full_left_encoder_value, full_right_encoder_value, full_left_angle_rad,full_right_angle_rad);
  steering_measured = constrain(steering_measured, full_right_angle_rad, full_left_angle_rad);

  float err = steering - steering_measured;
  steering_requested = steering;
  Serial.print("raw: ");
  Serial.print(raw_encoder);
  Serial.print(" goal: ");
  Serial.print(steering/PI*180.0,5);
  Serial.print(" actual: ");
  Serial.print(steering_measured/PI*180.0,5);
  Serial.println();

  if (abs(err) < steering_deadzone_rad){
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  } else if (err > 0){
    analogWrite(steer_rev_pin, constrain(err*param_steering_P, 0,255));
    analogWrite(steer_fwd_pin, 0);
  } else if (err < 0){
    analogWrite(steer_fwd_pin, constrain(-err*param_steering_P, 0,255));
    analogWrite(steer_rev_pin, 0);
  }

}

