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
typedef struct {
  uint32_t seq_no;
  uint32_t ts;
  uint16_t type;
  uint16_t sub_type;
  union {
    // general
    char payload[52];
    // type 1.x
    struct {
      float throttle;
      float steering;
    };
  };
} Packet;

// board pin layout
int encoder_s_pin = 14;
// left
int steer_rev_pin = 3;
// right
int steer_fwd_pin = 2;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

// electronics calibration
float steering_kp = 100;
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
  digitalWrite(steer_rev_pin,LOW);
  digitalWrite(steer_fwd_pin,LOW);

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

    // response
    buildResponsePacket();
    int retval = sendResponsePacket();
    /*
    if (retval){
      Serial.println("sent success");
    } else {
      Serial.println("sent FAIL");
    }
    */
  }

  if (millis() - last_packet_ts > 100){
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    Serial.print(millis());
    Serial.println(" failsafe");
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
  Serial.print("type: ");
  Serial.println(p->type);
  Serial.print("subtype: ");
  Serial.println(p->sub_type);
  */
  if (p->type == 1){
    /*
    Serial.print("throttle: ");
    Serial.println(p->throttle,5);
    Serial.print("steering: ");
    Serial.println(p->steering,5);
    */
    steering = p->steering;
    throttle = p->throttle;
  }
}

// build response packet, save in out_buffer
// here we just echo the packet
void buildResponsePacket(){
  memcpy(out_buffer, in_buffer, PACKET_SIZE);
}

int sendResponsePacket(){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(in_buffer,PACKET_SIZE);
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
  Serial.print(millis());
  Serial.print("  ");

  if (abs(throttle) < throttle_deadzone){
    analogWrite(drive_fwd_pin, 0);
    analogWrite(drive_rev_pin, 0);
    digitalWrite(LED_BUILTIN,1);
    Serial.print("brake ");
  } else {
    digitalWrite(LED_BUILTIN,0);
    int throttle_value = (int)fmap(throttle, -1.0, 1.0, -255.0, 255.0);
    throttle_value = constrain(throttle_value, -255, 255);
    Serial.print(throttle_value);
    if (throttle_value > 0) {
      analogWrite(drive_fwd_pin, throttle_value);
      analogWrite(drive_rev_pin, 0);
    } else {
      analogWrite(drive_rev_pin, -throttle_value);
      analogWrite(drive_fwd_pin, 0);
    }
  }
  Serial.println();
  //FIXME XXX
  return;


  float raw_encoder = analogRead(encoder_s_pin);
  float measured_steering_rad = fmap(raw_encoder, full_left_encoder_value, full_right_encoder_value, full_left_angle_rad,full_right_angle_rad);
  measured_steering_rad = constrain(measured_steering_rad, full_right_angle_rad, full_left_angle_rad);

  float err = steering - measured_steering_rad;
  /*
  Serial.print("raw: ");
  Serial.print(raw_encoder);
  Serial.print(" goal: ");
  Serial.print(steering/PI*180.0,5);
  Serial.print(" actual: ");
  Serial.print(measured_steering_rad/PI*180.0,5);
  Serial.println();
  */

  if (abs(err) < steering_deadzone_rad){
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  } else if (err > 0){
    analogWrite(steer_rev_pin, constrain(err*steering_kp, 0,255));
    analogWrite(steer_fwd_pin, 0);
  } else if (err < 0){
    analogWrite(steer_fwd_pin, constrain(-err*steering_kp, 0,255));
    analogWrite(steer_rev_pin, 0);
  }

}

