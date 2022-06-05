#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define PACKET_SIZE 64
unsigned int localPort = 2390;
char in_buffer[PACKET_SIZE];
char out_buffer[PACKET_SIZE];

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

char ssid[] = "TP-LINK_F4D4";
char pass[] = "15291356";
int status = WL_IDLE_STATUS;

int encoder_s_pin = 14;
int steer_rev_pin = 3;
int steer_fwd_pin = 2;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

float steering_kp = 106.0;
// [-1,1]
float throttle = 0.0;
float full_left_angle_rad = 27.0/180.0*3.141593;
float full_right_angle_rad = -27.0/180.0*3.141593;
float steering = 0.0;

WiFiUDP Udp;

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
  
  if (packet_size) {
    if (packet_size != PACKET_SIZE){ Serial.println("err packet size"); }

    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE){ Serial.println("err reading packet size"); }
    Serial.println("parsing packet");
    parsePacket();
    buildResponsePacket();
    int retval = sendResponsePacket();
    if (retval){
      Serial.println("sent success");
    } else {
      Serial.println("sent FAIL");
    }

  }
}

// parse packet in in_buffer
// set appropriate global variables
void parsePacket(){
  Packet *p = (Packet *) in_buffer;

  Serial.print("seq_no: ");
  Serial.println(p->seq_no);
  Serial.print("ts: ");
  Serial.println(p->ts);
  Serial.print("type: ");
  Serial.println(p->type);
  Serial.print("subtype: ");
  Serial.println(p->sub_type);
  if (p->type == 1){
    Serial.print("throttle: ");
    Serial.println(p->throttle,5);
    Serial.print("steering: ");
    Serial.println(p->steering,5);
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

// The encoder returns values from ~400-650... We wan't to scale this to 0-255.
// This barely changes the values for most encoders, but is here in case some encoders have a wider or narrower range.
float full_left_encoder_value = 400.0;
float full_right_encoder_value = 650.0;
float steering_deadzone_rad = 10.0/180.0*3.141593;

// using global variable throttle and steering
// set pwm for servo and throttle
void actuateControls(){
  int throttle_value = map(throttle, -1.0, 1.0, 0, 510);
  if (throttle_value > 255) {
    analogWrite(drive_fwd_pin, throttle_value - 255);
    analogWrite(drive_rev_pin, 0);
  } else if (throttle_value <= 255) {
    analogWrite(drive_rev_pin, throttle_value);
    analogWrite(drive_fwd_pin, 0);
  }


  float raw_encoder = analogRead(encoder_s_pin);
  float actual_steering_angle_rad = map(raw_encoder, full_left_encoder_value, full_right_encoder_value, full_left_angle_rad,full_right_angle_rad);
  actual_steering_angle_rad = constrain(actual_steering_angle_rad, full_left_angle_rad, full_right_angle_rad);

  int err = steering - actual_steering_angle_rad;

  if (err < -(steering_deadzone_rad/2.0)) {
    analogWrite(steer_fwd_pin, constrain(-err*steering_kp, 0,255));
    analogWrite(steer_rev_pin, 0);
  } else if (err > (steering_deadzone_rad/2.0)) {
    analogWrite(steer_fwd_pin, constrain(err*steering_kp, 0,255));
    analogWrite(steer_fwd_pin, 0);
  } else {
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  }

}

