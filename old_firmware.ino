#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

//#include "arduino_secrets.h" 

int encoder_s_pin = 14;
int steer_rev_pin = 3;
int steer_fwd_pin = 2;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

unsigned int localPort = 2390;
char packetBuffer[255];
char ReplyBuffer[] = "acknowledged";

char driveBuffer[10];
char steerBuffer[10];


//long loop_time = millis();

char ssid[] = "mkel";
char pass[] = "matthewk";
int status = WL_IDLE_STATUS;

WiFiUDP Udp;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(encoder_s_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);

  pinMode(drive_rev_pin, OUTPUT);
  pinMode(drive_fwd_pin, OUTPUT);

  // LED on indicates we are connecting to wifi
  digitalWrite(LED_BUILTIN, HIGH);

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
  
  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  Udp.begin(localPort);

  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
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
  Serial.println();
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
float full_left_calibration_value = 400.0;
float full_right_calibration_value = 650.0;

float scalar_multiplier = 255.0 / (full_right_calibration_value - full_left_calibration_value);

float kP = 0.4;
float steer_tolerance = 10.0;

// default to straight
float steering_target = 127.5;
float P = 0.0;

void loop() {
//  Serial.println(millis() - loop_time);
//  loop_time = millis();
  int packetSize = Udp.parsePacket();

  // Uncomment this line to print encoder values to serial
  // This is useful if you want to recalibrate the steering encoder range
  // Serial.println(analogRead(encoder_s_pin));

  float raw_encoder = analogRead(encoder_s_pin);
  
  float scaled_encoder = (raw_encoder - full_left_calibration_value) * scalar_multiplier;
  scaled_encoder = max(scaled_encoder, 0);
  scaled_encoder = min(scaled_encoder, 255);

 
  
  // update steering P controller
  P = (int) (steering_target - scaled_encoder) * kP;
  if (P < -(steer_tolerance/2.0)) {
    analogWrite(steer_fwd_pin, max(min(P*-1, 255), 0));
    analogWrite(steer_rev_pin, 0);
  } else if (P > (steer_tolerance/2.0)) {
    analogWrite(steer_rev_pin, max(min(P, 255), 0));
    analogWrite(steer_fwd_pin, 0);
  } else {
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  }
  
  
  if (packetSize) {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
//    Serial.print(remoteIp);
//    Serial.print(", port ");
//    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
//    Serial.println("Contents:");
//    Serial.println(packetBuffer);

    


    strncpy(driveBuffer, packetBuffer, 3);
    strncpy(steerBuffer, packetBuffer + 3, 3);
    int raw_speed = atoi(driveBuffer);
    int raw_steer = atoi(steerBuffer);

    // Raw speed values range from 0 to 510.
    // 0 is full stop
    // 1-255 is forward motion, 256-510 is reverse motion
    if (raw_speed > 255) {
      analogWrite(drive_fwd_pin, raw_speed - 255);
      analogWrite(drive_rev_pin, 0);
    } else if (raw_speed <= 255) {
      analogWrite(drive_rev_pin, raw_speed);
      analogWrite(drive_fwd_pin, 0);
    }

    steering_target = raw_steer;
//    Serial.println(steering_target);

    
//    
//    analogWrite(drive_fwd_pin, );
//    analogWrite(drive_fwd_pin, atoi(speedBuffer));

    
//    if (value < 600 && value > 450) {
//       analogWrite(drive_fwd_pin, 100);
//    } else {
//      analogWrite(drive_fwd_pin, 0);
//    }
    
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
}
