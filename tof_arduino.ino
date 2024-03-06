#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "range_finder.hpp"
#include <SPI.h>
#include <WiFi.h>
#include "utility/spi_drv.h"

char ssid[] = "TP-LINK_F4D4";
char password[] = "15291356";

WiFiUDP Udp;

unsigned int localPort = 28830;

static TCA9548A I2CMux;
//volatile byte VL53LOX_State = LOW;
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

static byte XSHUT=2;

//DONT USE 7 it interferes with SPI
RangeFinder frontSensor(0, &I2CMux);
RangeFinder rightSensor(1, &I2CMux);
RangeFinder leftSensor(2, &I2CMux);
RangeFinder backSensor(3, &I2CMux);
//RangeFinder sensor1(1, &I2CMux, 6, 7);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(XSHUT, OUTPUT);

  
  Serial.println("Adafruit VL53L0X XShut set Low to Force HW Reset");
  digitalWrite(XSHUT, LOW);
  delay(500);
  digitalWrite(XSHUT, HIGH);
  Serial.println("Adafruit VL53L0X XShut set high to Allow Boot");

  //now initialize sensors

  I2CMux.begin(Wire);
  I2CMux.closeAll();

  Serial.println("ToF...");

  frontSensor.setup();
  rightSensor.setup();
  leftSensor.setup();
  backSensor.setup();


  // Check for the presence of the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue
    while (true);
  }

  // Start the access point
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  // Set up the network
  //WiFi.beginAP(ssid, password);
  WiFi.begin(ssid, password);

  // Wait for network to be created
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(WiFi.status());
    Serial.println("Setting up network...");
  }

  Serial.print("Access Point Created. IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
}

void VL53LOXISR() {
  // Read if we are high or low
  //VL53LOX_State = digitalRead(VL53LOX_InterruptPin);
  // set the built in LED to reflect in range on for Out of range off for in
  // range
  //digitalWrite(LED_BUILTIN, VL53LOX_State);
}


//MUST BE 64 BYTES!
#pragma pack(push, 1)
struct sensor_data_t {
  uint16_t front;
  uint16_t left;
  uint16_t right;
  uint16_t back;

  uint32_t seq;

  uint8_t padding[52];
};
#pragma pack(pop)

static_assert(sizeof(sensor_data_t) == 64, "sensor data must be 64 bytes!");

//MUST BE 64 BYTES!
#pragma pack(push, 1)
struct command_data_t {
  uint8_t data[64];
};
#pragma pack(pop)

static_assert(sizeof(command_data_t) == 64, "command data must be 64 bytes!");

static struct sensor_data_t lidar_data{};
static struct command_data_t command_data{}; //mark as volatile if write lidar data is in interrupt!

uint8_t SEND_LIDAR_COMMAND = 0x41;

void writeLidarData() {
  WAIT_FOR_SLAVE_SELECT();
  SpiDrv::sendCmd(SEND_LIDAR_COMMAND, 1);

  SpiDrv::sendParamNoLen((uint8_t *) &lidar_data, sizeof(struct sensor_data_t), 1);

  SpiDrv::spiSlaveDeselect();



  //Wait the reply elaboration
  SpiDrv::waitForSlaveReady();
  SpiDrv::spiSlaveSelect();

  // Wait for reply
  uint8_t _dataLen = 0; 
  if (!SpiDrv::waitResponseCmd(SEND_LIDAR_COMMAND, PARAM_NUMS_1, (uint8_t*) &command_data, &_dataLen))
  {
      Serial.println("error waitResponse");
  }
  //Serial.print("Response data len:");
  //Serial.println(_dataLen);
  Serial.print("Command: ");
  Serial.println(command_data.data[0]);
  
  SpiDrv::spiSlaveDeselect();
}

int i = 0;
void loop() {
  lidar_data.front = (uint16_t) frontSensor.measureRange();
  lidar_data.right = (uint16_t) rightSensor.measureRange();
  lidar_data.left = (uint16_t) leftSensor.measureRange();
  lidar_data.back = (uint16_t) backSensor.measureRange();

  lidar_data.seq++;

  i++;
  if (i % 10 == 0) {
    Serial.println(lidar_data.front);
    //Serial.println(lidar_data.front);
  }

  writeLidarData();

  delay(1);
}
