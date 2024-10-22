#include "packet.h"
#include <SPI.h>
#include "utility/spi_drv.h"
#include "range_finder_block.hpp"

char in_buffer[PACKET_SIZE];
char out_buffer[PACKET_SIZE];
uint32_t last_seq_no = -1;

float steering_requested;
float steering_measured;

uint32_t current_seq_no = 0;
// params
bool param_sensor_update = true;
float param_steering_P = 1.5;
float param_steering_I = 0.0;
float param_steering_D = 0.05;

void initPacket() {
  memset(in_buffer, 0, PACKET_SIZE);
  memset(out_buffer, 0, PACKET_SIZE);

  Packet *p = (Packet *) in_buffer;

  p->seq_no = -1;
}

uint8_t SEND_LIDAR_COMMAND = 0x41;
void fastNetworkStep() {
  WAIT_FOR_SLAVE_SELECT();
  SpiDrv::sendCmd(SEND_LIDAR_COMMAND, 1);

  SpiDrv::sendParamNoLen((uint8_t *) &out_buffer, PACKET_SIZE, 1);

  SpiDrv::spiSlaveDeselect();



  //Wait the reply elaboration
  SpiDrv::waitForSlaveReady();
  SpiDrv::spiSlaveSelect();

  // Wait for reply
  uint8_t _dataLen = 0; 
  if (!SpiDrv::waitResponseCmd(SEND_LIDAR_COMMAND, PARAM_NUMS_1, (uint8_t*) &in_buffer, &_dataLen))
  {
      Serial.println("error waitResponse");
  }
  //Serial.print("Response data len:");
  //Serial.println(_dataLen);


  
  SpiDrv::spiSlaveDeselect();
}

bool parsePacketIfUnique() {
  Packet *p = (Packet *) in_buffer;

  if (last_seq_no == p->seq_no) return false;

  last_seq_no = p->seq_no;

  parsePacket();

  return true;
}

// parse packet in in_buffer
// set appropriate global variables
void parsePacket() {
  Packet *p = (Packet *)in_buffer;

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
  switch (p->type) {
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

void parsePingPacket() {
  // Packet *p = (Packet *) in_buffer;
  buildPingResponsePacket();
  sendResponsePacket();
}
// send sensor response
void parseSensorPacket() {
  buildSensorResponsePacket();
  sendResponsePacket();
}
void parseCommandPacket() {
  /*
  Serial.print("throttle: ");
  Serial.println(p->throttle,5);
  Serial.print("steering: ");
  Serial.println(p->steering,5);
  */
  Packet *p = (Packet *)in_buffer;
  steering = p->steering;
  throttle = p->throttle;

  buildSensorResponsePacket();
  int sent_size = sendResponsePacket();
}

void parseParamPacket() {
  Packet *p = (Packet *)in_buffer;
  if (p->sub_type == 3) {
    param_sensor_update = p->sensor_update;
    param_steering_P = p->steering_P;
    param_steering_I = p->steering_I;
    param_steering_D = p->steering_D;
  }
  // all subtype get parameter printout 3.0
  p = (Packet *)out_buffer;
  p->sensor_update = param_sensor_update;
  p->steering_P = param_steering_P;
  p->steering_I = param_steering_I;
  p->steering_D = param_steering_D;
  buildHeader(3, 0);
  sendResponsePacket();
}

void buildPingResponsePacket() {
  buildHeader(0, 1);
  Packet *p = (Packet *)out_buffer;
}

void buildSensorResponsePacket() {
  buildHeader(2, 0);
  Packet *p = (Packet *)out_buffer;
  p->steering_requested = steering_requested;
  p->steering_measured = steering_measured;

  range_finder_readings_t lidar = get_latest_range_finder_sample();
  p->front_lidar = lidar.front;
  p->left_lidar = lidar.left;
  p->right_lidar = lidar.right;
  p->back_lidar = lidar.back;
}

void buildHeader(uint8_t type, uint8_t subtype) {
  Packet *p = (Packet *)out_buffer;
  p->seq_no = current_seq_no++;
  p->ts = micros();
  p->dest_addr = 0;
  p->src_addr = 1;
  p->type = type;
  p->sub_type = subtype;
}

int sendResponsePacket() {
  if (2 > 1) return 0;
  
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  int size = Udp.write(out_buffer, PACKET_SIZE);
  /*
  if (retval){
    Serial.println("sent success");
  } else {
    Serial.println("sent FAIL");
  }
  */
  Udp.endPacket();
  return size;
}