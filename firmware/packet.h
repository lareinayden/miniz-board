#ifndef PACKET_H
#define PACKET_H
#include <Arduino.h>
#include "firmware.h"
#include "network.h"

#define PACKET_SIZE 64
extern char in_buffer[PACKET_SIZE];
extern char out_buffer[PACKET_SIZE];
// sensors
// steering_requested differs from steering in that it is synchronized to steering_measured
extern float steering_requested;
extern float steering_measured;

extern uint32_t current_seq_no;
// params
extern bool  param_sensor_update;
extern float param_steering_P;
extern float param_steering_I;
extern float param_steering_D;

typedef struct {
  uint32_t seq_no;
  uint32_t ts;
  uint8_t dest_addr;
  uint8_t src_addr;
  uint8_t type;
  uint8_t sub_type;
  
  uint16_t front_lidar;
  uint16_t back_lidar;
  uint16_t left_lidar;
  uint16_t right_lidar;
  
  union {
    // general
    char payload[44];
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

void fastNetworkStep();
bool parsePacketIfUnique();
void initPacket();

// parse packet in in_buffer
// set appropriate global variables
void parsePacket();
void parsePingPacket();
void parseSensorPacket();
void parseCommandPacket();
void parseParamPacket();
void buildPingResponsePacket();
void buildSensorResponsePacket();
void buildHeader(uint8_t type, uint8_t subtype);
int sendResponsePacket();

#endif
