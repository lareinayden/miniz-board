#ifndef NETWORK_H
#define NETWORK_H
#include <WiFiNINA.h>
#include <WiFiUdp.h>

extern char ssid[];
extern char pass[];
extern int status;
extern WiFiUDP Udp;

void printWifiData();
void printCurrentNet();
void printMacAddress(byte mac[]);

#endif NETWORK_H
