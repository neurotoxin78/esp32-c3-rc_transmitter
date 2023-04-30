#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "ui/ui.h"

extern const uint16_t port;
extern IPAddress addr;
extern AsyncUDP udp;

void parsePacket(AsyncUDPPacket packet);
void udpConnect(IPAddress ipAddress, uint16_t udpPort);
void sendPacket(String message, IPAddress address, uint16_t port);
