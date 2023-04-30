#include "network.h"

const uint16_t port = 50000;
IPAddress addr(10, 175, 1, 252);
AsyncUDP udp;

void udpConnect(IPAddress ipAddress, uint16_t udpPort)
{
    if (udp.connect(ipAddress, udpPort))
    {
        Serial.println("UDP подключён");
        udp.onPacket(parsePacket);
    }
}

void parsePacket(AsyncUDPPacket packet)
{
    // Выводи в последовательный порт все полученные данные
    Serial.write(packet.data(), packet.length());
    Serial.println();
}

void sendPacket(String message, IPAddress address, uint16_t port)
{
    if (udp.connect(address, port))
    {
        udp.print(message);
        udp.close();
    }
}