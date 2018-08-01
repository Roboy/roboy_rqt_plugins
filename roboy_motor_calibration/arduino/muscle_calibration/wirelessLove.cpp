#include "wirelessLove.hpp"

 const char  TestBuffer[] ="Hello World";
 const int    timestampSize = 2;

WirelessLove::WirelessLove(const char* SSID, const char* PASSWD, IPAddress &broadcastIP):
broadcastIP(broadcastIP){
    uint32_t timoutCounter = 0;

    // check for presence of the WiFiShield:
    if(WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
    }

    connected = WiFi.begin(SSID,PASSWD);
}

uint32_t WirelessLove::getLocalIP()
{
    return  WiFi.localIP();
}

void WirelessLove::printWifiStatus(void)
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

bool WirelessLove::connect(const char* ssid, const char* passwd, IPAddress &bIP){
  broadcastIP = bIP;
  connected = WiFi.begin(ssid,passwd);
}

bool WirelessLove::initUDPSockets(void)
{
    return UDPbroadcast.begin(broadcast_port);
}

bool WirelessLove::broadcast_send(const uint8_t * buffer, size_t size)
{
    if(0 == UDPbroadcast.beginPacket(broadcastIP, broadcast_port))
    {
        Serial.println("Can not connect to the supplied IP or PORT");
        return  false;
    }

    if(size != UDPbroadcast.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data");
    }
    UDPbroadcast.endPacket();
    return true;
}
