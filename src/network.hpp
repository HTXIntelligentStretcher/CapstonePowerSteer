#ifndef NETWORK_HPP 
#define NETWORK_HPP

#include <WiFiUdp.h>
#include <WiFiClientSecure.h>

#include <NTPClient.h>

namespace net {

const int CONNECT_WIFI_DELAY_MS = 3000;

void connectToWifi();

void checkConnection();

void reconnect();

void connectToMQTT();

void publishToMQTT(const char* topic, const char jsonPayload[]);

void subscribeToMQTT(char* _topic, void (*cb)(char* topic, byte* payload, unsigned int length));
}
#endif