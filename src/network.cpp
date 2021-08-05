

/*
Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 
 Licensed under the Apache License, Version 2.0 (the "License").
 You may not use this file except in compliance with the License.
 A copy of the License is located at
 
     http://www.apache.org/licenses/LICENSE-2.0
 
 or in the "license" file accompanying this file. This file is distributed
 on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 express or implied. See the License for the specific language governing
 permissions and limitations under the License.
*/
 
// ADDING LIBRARIES

#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>

#include <PubSubClient.h>
#include "network.hpp"

namespace net {

int status = WL_IDLE_STATUS;
char msgReceived = 0;
char rcvdPayload[1024];
  

const char* WIFI_SSID = "S02_WIFI";
const char* WIFI_PASSWORD = "royisaboy";
const char* NODE_NAME = "powersteer";
char* subTopic = "actuator/assist";
int subscribed = 0;

// const char* MQTT_SERVER = "192.168.201.74";
const char* MQTT_SERVER = "192.168.4.1";

WiFiUDP ntpUDP;
WiFiClient espClient;
PubSubClient client(espClient);

void connectToWifi() {
  // Starting WiFi conncetion
  // Serial.println("Connecting to wifi");
#ifdef ESP32
  while (status != WL_CONNECTED)  
  {
    // Serial.print("Attempting to connect to SSID: ");
    // Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. 
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // wait 3 seconds for connection:
    delay(CONNECT_WIFI_DELAY_MS);
  }
#else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)  
  {
    // Serial.print("Attempting to connect to SSID: ");
    // Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. 
    // wait 3 seconds for connection:
    delay(CONNECT_WIFI_DELAY_MS);
  }
#endif
  // Serial.println("Connected to wifi");
}

void connectToMQTT() {
  client.setServer(MQTT_SERVER, 1883);
}

void checkConnection() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(NODE_NAME)) {
      client.subscribe(subTopic);
    } else {
      delay(5000);
    }
  }
}
// PUBLISH FUNCTION DEFINATION
void publishToMQTT(const char* topic, const char jsonPayload[]) {

  boolean success = client.publish(topic, jsonPayload);
  // Serial.print("Success");
  // Serial.println(success);
}

void subscribeToMQTT(char* _topic, void (*cb)(char* topic, byte* payload, unsigned int length)) {
  // subTopic = _topic;
  subscribed = 1;
  client.setCallback(cb);
}

}