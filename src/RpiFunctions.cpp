#include "RpiFunctions.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#ifdef ESP32
#include <WireSlave.h>
#endif


namespace Rpi 
{
uint8_t data[MAX_PACKET_LENGTH];
uint8_t* dataToWrite;
uint32_t messageLength = 0;
uint8_t messageType = 0; // 0 = write to current device, 1 = read req from current device
uint8_t registerCode = 0x00;
size_t packetCount = 0;
size_t packetCounter = 0;
void (*receiveFunc)(uint8_t, int, uint8_t*);
int (*readReqFunc)(uint8_t, uint8_t*&);

static void _receiveEvent(int howMany);
static void _requestEvent(void);

void initWireSlave(const int SDAPin, const int SCLPin, const uint64_t i2c_addr)
{
#ifdef ESP32
  bool success = WireSlave.begin(SDAPin,SCLPin, i2c_addr);
  if (!success) {
    Serial.println("I2C slave init failed");
    while(1) delay(100);
  }
  Serial.println("Init success");
#else
  Wire.begin(SDAPin, SCLPin, i2c_addr);
  Serial.println("Init success");
#endif
}

void registerReceiveRequest(void (*function)(uint8_t, int, uint8_t*))
{
#ifdef ESP32
  WireSlave.onReceive(_receiveEvent);
#else
  Wire.onReceive(_receiveEvent);
#endif
  receiveFunc = function;
}

void registerReadRequest(int (*function)(uint8_t, uint8_t*&)) {
#ifdef ESP32
  WireSlave.onRequest(_requestEvent);
#else
  Wire.onRequest(_requestEvent);
#endif // ESP32
  readReqFunc = function;
}

void updateWire(void) {
#ifdef ESP32
  WireSlave.update();
#endif
}

static void _receiveEvent(int howMany) {
  Serial.println("Receive");

#ifdef ESP32
  if (WireSlave.available() && howMany >=2) {
    registerCode = WireSlave.read();
    messageType = WireSlave.read();
  }
#else
  if (Wire.available() && howMany >=2) {
    registerCode = Wire.read();
    messageType = Wire.read();
  }
#endif // ESP32

  if (messageType != MSG_TYPE_WRITE) return;
  packetCounter = 0;

#ifdef ESP32
  while(WireSlave.available() && packetCounter < MAX_PACKET_LENGTH) {
    data[packetCounter++] = WireSlave.read();
  }
#else
  while(Wire.available() && packetCounter < MAX_PACKET_LENGTH) {
    data[packetCounter++] = Wire.read();
  }
#endif // ESP32
  receiveFunc(registerCode, howMany - 2, data); 
}

static void _requestEvent(void) {
  if (messageType != MSG_TYPE_READ_REQ) return;
  Serial.println("Requested read");
  int numBytes = readReqFunc(registerCode, dataToWrite);
#ifdef ESP32
  WireSlave.write(registerCode);
  for (size_t i = 0; i < numBytes; ++i) {
    WireSlave.write(dataToWrite[i]);
  }
#else
  Wire.write(registerCode);
  for (size_t i = 0; i < numBytes; ++i) {
    Wire.write(dataToWrite[i]);
  }
#endif // ESP32
}

} // namespace Rpi