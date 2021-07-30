#ifndef RPI_FUNCTIONS_HPP
#define RPI_FUNCTIONS_HPP

#include <Arduino.h>

namespace Rpi
{

const uint8_t MSG_TYPE_WRITE = 0x01;
const uint8_t MSG_TYPE_READ_REQ = 0x02;
const size_t MAX_PACKET_LENGTH = 124;

void initWireSlave(const int SDAPin, const int SCLPin, const uint64_t i2c_addr);
void registerReceiveRequest(void (*function)(uint8_t, int, uint8_t*));
void registerReadRequest(int (*function)(uint8_t, uint8_t*&));
void updateWire(void);

} // namespace Rpi

#endif // RPI_FUNCTIONS_HPP