#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>
#include <stddef.h>

uint8_t crc8_update(uint8_t crc, const void *data, size_t length);

#endif