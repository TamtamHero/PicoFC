#ifndef MISC_H
#define MISC_H

#include <stdint.h>
#include <stddef.h>

uint8_t crc8_update(uint8_t crc, const void *data, size_t length);

#endif