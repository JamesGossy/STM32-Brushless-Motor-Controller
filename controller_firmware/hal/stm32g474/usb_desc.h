#pragma once
#include <stdint.h>

/* returns descriptor length, 0 if unknown */
int usb_get_descriptor(uint16_t val, const uint8_t **p);
