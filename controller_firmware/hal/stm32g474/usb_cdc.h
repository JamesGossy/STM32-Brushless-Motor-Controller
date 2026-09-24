#pragma once
#include <stdint.h>
#include <stddef.h>

void usb_init(void);
int usb_write(const void *data, uint32_t len);
size_t usb_read(uint8_t *d, size_t max);
