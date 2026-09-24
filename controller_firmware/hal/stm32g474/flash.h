#pragma once
#include <stddef.h>

extern volatile int flash_reading;

int flash_read(void *d, size_t n);
int flash_write(const void *d, size_t n);   /* n must be a multiple of 8 */
