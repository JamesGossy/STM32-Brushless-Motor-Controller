/*
 * flash.h - settings storage in the last flash page.
 */
#pragma once
#include <stddef.h>

/* set while reading, so the NMI handler can tell an ECC error in the
   settings page apart from a real fault */
extern volatile int flash_reading;

int flash_read(void *data, size_t len);
int flash_write(const void *data, size_t len);  /* len must be a multiple of 8 */
