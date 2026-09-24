/*
 * drv8353.h - TI DRV8353S gate driver over SPI.
 */
#pragma once
#include <stdint.h>

int drv_init(void);
uint16_t drv_read(uint8_t addr);
void drv_write(uint8_t addr, uint16_t data);
void drv_clear_fault(void);
