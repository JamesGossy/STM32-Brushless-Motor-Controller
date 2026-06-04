#pragma once

#include <stdint.h>

/*
 * hal_spi.h — SPI1 bit-bang-free hardware SPI for DRV835x gate driver
 *
 * Pin mapping:
 *   PA5 -> SPI1_SCK   (AF5)
 *   PA6 -> SPI1_MISO  (AF5)
 *   PA7 -> SPI1_MOSI  (AF5)
 *   PC6 -> GD_CS      (GPIO output, active low)
 *
 * DRV835x SPI: CPOL=0, CPHA=1 (Mode 1 per datasheet — data captured on falling
 * SCLK edge), 16-bit frames, MSB first.
 */

void     SPI_Init(void);
uint16_t SPI_GD_Transfer(uint16_t tx);
