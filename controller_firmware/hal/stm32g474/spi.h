/*
 * spi.h - SPI1, shared by the MA730 encoder and the DRV8353 gate driver.
 *
 * The encoder uses mode 0 at 21 MHz and is read every control cycle.
 * The gate driver uses mode 1 at 5.3 MHz and is only accessed from the main
 * loop with interrupts off, switching the mode and back around each transfer.
 */
#pragma once
#include <stdint.h>
#include "stm32g4xx.h"
#include "board.h"

void spi_init(void);
void spi_mode_encoder(void);
void spi_mode_driver(void);

/* The encoder read is split in three so the ISR can do other work while
   CS settles and while the 16 bits shift in. */
static inline void enc_cs_low(void) { GPIOB->BRR = 1u << ENC_CS_PIN; }
static inline void enc_start(void) { *(volatile uint16_t *)&SPI1->DR = 0; }

static inline uint16_t enc_finish(void)
{
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    uint16_t angle = *(volatile uint16_t *)&SPI1->DR;
    GPIOB->BSRR = 1u << ENC_CS_PIN;
    return angle;
}
