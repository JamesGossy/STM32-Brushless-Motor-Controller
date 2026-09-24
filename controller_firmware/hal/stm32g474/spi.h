#pragma once
#include <stdint.h>
#include "stm32g4xx.h"
#include "board.h"

/* SPI1 is shared: MA730 mode 0 @ 21 MHz, DRV8353 mode 1 @ 5.3 MHz */
void spi_init(void);
void spi_mode_enc(void);
void spi_mode_drv(void);

/* MA730 angle read, split so the ISR can do other work while it shifts */
static inline void enc_cs_low(void) { GPIOB->BRR = 1u << ENC_CS_PIN; }
static inline void enc_start(void) { *(volatile uint16_t *)&SPI1->DR = 0; }

static inline uint16_t enc_finish(void)
{
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    uint16_t v = *(volatile uint16_t *)&SPI1->DR;
    GPIOB->BSRR = 1u << ENC_CS_PIN;
    return v;
}
