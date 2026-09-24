/*
 * spi.c - SPI1 configuration for the two devices on the bus.
 */
#include "spi.h"

/* Mode 0, 170 MHz / 8 = 21 MHz (MA730 max 25 MHz). */
void spi_mode_encoder(void)
{
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (2u << SPI_CR1_BR_Pos) | SPI_CR1_SPE;
}

/* Mode 1, 170 MHz / 32 = 5.3 MHz (DRV8353 max 10 MHz). */
void spi_mode_driver(void)
{
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (4u << SPI_CR1_BR_Pos) | SPI_CR1_CPHA | SPI_CR1_SPE;
}

/* 16-bit frames, encoder mode by default. */
void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR2 = 15u << SPI_CR2_DS_Pos;
    spi_mode_encoder();
}
