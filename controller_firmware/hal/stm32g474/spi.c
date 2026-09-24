#include "spi.h"

void spi_mode_enc(void)
{
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (2u << SPI_CR1_BR_Pos) | SPI_CR1_SPE;
}

void spi_mode_drv(void)
{
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (4u << SPI_CR1_BR_Pos) | SPI_CR1_CPHA | SPI_CR1_SPE;
}

void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR2 = 15u << SPI_CR2_DS_Pos;
    spi_mode_enc();
}
