/*
 * hal_spi.c — SPI1 for DRV835x gate driver
 *
 * DRV835x SPI timing (datasheet section 7.6):
 *   - 16-bit frames, MSB first
 *   - CPOL=1, CPHA=1 (SPI Mode 1): SCLK idles high, data captured on falling edge
 *   - Max SCLK: 10 MHz — we use SPI1 at 170/32 = ~5.3 MHz (BR=4, /32)
 *   - nSCS must be low for the entire 16-bit transaction
 */

#include "hal/hal_spi.h"
#include "stm32g474xx.h"

/* GD chip-select helpers */
#define GD_CS_LOW()   (GPIOC->BSRR = GPIO_BSRR_BR6)
#define GD_CS_HIGH()  (GPIOC->BSRR = GPIO_BSRR_BS6)

void SPI_Init(void)
{
    /* Enable clocks */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->APB2ENR;

    /* PA5 -> SCK (AF5), PA6 -> MISO (AF5), PA7 -> MOSI (AF5) */
    GPIOA->MODER = (GPIOA->MODER
        & ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk))
        | (2u << GPIO_MODER_MODE5_Pos)
        | (2u << GPIO_MODER_MODE6_Pos)
        | (2u << GPIO_MODER_MODE7_Pos);

    GPIOA->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED5_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED6_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED7_Pos);

    /* Pull-up on MISO for open-drain SDO line */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD6_Msk)
                 | (1u << GPIO_PUPDR_PUPD6_Pos);

    GPIOA->AFR[0] = (GPIOA->AFR[0]
        & ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk))
        | (5u << GPIO_AFRL_AFSEL5_Pos)
        | (5u << GPIO_AFRL_AFSEL6_Pos)
        | (5u << GPIO_AFRL_AFSEL7_Pos);

    /* PC6 -> GD_CS: GPIO output, start high (deselected) */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE6_Msk)
                 | (1u << GPIO_MODER_MODE6_Pos);
    GPIOC->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED6_Pos);
    GD_CS_HIGH();

    /* SPI1 config:
     *   Master mode, CPOL=0, CPHA=1 (Mode 1 per DRV835x datasheet)
     *   16-bit data frame
     *   BR = 100b (/32) -> 170/32 = 5.3 MHz (< 10 MHz max)
     *   MSB first, SSM+SSI (software CS management) */
    SPI1->CR1 = SPI_CR1_MSTR
              | SPI_CR1_CPHA
              | (4u << SPI_CR1_BR_Pos)   /* /32 */
              | SPI_CR1_SSM
              | SPI_CR1_SSI;

    SPI1->CR2 = (0xFu << SPI_CR2_DS_Pos)   /* 16-bit data size */
              | SPI_CR2_FRXTH;              /* RXNE on 1 frame */

    SPI1->CR1 |= SPI_CR1_SPE;
}

uint16_t SPI_GD_Transfer(uint16_t tx)
{
    GD_CS_LOW();

    /* Wait until TX FIFO has space, then write */
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = tx;

    /* Wait for RX FIFO to have data */
    while (!(SPI1->SR & SPI_SR_RXNE));
    uint16_t rx = (uint16_t)SPI1->DR;

    /* Wait for bus idle before releasing CS */
    while (SPI1->SR & SPI_SR_BSY);

    GD_CS_HIGH();
    return rx;
}
