#include "board.h"

void board_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    GPIOB->BSRR = (1u << ENC_CS_PIN) | (1u << (ENABLE_PIN + 16));
    GPIOB->PUPDR &= ~(3u << (ENABLE_PIN * 2));
    GPIOC->BSRR = 1u << GD_CS_PIN;
    pin_mode(GPIOA, LED_PIN, PIN_OUT, 0);
    pin_mode(GPIOB, ENABLE_PIN, PIN_OUT, 0);
    pin_mode(GPIOB, ENC_CS_PIN, PIN_OUT, 0);
    pin_mode(GPIOC, GD_CS_PIN, PIN_OUT, 0);

    pin_mode(GPIOA, 5, PIN_AF, 5);   /* SPI1 SCK  */
    pin_mode(GPIOA, 6, PIN_AF, 5);   /* SPI1 MISO */
    pin_mode(GPIOA, 7, PIN_AF, 5);   /* SPI1 MOSI */
    pin_mode(GPIOB, 5, PIN_AF, 9);   /* FDCAN2 RX */
    pin_mode(GPIOB, 6, PIN_AF, 9);   /* FDCAN2 TX */
}
