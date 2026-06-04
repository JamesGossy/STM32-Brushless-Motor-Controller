/*
 * hal_uart.c — LPUART1 at 115200 baud (SYSCLK = 170 MHz)
 *
 * Pin mapping (NUCLEO-G474RE VCP via ST-Link):
 *   PA2 -> LPUART1_TX  (AF12)
 *   PA3 -> LPUART1_RX  (AF12)
 *
 * BRR calculation for LPUART1:
 *   BRR = (256 * fCLK) / baud = (256 * 170000000) / 115200 = 377,955 (~0x5C444)
 */

#include "hal/hal_uart.h"
#include "stm32g474xx.h"

void UART_Init(void)
{
    /* LPUART1 clock from PCLK1 (170 MHz) */
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    (void)RCC->APB1ENR2;

    /* PA2 -> AF12 (LPUART1_TX), PA3 -> AF12 (LPUART1_RX) */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;

    GPIOA->MODER = (GPIOA->MODER
        & ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk))
        | (2u << GPIO_MODER_MODE2_Pos)
        | (2u << GPIO_MODER_MODE3_Pos);

    GPIOA->AFR[0] = (GPIOA->AFR[0]
        & ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk))
        | (12u << GPIO_AFRL_AFSEL2_Pos)
        | (12u << GPIO_AFRL_AFSEL3_Pos);

    /* 8N1, no parity, FIFO disabled */
    LPUART1->CR1 = 0u;
    LPUART1->CR2 = 0u;
    LPUART1->CR3 = 0u;

    /* BRR = 256 * 170000000 / 921600 = 47,243 */
    LPUART1->BRR = 47243u;

    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_PutChar(char c)
{
    while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF));
    LPUART1->TDR = (uint8_t)c;
}

void UART_WriteBytes(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF));
        LPUART1->TDR = buf[i];
    }
}

/* newlib printf hook */
int __io_putchar(int ch)
{
    if (ch == '\n')
        UART_PutChar('\r');
    UART_PutChar((char)ch);
    return ch;
}
