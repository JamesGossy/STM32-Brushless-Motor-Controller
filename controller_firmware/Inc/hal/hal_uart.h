#pragma once

#include <stdint.h>

/*
 * hal_uart.h — LPUART1 on PA2 (TX) / PA3 (RX), 115200 baud, 170 MHz SYSCLK
 *
 * After UART_Init(), printf() routes here via __io_putchar().
 */

void UART_Init(void);
void UART_PutChar(char c);
void UART_WriteBytes(const uint8_t *buf, uint32_t len);
