/*
 * board.h - pin assignments and a small GPIO helper.
 * Pinout from controller_firmware/stm32_esc_r1_2.ioc and the schematic.
 */
#pragma once
#include "stm32g4xx.h"

#define LED_PIN     4   /* PA4  heartbeat LED */
#define ENABLE_PIN  4   /* PB4  DRV8353 ENABLE */
#define ENC_CS_PIN  11  /* PB11 MA730 chip select */
#define GD_CS_PIN   10  /* PC10 DRV8353 chip select */

enum { PIN_IN, PIN_OUT, PIN_AF, PIN_ANALOG };

/* Set a pin's mode, and its alternate function number when mode is PIN_AF. */
static inline void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t af)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2))) | (mode << (pin * 2));
    if (mode == PIN_AF) {
        port->OSPEEDR |= 3u << (pin * 2);
        uint32_t shift = (pin & 7) * 4;
        port->AFR[pin >> 3] = (port->AFR[pin >> 3] & ~(0xFu << shift)) | (af << shift);
    }
}

void board_init(void);
