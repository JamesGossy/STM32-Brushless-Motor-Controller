#pragma once
#include "stm32g4xx.h"

#define LED_PIN     4   /* PA4  */
#define ENABLE_PIN  4   /* PB4  */
#define ENC_CS_PIN  11  /* PB11 */
#define GD_CS_PIN   10  /* PC10 */

enum { PIN_IN, PIN_OUT, PIN_AF, PIN_ANALOG };

static inline void pin_mode(GPIO_TypeDef *g, uint32_t n, uint32_t mode, uint32_t af)
{
    g->MODER = (g->MODER & ~(3u << (n * 2))) | (mode << (n * 2));
    if (mode == PIN_AF) {
        g->OSPEEDR |= 3u << (n * 2);
        g->AFR[n >> 3] = (g->AFR[n >> 3] & ~(0xFu << ((n & 7) * 4))) | (af << ((n & 7) * 4));
    }
}

void board_init(void);
