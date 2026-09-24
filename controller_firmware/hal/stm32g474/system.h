#pragma once
#include <stdint.h>
#include "stm32g4xx.h"

extern volatile uint32_t ms_ticks;

void system_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void wdg_init(void);
static inline void wdg_kick(void) { IWDG->KR = 0xAAAA; }
