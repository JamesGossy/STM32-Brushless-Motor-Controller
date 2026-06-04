#pragma once

#include <stdint.h>

void SystemClock_Config(void);
void GPIO_Init(void);
void TIM1_Init(void);

/* Peripheral fault input — nFAULT is active-low on PB0 */
#define GD_NFAULT_ACTIVE()  (!(GPIOB->IDR & (1u << 0)))
