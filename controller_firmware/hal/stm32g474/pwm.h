#pragma once
#include <stdint.h>
#include "stm32g4xx.h"
#include "config.h"

void pwm_init(void);
void pwm_start(void);
void pwm_on(void);
void pwm_off(void);

/* high-side duty per phase; low side is on around the counter wrap */
static inline void pwm_set(float da, float db, float dc)
{
    const float h = PWM_PER * 0.5f;
    uint32_t a = (uint32_t)(da * h), b = (uint32_t)(db * h), c = (uint32_t)(dc * h);
    uint32_t m = PWM_PER / 2;
    HRTIM1->sTimerxRegs[0].CMP1xR = m - a;
    HRTIM1->sTimerxRegs[0].CMP2xR = m + a;
    HRTIM1->sTimerxRegs[4].CMP1xR = m - b;
    HRTIM1->sTimerxRegs[4].CMP2xR = m + b;
    HRTIM1->sTimerxRegs[5].CMP1xR = m - c;
    HRTIM1->sTimerxRegs[5].CMP2xR = m + c;
}
