/*
 * pwm.h - three-phase PWM on HRTIM timers A, E and F.
 */
#pragma once
#include <stdint.h>
#include "stm32g4xx.h"
#include "config.h"

void pwm_init(void);
void pwm_start(void);
void pwm_on(void);
void pwm_off(void);

/* Set the high-side duty of each phase (0..1). The high side is on between
   CMP1 and CMP2, centred in the period, so the low side is on around the
   counter wrap where the ADC samples. Preloaded: applies next period. */
static inline void pwm_set(float da, float db, float dc)
{
    const float half = PWM_PER * 0.5f;
    const uint32_t mid = PWM_PER / 2;
    uint32_t a = (uint32_t)(da * half), b = (uint32_t)(db * half), c = (uint32_t)(dc * half);

    HRTIM1->sTimerxRegs[0].CMP1xR = mid - a;    /* timer A = phase A */
    HRTIM1->sTimerxRegs[0].CMP2xR = mid + a;
    HRTIM1->sTimerxRegs[4].CMP1xR = mid - b;    /* timer E = phase B */
    HRTIM1->sTimerxRegs[4].CMP2xR = mid + b;
    HRTIM1->sTimerxRegs[5].CMP1xR = mid - c;    /* timer F = phase C */
    HRTIM1->sTimerxRegs[5].CMP2xR = mid + c;
}
