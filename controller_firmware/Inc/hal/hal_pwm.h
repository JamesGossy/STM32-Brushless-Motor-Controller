#pragma once

#include <stdint.h>

void PWM_EnableOutput(void);
void PWM_DisableOutput(void);

/* duty in [0.0, 1.0] — sets all three CCRs independently (FOC / debug use) */
void PWM_SetDuty(float dutyA, float dutyB, float dutyC);

/* 6-step commutation: energises the correct phase pair for the given step (0-5)
 * at the given duty cycle; floats the third phase by disabling its CCER bits. */
void PWM_SetStep(uint8_t step, float duty);
