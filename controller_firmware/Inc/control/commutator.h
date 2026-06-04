#pragma once

#include <stdint.h>

/*
 * 6-step commutation table for trapezoidal BLDC.
 *
 * Each step energises two phases and floats the third:
 *   +phase  -> high-side ON, low-side OFF  (PWM duty applied)
 *   -phase  -> high-side OFF, low-side ON  (PWM duty applied, complementary)
 *   float   -> both high and low sides OFF (channel disabled)
 *
 * Step advances every time the electrical angle crosses a 60° boundary.
 * In open-loop mode the angle is integrated from g_freqHz, exactly as
 * the old AngleRamp did, but the output is a step index not a sine.
 */

typedef struct {
    float    angle;    /* accumulated electrical angle, 0..2π */
    uint8_t  step;     /* current commutation step 0..5 */
} Commutator_t;

void Commutator_init  (Commutator_t *c);
void Commutator_update(Commutator_t *c, float freqHz, float ts);
