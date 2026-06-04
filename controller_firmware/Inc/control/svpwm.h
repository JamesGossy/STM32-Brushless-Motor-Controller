#pragma once

/*
 * Space-vector PWM modulation.
 * Converts three-phase voltages to duty cycles in [0, 1].
 * Uses midpoint-clamp (min+max)/2 injection — no lookup table needed.
 */
void SVPWM_calc(float Va, float Vb, float Vc, float Vdc,
                float *dutyA, float *dutyB, float *dutyC);
