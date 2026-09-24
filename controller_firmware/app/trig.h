/*
 * trig.h - fast sine/cosine from a lookup table.
 *
 * libm sinf/cosf are too slow for the 20 kHz loop, so a 1024 point table
 * with linear interpolation is used instead (error < 5e-5).
 */
#pragma once
#include "mathx.h"

#define LUT_N 1024

extern float sin_lut[LUT_N + 1];

void trig_init(void);

/* Sine and cosine of x, where x is in [0, 2pi). */
static inline void sincos_lut(float x, float *s, float *c)
{
    float f = x * (LUT_N / TWO_PI);
    int i = (int)f;
    float frac = f - i;

    i &= LUT_N - 1;
    int j = (i + LUT_N / 4) & (LUT_N - 1);   /* cos(x) = sin(x + pi/2) */

    *s = sin_lut[i] + frac * (sin_lut[i + 1] - sin_lut[i]);
    *c = sin_lut[j] + frac * (sin_lut[j + 1] - sin_lut[j]);
}
