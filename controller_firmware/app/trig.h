#pragma once
#include "mathx.h"

#define LUT_N 1024
extern float sin_lut[LUT_N + 1];

void trig_init(void);

/* x in [0, 2pi) */
static inline void sincos_lut(float x, float *s, float *c)
{
    float f = x * (LUT_N / TWO_PI);
    int i = (int)f;
    float fr = f - i;
    i &= LUT_N - 1;
    int j = (i + LUT_N / 4) & (LUT_N - 1);
    *s = sin_lut[i] + fr * (sin_lut[i + 1] - sin_lut[i]);
    *c = sin_lut[j] + fr * (sin_lut[j + 1] - sin_lut[j]);
}
