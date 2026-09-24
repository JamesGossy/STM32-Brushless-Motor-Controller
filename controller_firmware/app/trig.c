/*
 * trig.c - builds the sine lookup table used by sincos_lut().
 */
#include "trig.h"
#include <math.h>

float sin_lut[LUT_N + 1];

/* Fill the table once at boot. The extra last entry avoids a wrap check
   when interpolating. */
void trig_init(void)
{
    for (int i = 0; i <= LUT_N; i++)
        sin_lut[i] = sinf(TWO_PI * i / LUT_N);
}
