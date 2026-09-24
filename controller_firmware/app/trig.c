#include "trig.h"
#include <math.h>

float sin_lut[LUT_N + 1];

void trig_init(void)
{
    for (int i = 0; i <= LUT_N; i++) sin_lut[i] = sinf(TWO_PI * i / LUT_N);
}
