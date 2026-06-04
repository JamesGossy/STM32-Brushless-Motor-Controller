#include "control/angle_ramp.h"
#include <math.h>

#define TWO_PI  6.28318530717958647f

void AngleRamp_init(AngleRamp_t *r, float freqHz, float Ts)
{
    r->theta  = 0.0f;
    r->Ts     = Ts;
    r->dTheta = TWO_PI * freqHz * Ts;
}

void AngleRamp_setFreq(AngleRamp_t *r, float freqHz)
{
    r->dTheta = TWO_PI * freqHz * r->Ts;
}

float AngleRamp_update(AngleRamp_t *r)
{
    r->theta += r->dTheta;
    if (r->theta >= TWO_PI)
        r->theta -= TWO_PI;
    else if (r->theta < 0.0f)
        r->theta += TWO_PI;
    return r->theta;
}
