#include "svpwm.h"
#include "trig.h"
#include "hal.h"
#include "config.h"

float duty[3] = {0.5f, 0.5f, 0.5f};

void svpwm_reset(void)
{
    duty[0] = duty[1] = duty[2] = 0.5f;
    hal_pwm_set(0.5f, 0.5f, 0.5f);
}

/* inverse Park, inverse Clarke, min/max injection */
void svpwm_apply(float vd, float vq, float th, float vbus)
{
    float s, c;
    sincos_lut(wrap_2pi(th), &s, &c);
    float val = vd * c - vq * s;
    float vbe = vd * s + vq * c;
    float v[3] = {val, -0.5f * val + SQRT3_2 * vbe, -0.5f * val - SQRT3_2 * vbe};
    float off = 0.5f * (maxf(v[0], maxf(v[1], v[2])) + minf(v[0], minf(v[1], v[2])));
    float k = 1.0f / maxf(vbus, 1.0f);
    for (int i = 0; i < 3; i++) duty[i] = clampf(0.5f + (v[i] - off) * k, DUTY_MIN, DUTY_MAX);
    hal_pwm_set(duty[0], duty[1], duty[2]);
}
