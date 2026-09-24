/*
 * svpwm.c - space vector modulation.
 *
 * Uses the min/max injection method: add a common-mode offset so the phase
 * voltages are centred in the bus. This gives the same result as classic
 * sector-based SVPWM with much less code.
 */
#include "svpwm.h"
#include "trig.h"
#include "hal.h"
#include "config.h"

float duty[3] = {0.5f, 0.5f, 0.5f};

/* Set all phases to 50 % (zero voltage). */
void svpwm_reset(void)
{
    duty[0] = duty[1] = duty[2] = 0.5f;
    hal_pwm_set(0.5f, 0.5f, 0.5f);
}

/* Apply vd/vq (volts) at electrical angle theta. */
void svpwm_apply(float vd, float vq, float theta, float vbus)
{
    float s, c;
    sincos_lut(wrap_2pi(theta), &s, &c);

    /* inverse Park */
    float valpha = vd * c - vq * s;
    float vbeta = vd * s + vq * c;

    /* inverse Clarke */
    float v[3] = {
        valpha,
        -0.5f * valpha + SQRT3_2 * vbeta,
        -0.5f * valpha - SQRT3_2 * vbeta,
    };

    /* centre the three phase voltages in the bus */
    float offset = 0.5f * (maxf(v[0], maxf(v[1], v[2])) + minf(v[0], minf(v[1], v[2])));
    float inv_vbus = 1.0f / maxf(vbus, 1.0f);

    for (int i = 0; i < 3; i++)
        duty[i] = clampf(0.5f + (v[i] - offset) * inv_vbus, DUTY_MIN, DUTY_MAX);

    hal_pwm_set(duty[0], duty[1], duty[2]);
}
