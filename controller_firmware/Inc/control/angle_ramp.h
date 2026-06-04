#pragma once

typedef struct {
    float theta;    /* current angle [0, 2π) */
    float dTheta;   /* angle increment per ISR tick */
    float Ts;       /* control period (s) */
} AngleRamp_t;

void  AngleRamp_init(AngleRamp_t *r, float freqHz, float Ts);
void  AngleRamp_setFreq(AngleRamp_t *r, float freqHz);
float AngleRamp_update(AngleRamp_t *r);
