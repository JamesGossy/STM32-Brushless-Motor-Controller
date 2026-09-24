#pragma once

extern float duty[3];

void svpwm_reset(void);
void svpwm_apply(float vd, float vq, float th, float vbus);
