/*
 * svpwm.h - turns a dq voltage command into three PWM duty cycles.
 */
#pragma once

/* Duties currently applied to phases A, B, C (0..1, high side on-time). */
extern float duty[3];

void svpwm_reset(void);
void svpwm_apply(float vd, float vq, float theta, float vbus);
