/*
 * mathx.h - small maths helpers shared by the control code.
 *
 * Angle wrapping, clamping and a PI controller with anti-windup.
 */
#pragma once

#define TWO_PI    6.28318531f
#define PI_F      3.14159265f
#define SQRT3_2   0.86602540f
#define INV_SQRT3 0.57735027f

/* Wrap an angle into [0, 2pi). */
static inline float wrap_2pi(float x)
{
    while (x >= TWO_PI) x -= TWO_PI;
    while (x < 0.0f) x += TWO_PI;
    return x;
}

/* Wrap an angle into [-pi, pi). */
static inline float wrap_pi(float x)
{
    while (x >= PI_F) x -= TWO_PI;
    while (x < -PI_F) x += TWO_PI;
    return x;
}

static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

typedef struct {
    float kp, ki;   /* gains */
    float i;        /* integrator state */
} pi_t;

/* One PI step with feed-forward. The integrator stops while the output is
   saturated in the direction of the error, so it never winds up. */
static inline float pi_run(pi_t *p, float err, float ff, float lo, float hi, float ts)
{
    float di = p->ki * ts * err;
    float out = ff + p->kp * err + p->i + di;

    if (out > hi) {
        out = hi;
        if (err < 0) p->i += di;
    } else if (out < lo) {
        out = lo;
        if (err > 0) p->i += di;
    } else {
        p->i += di;
    }

    p->i = clampf(p->i, lo - ff, hi - ff);
    return out;
}
