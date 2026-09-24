#pragma once

#define TWO_PI    6.28318531f
#define PI_F      3.14159265f
#define SQRT3_2   0.86602540f
#define INV_SQRT3 0.57735027f

static inline float wrap_2pi(float x)
{
    while (x >= TWO_PI) x -= TWO_PI;
    while (x < 0.0f) x += TWO_PI;
    return x;
}

static inline float wrap_pi(float x)
{
    while (x >= PI_F) x -= TWO_PI;
    while (x < -PI_F) x += TWO_PI;
    return x;
}

static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

typedef struct { float kp, ki, i; } pi_t;

/* PI with feedforward and clamping anti-windup */
static inline float pi_run(pi_t *p, float e, float ff, float lo, float hi, float ts)
{
    float di = p->ki * ts * e;
    float u = ff + p->kp * e + p->i + di;
    if (u > hi) { u = hi; if (e < 0) p->i += di; }
    else if (u < lo) { u = lo; if (e > 0) p->i += di; }
    else p->i += di;
    p->i = clampf(p->i, lo - ff, hi - ff);
    return u;
}
