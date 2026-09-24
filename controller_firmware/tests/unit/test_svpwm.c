/*
 * test_svpwm.c - unit tests for the space vector modulator.
 */
#include "test.h"
#include "svpwm.h"
#include "trig.h"
#include "config.h"

/* Zero voltage gives 50 % on every phase. */
static void zero_vector_is_half(void)
{
    trig_init();
    svpwm_apply(0, 0, 1.0f, 24.0f);
    for (int i = 0; i < 3; i++) CHECK_NEAR(duty[i], 0.5, 1e-6);
}

/* line-to-line voltages from duties must reproduce the commanded vector */
/* The duties must reproduce the commanded dq vector at any angle. */
static void reproduces_vector(void)
{
    trig_init();
    const float vbus = 24.0f, vd = 3.0f, vq = 6.0f;
    for (int k = 0; k < 36; k++) {
        float th = TWO_PI * k / 36.0f;
        svpwm_apply(vd, vq, th, vbus);
        float va = (2 * duty[0] - duty[1] - duty[2]) / 3.0f * vbus;
        float vb = (duty[1] - duty[2]) * vbus / 1.7320508f;
        float d = va * cosf(th) + vb * sinf(th), q = -va * sinf(th) + vb * cosf(th);
        CHECK_NEAR(d, vd, 0.01);
        CHECK_NEAR(q, vq, 0.01);
    }
}

/* The largest linear vector fits inside the duty limits. */
static void linear_limit_inside_duty_range(void)
{
    trig_init();
    const float vbus = 24.0f, vmax = MOD_MAX * vbus / 1.7320508f;
    for (int k = 0; k < 360; k++) {
        svpwm_apply(0, vmax, TWO_PI * k / 360.0f, vbus);
        for (int i = 0; i < 3; i++) CHECK(duty[i] > DUTY_MIN && duty[i] < DUTY_MAX);
    }
}

/* Asking for too much voltage clamps the duties. */
static void overmodulation_clamps(void)
{
    trig_init();
    svpwm_apply(0, 100.0f, 0.3f, 24.0f);
    for (int i = 0; i < 3; i++) CHECK(duty[i] >= DUTY_MIN && duty[i] <= DUTY_MAX);
}

int main(void)
{
    test_case t[] = {T(zero_vector_is_half), T(reproduces_vector), T(linear_limit_inside_duty_range),
                     T(overmodulation_clamps)};
    return run_tests(t, sizeof t / sizeof t[0]);
}
