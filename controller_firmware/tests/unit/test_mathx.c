/*
 * test_mathx.c - unit tests for the maths helpers and the sine table.
 */
#include "test.h"
#include "mathx.h"
#include "trig.h"

/* Angles wrap into [0, 2pi) and [-pi, pi). */
static void wraps(void)
{
    CHECK_NEAR(wrap_2pi(-0.5f), TWO_PI - 0.5f, 1e-5);
    CHECK_NEAR(wrap_2pi(7.0f), 7.0f - TWO_PI, 1e-5);
    CHECK_NEAR(wrap_2pi(3.0f * TWO_PI + 1.0f), 1.0f, 1e-4);
    CHECK_NEAR(wrap_pi(4.0f), 4.0f - TWO_PI, 1e-5);
    CHECK_NEAR(wrap_pi(-4.0f), -4.0f + TWO_PI, 1e-5);
    CHECK(wrap_pi(PI_F) < PI_F);
}

/* Clamp and min/max helpers. */
static void clamps(void)
{
    CHECK(clampf(5.0f, -1.0f, 1.0f) == 1.0f);
    CHECK(clampf(-5.0f, -1.0f, 1.0f) == -1.0f);
    CHECK(clampf(0.3f, -1.0f, 1.0f) == 0.3f);
    CHECK(maxf(1.0f, 2.0f) == 2.0f && minf(1.0f, 2.0f) == 1.0f);
}

/* Unsaturated PI: output = ff + P + I. */
static void pi_linear_region(void)
{
    pi_t p = {2.0f, 100.0f, 0.0f};
    float u = pi_run(&p, 1.0f, 0.5f, -10.0f, 10.0f, 0.01f);
    CHECK_NEAR(u, 0.5f + 2.0f + 1.0f, 1e-6);
    CHECK_NEAR(p.i, 1.0f, 1e-6);
}

/* A saturated PI must not wind up and must recover immediately. */
static void pi_anti_windup(void)
{
    pi_t p = {1.0f, 1000.0f, 0.0f};
    for (int i = 0; i < 1000; i++) CHECK(pi_run(&p, 5.0f, 0.0f, -1.0f, 1.0f, 0.001f) <= 1.0f);
    CHECK(p.i <= 1.0f);
    /* error reverses: output leaves the limit on the very next step */
    CHECK(pi_run(&p, -0.5f, 0.0f, -1.0f, 1.0f, 0.001f) < 1.0f);
}

/* Integration stops into the rail but continues out of it. */
static void pi_freezes_when_saturated(void)
{
    pi_t p = {0.0f, 10.0f, 0.9f};
    CHECK(pi_run(&p, 1.0f, 0.0f, -1.0f, 1.0f, 0.1f) == 1.0f);
    CHECK_NEAR(p.i, 0.9f, 1e-6);           /* no integration into the rail */
    pi_run(&p, -1.0f, 0.0f, -1.0f, 1.0f, 0.1f);
    CHECK_NEAR(p.i, -0.1f, 1e-6);          /* integrating back out is allowed */
}

/* Lookup-table sine/cosine within 5e-5 of libm. */
static void sincos_accuracy(void)
{
    trig_init();
    float worst = 0;
    for (int k = 0; k < 10000; k++) {
        float x = TWO_PI * k / 10000.0f, s, c;
        sincos_lut(x, &s, &c);
        worst = maxf(worst, fabsf(s - sinf(x)));
        worst = maxf(worst, fabsf(c - cosf(x)));
    }
    CHECK(worst < 5e-5f);
}

int main(void)
{
    test_case t[] = {T(wraps), T(clamps), T(pi_linear_region), T(pi_anti_windup),
                     T(pi_freezes_when_saturated), T(sincos_accuracy)};
    return run_tests(t, sizeof t / sizeof t[0]);
}
