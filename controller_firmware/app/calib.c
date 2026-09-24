/*
 * calib.c - finds everything the control loop needs to know about the sensors.
 *
 *   1. Ramp a d-axis voltage at 0 rad until CAL_CURRENT flows. The sign of the
 *      measured phase A current gives the current sense polarity.
 *   2. Hold, so the rotor settles on the d axis.
 *   3. Sweep CAL_EREVS electrical revolutions forward. The way the encoder
 *      moves gives its direction, and how far it moves checks the pole pairs.
 *   4. Sweep back. The average difference between the encoder angle and the
 *      forced angle over both sweeps is the encoder offset (friction lag
 *      cancels because it has opposite sign in each direction).
 *
 * Runs open loop in voltage mode from the control interrupt.
 */
#include "calib.h"
#include "foc.h"
#include "cfg.h"
#include "hal.h"
#include "svpwm.h"
#include "trig.h"
#include "config.h"
#include <math.h>

enum { RAMP, SETTLE, FORWARD, BACKWARD };

static int phase, ticks, dir;
static float volts, angle, ia_filt, travel, last_raw;
static float sum_sin_fwd, sum_cos_fwd, sum_sin_rev, sum_cos_rev;

/* Begin calibration: reset state and enable the power stage. */
void cal_start(void)
{
    phase = RAMP;
    ticks = 0;
    volts = angle = ia_filt = travel = 0.0f;
    sum_sin_fwd = sum_cos_fwd = sum_sin_rev = sum_cos_rev = 0.0f;
    svpwm_reset();
    foc.state = ST_CAL;
    hal_pwm_enable(1);
}

/* Add one sample of (encoder electrical angle - forced angle) to a circular
   mean, assuming encoder direction d. */
static void accumulate(float raw, int d, float *s, float *c)
{
    float enc = d > 0 ? raw : TWO_PI - raw;
    float err = wrap_2pi(POLE_PAIRS * enc - angle);
    float se, ce;
    sincos_lut(err, &se, &ce);
    *s += se;
    *c += ce;
}

/* Finish: store the results and return to idle. */
static void finish(float raw)
{
    hal_pwm_enable(0);
    cfg.enc_dir = dir;
    cfg.enc_offset = wrap_2pi(atan2f(sum_sin_fwd, sum_cos_fwd));
    cfg.cal_valid = 1;
    foc_pll_reset(dir > 0 ? raw : wrap_2pi(TWO_PI - raw));
    foc.cal_done = 1;
    foc.state = ST_IDLE;
}

/* One calibration step per control interrupt. */
void cal_run(float ia_raw, float raw)
{
    const float sweep = TWO_PI * CAL_EREVS;
    const float step = sweep / CAL_SWEEP_S * TS;
    ticks++;

    switch (phase) {
    case RAMP:
        ia_filt += 0.01f * (ia_raw - ia_filt);
        volts += CAL_VMAX * TS;     /* reaches CAL_VMAX in 1 s */
        if (fabsf(ia_filt) >= CAL_CURRENT) {
            cfg.cur_sign = ia_filt > 0 ? 1 : -1;
            phase = SETTLE;
            ticks = 0;
        } else if (volts > CAL_VMAX) {
            foc_fault(F_CAL);   /* no current: motor or sensing not connected */
            return;
        }
        break;

    case SETTLE:
        if (ticks > (int)(F_PWM / 2)) {
            phase = FORWARD;
            last_raw = raw;
        }
        break;

    case FORWARD:
        angle += step;
        travel += wrap_pi(raw - last_raw);
        last_raw = raw;
        /* direction isn't known yet, so keep sums for both */
        accumulate(raw, 1, &sum_sin_fwd, &sum_cos_fwd);
        accumulate(raw, -1, &sum_sin_rev, &sum_cos_rev);

        if (angle >= sweep) {
            float expected = sweep / POLE_PAIRS;
            if (fabsf(travel) < 0.75f * expected || fabsf(travel) > 1.25f * expected) {
                foc_fault(F_CAL);   /* wrong pole pairs or rotor stuck */
                return;
            }
            dir = travel > 0 ? 1 : -1;
            if (dir < 0) {
                sum_sin_fwd = sum_sin_rev;
                sum_cos_fwd = sum_cos_rev;
            }
            phase = BACKWARD;
        }
        break;

    case BACKWARD:
        angle -= step;
        accumulate(raw, dir, &sum_sin_fwd, &sum_cos_fwd);
        if (angle <= 0.0f) {
            finish(raw);
            return;
        }
        break;
    }

    svpwm_apply(volts, 0.0f, angle, foc.vbus);
}
