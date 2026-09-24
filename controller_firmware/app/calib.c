#include "calib.h"
#include "foc.h"
#include "cfg.h"
#include "hal.h"
#include "svpwm.h"
#include "trig.h"
#include "config.h"
#include <math.h>

/* 1. ramp d-axis voltage at 0 rad until CAL_CURRENT flows -> current sign
   2. settle, then sweep CAL_EREVS electrical revs forward and back
   3. forward travel gives encoder direction and a pole pair sanity check
   4. circular mean of (p*theta_m - theta_forced) gives the offset */

enum { CAL_RAMP, CAL_SETTLE, CAL_FWD, CAL_REV };
static int step, t, dir;
static float v, th, ia_f, moved, last;
static float s1, c1, s2, c2;

void cal_start(void)
{
    step = CAL_RAMP;
    t = 0;
    v = th = ia_f = moved = 0.0f;
    s1 = c1 = s2 = c2 = 0.0f;
    svpwm_reset();
    foc.state = ST_CAL;
    hal_pwm_enable(1);
}

static void accum(float raw, float *s, float *c, int d)
{
    float e = wrap_2pi(POLE_PAIRS * (d > 0 ? raw : TWO_PI - raw) - th);
    float se, ce;
    sincos_lut(e, &se, &ce);
    *s += se;
    *c += ce;
}

void cal_run(float ia_raw, float raw)
{
    const float sweep = TWO_PI * CAL_EREVS;
    const float dth = sweep / CAL_SWEEP_S * TS;
    t++;

    switch (step) {
    case CAL_RAMP:
        ia_f += 0.01f * (ia_raw - ia_f);
        v += CAL_VMAX * TS;
        if (fabsf(ia_f) >= CAL_CURRENT) {
            cfg.cur_sign = ia_f > 0 ? 1 : -1;
            step = CAL_SETTLE;
            t = 0;
        } else if (v > CAL_VMAX) {
            foc_fault(F_CAL);
            return;
        }
        break;
    case CAL_SETTLE:
        if (t > (int)(F_PWM / 2)) {
            step = CAL_FWD;
            last = raw;
            t = 0;
        }
        break;
    case CAL_FWD:
        th += dth;
        moved += wrap_pi(raw - last);
        last = raw;
        accum(raw, &s1, &c1, 1);
        accum(raw, &s2, &c2, -1);
        if (th >= sweep) {
            float expect = sweep / POLE_PAIRS;
            if (fabsf(moved) < 0.75f * expect || fabsf(moved) > 1.25f * expect) {
                foc_fault(F_CAL);
                return;
            }
            dir = moved > 0 ? 1 : -1;
            if (dir < 0) { s1 = s2; c1 = c2; }
            step = CAL_REV;
        }
        break;
    case CAL_REV:
        th -= dth;
        accum(raw, &s1, &c1, dir);
        if (th <= 0.0f) {
            hal_pwm_enable(0);
            cfg.enc_dir = dir;
            cfg.enc_offset = wrap_2pi(atan2f(s1, c1));
            cfg.cal_valid = 1;
            foc_pll_reset(dir > 0 ? raw : wrap_2pi(TWO_PI - raw));
            foc.cal_done = 1;
            foc.state = ST_IDLE;
            return;
        }
        break;
    }
    svpwm_apply(v, 0.0f, th, foc.vbus);
}
