#include "foc.h"
#include "hal.h"
#include "svpwm.h"
#include "loops.h"
#include "calib.h"
#include "cfg.h"
#include "trig.h"
#include "config.h"
#include <math.h>

foc_t foc;

static float pll_kp, pll_ki, pll_th, pll_w;
static float off_a, off_b, off_c;
static uint32_t n_isr;
static int sum_bad, enc_bad, pll_init;

void foc_init(void)
{
    trig_init();
    loops_init();
    float wn = TWO_PI * PLL_BW_HZ;
    pll_kp = 2.0f * wn;
    pll_ki = wn * wn;
    foc.i_limit = I_MAX;
    foc.speed_limit = SPEED_MAX;
    foc.state = ST_BOOT;
}

uint32_t foc_isr_count(void) { return n_isr; }

void foc_pll_reset(float th)
{
    pll_th = th;
    pll_w = 0.0f;
}

void foc_fault(uint16_t f)
{
    uint32_t pm = hal_irq_save();
    hal_pwm_enable(0);
    foc.faults |= f;
    if (foc.state != ST_BOOT) foc.state = ST_FAULT;
    hal_irq_restore(pm);
}

void foc_clear_faults(void)
{
    uint32_t pm = hal_irq_save();
    foc.faults = 0;
    sum_bad = enc_bad = 0;
    if (foc.state == ST_FAULT) foc.state = ST_IDLE;
    hal_irq_restore(pm);
}

static void start_run(uint8_t mode)
{
    loops_reset();
    foc.mode = mode;
    svpwm_reset();
    foc.state = ST_RUN;
    hal_pwm_enable(1);
}

/* average the current sense offsets with PWM off */
static void boot_offsets(const hal_sample_t *s)
{
    foc.req = REQ_NONE;
    if (n_isr <= 1000) return;
    off_a += s->ia; off_b += s->ib; off_c += s->ic;
    if (n_isr < 1000 + 4096) return;
    off_a /= 4096.0f; off_b /= 4096.0f; off_c /= 4096.0f;
    if (fabsf(off_a - 2048.0f) > 150.0f || fabsf(off_b - 2048.0f) > 150.0f ||
        fabsf(off_c - 2048.0f) > 150.0f) foc.faults |= F_OFFSET;
    foc.state = foc.faults ? ST_FAULT : ST_IDLE;
}

/* state requests from the main loop */
static void handle_request(void)
{
    uint8_t req = foc.req;
    if (req == REQ_NONE) return;
    foc.req = REQ_NONE;
    int can_start = foc.state == ST_IDLE && !foc.faults && foc.vbus >= VBUS_MIN;

    if (req == REQ_IDLE) {
        if (foc.state == ST_RUN || foc.state == ST_CAL) { hal_pwm_enable(0); foc.state = ST_IDLE; }
    } else if (req == REQ_CAL) {
        if (can_start) cal_start();
    } else {
        uint8_t m = req == REQ_SPEED ? MODE_SPEED : MODE_TORQUE;
        if (!cfg.cal_valid) foc_fault(F_NOT_CAL);
        else if (can_start) start_run(m);
        else if (foc.state == ST_IDLE && foc.vbus < VBUS_MIN) foc_fault(F_UNDERVOLT);
        else if (foc.state == ST_RUN && foc.mode != m) {
            if (m == MODE_SPEED) loops_speed_bumpless(pll_w);
            foc.mode = m;
        }
    }
}

/* 20 kHz current loop */
void app_control_isr(const hal_sample_t *smp)
{
    foc.vbus = smp->vbus * V_PER_COUNT;
    n_isr++;
    if (foc.state == ST_BOOT) { boot_offsets(smp); return; }

    float ia_raw = (smp->ia - off_a) * I_PER_COUNT;
    float s = (float)cfg.cur_sign;
    float ia = ia_raw * s;
    float ib = (smp->ib - off_b) * I_PER_COUNT * s;
    float ic = (smp->ic - off_c) * I_PER_COUNT * s;

    /* drop the phase with the shortest low-side window */
    float dmax = maxf(duty[0], maxf(duty[1], duty[2]));
    if (dmax < 0.8f && (foc.state == ST_RUN || foc.state == ST_CAL)) {
        if (fabsf(ia + ib + ic) > I_SUM_FAULT) { if (++sum_bad > 100) foc_fault(F_CURRENT_SUM); }
        else if (sum_bad) sum_bad--;
    }
    if (duty[0] >= duty[1] && duty[0] >= duty[2]) ia = -ib - ic;
    else if (duty[1] >= duty[2]) ib = -ia - ic;
    else ic = -ia - ib;
    foc.ia = ia; foc.ib = ib; foc.ic = ic;

    if (fabsf(ia) > I_TRIP || fabsf(ib) > I_TRIP || fabsf(ic) > I_TRIP) foc_fault(F_OVERCURRENT);
    if (foc.vbus > VBUS_MAX) foc_fault(F_OVERVOLT);

    /* position, PLL speed */
    float raw = smp->enc * (TWO_PI / 65536.0f);
    float thm = cfg.enc_dir > 0 ? raw : wrap_2pi(TWO_PI - raw);
    if (!pll_init) { pll_th = thm; pll_init = 1; }
    float e = wrap_pi(thm - pll_th);
    if (foc.state == ST_RUN && fabsf(e) > 0.5f) { if (++enc_bad > 3) foc_fault(F_ENCODER); }
    else enc_bad = 0;
    pll_w += pll_ki * TS * e;
    pll_th = wrap_2pi(pll_th + TS * (pll_w + pll_kp * e));
    foc.theta_m = thm;
    foc.omega_m = pll_w;
    float th = wrap_2pi(POLE_PAIRS * thm - cfg.enc_offset);
    foc.theta_e = th;

    /* Clarke + Park */
    float sn, cs;
    sincos_lut(th, &sn, &cs);
    float ibe = (ib - ic) * INV_SQRT3;
    float id = ia * cs + ibe * sn;
    float iq = -ia * sn + ibe * cs;
    foc.id = id; foc.iq = iq;

    handle_request();

    if (foc.state == ST_CAL) {
        if (foc.vbus < VBUS_MIN) foc_fault(F_UNDERVOLT);
        else cal_run(ia_raw, raw);
        return;
    }
    if (foc.state != ST_RUN) return;
    if (foc.vbus < VBUS_MIN) { foc_fault(F_UNDERVOLT); return; }
    if (fabsf(pll_w) > 1.25f * SPEED_MAX) { foc_fault(F_OVERSPEED); return; }

    loops_run(id, iq, th, POLE_PAIRS * pll_w);
}
