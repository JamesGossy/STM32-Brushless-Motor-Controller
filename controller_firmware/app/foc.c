/*
 * foc.c - the 20 kHz control interrupt.
 *
 * Each PWM period the HAL hands over a fresh current/voltage/encoder sample.
 * This file turns it into phase currents, rotor angle and speed, handles
 * state changes and protection, and then runs either calibration or the
 * current/speed loops (loops.c).
 */
#include "foc.h"
#include "hal.h"
#include "svpwm.h"
#include "loops.h"
#include "calib.h"
#include "cfg.h"
#include "trig.h"
#include "config.h"
#include <math.h>

#define BOOT_SKIP     1000   /* samples to wait before measuring offsets */
#define BOOT_SAMPLES  4096   /* samples averaged for the offsets */

foc_t foc;

static float pll_kp, pll_ki, pll_theta, pll_speed;
static int pll_started;
static float offset_a, offset_b, offset_c;
static uint32_t isr_count;
static int sum_errors, encoder_errors;

/* Set up the PLL gains and control loops. Called once before the ISR starts. */
void foc_init(void)
{
    trig_init();
    loops_init();

    float wn = TWO_PI * PLL_BW_HZ;
    pll_kp = 2.0f * wn;     /* critically damped */
    pll_ki = wn * wn;

    foc.i_limit = I_MAX;
    foc.speed_limit = SPEED_MAX;
    foc.state = ST_BOOT;
}

/* Number of control interrupts so far (used by the watchdog check). */
uint32_t foc_isr_count(void) { return isr_count; }

/* Restart the PLL at a known angle, e.g. after calibration changes direction. */
void foc_pll_reset(float theta_m)
{
    pll_theta = theta_m;
    pll_speed = 0.0f;
}

/* Latch a fault and turn the power stage off. Safe from any context. */
void foc_fault(uint16_t f)
{
    uint32_t irq = hal_irq_save();
    hal_pwm_enable(0);
    foc.faults |= f;
    if (foc.state != ST_BOOT) foc.state = ST_FAULT;
    hal_irq_restore(irq);
}

/* Clear all faults and go back to idle. */
void foc_clear_faults(void)
{
    uint32_t irq = hal_irq_save();
    foc.faults = 0;
    sum_errors = encoder_errors = 0;
    if (foc.state == ST_FAULT) foc.state = ST_IDLE;
    hal_irq_restore(irq);
}

/* Average the current sense offsets while the PWM is off. */
static void measure_offsets(const hal_sample_t *s)
{
    foc.req = REQ_NONE;
    if (isr_count <= BOOT_SKIP) return;

    offset_a += s->ia;
    offset_b += s->ib;
    offset_c += s->ic;
    if (isr_count < BOOT_SKIP + BOOT_SAMPLES) return;

    offset_a /= BOOT_SAMPLES;
    offset_b /= BOOT_SAMPLES;
    offset_c /= BOOT_SAMPLES;

    /* the amplifiers should sit near mid-scale (2048) */
    if (fabsf(offset_a - 2048.0f) > 150.0f || fabsf(offset_b - 2048.0f) > 150.0f ||
        fabsf(offset_c - 2048.0f) > 150.0f)
        foc.faults |= F_OFFSET;

    foc.state = foc.faults ? ST_FAULT : ST_IDLE;
}

/* Convert raw ADC counts to amps and rebuild the least trustworthy phase. */
static void read_currents(const hal_sample_t *s, float *ia_raw)
{
    float sign = (float)cfg.cur_sign;
    *ia_raw = (s->ia - offset_a) * I_PER_COUNT;
    float ia = *ia_raw * sign;
    float ib = (s->ib - offset_b) * I_PER_COUNT * sign;
    float ic = (s->ic - offset_c) * I_PER_COUNT * sign;

    /* all three are valid when no duty is near 100 %: check they sum to zero */
    int active = foc.state == ST_RUN || foc.state == ST_CAL;
    if (active && maxf(duty[0], maxf(duty[1], duty[2])) < 0.8f) {
        if (fabsf(ia + ib + ic) > I_SUM_FAULT) {
            if (++sum_errors > 100) foc_fault(F_CURRENT_SUM);
        } else if (sum_errors) {
            sum_errors--;
        }
    }

    /* the phase with the highest duty has the shortest low-side window,
       so replace it with minus the sum of the other two */
    if (duty[0] >= duty[1] && duty[0] >= duty[2]) ia = -ib - ic;
    else if (duty[1] >= duty[2])                  ib = -ia - ic;
    else                                          ic = -ia - ib;

    foc.ia = ia;
    foc.ib = ib;
    foc.ic = ic;
}

/* Mechanical angle from the encoder, and speed from a PLL tracking it. */
static float read_angle(uint16_t enc)
{
    float raw = enc * (TWO_PI / 65536.0f);
    float theta_m = cfg.enc_dir > 0 ? raw : wrap_2pi(TWO_PI - raw);

    if (!pll_started) {
        pll_theta = theta_m;
        pll_started = 1;
    }

    float err = wrap_pi(theta_m - pll_theta);

    /* a real rotor can't jump: a large error for several samples is a bad encoder */
    if (foc.state == ST_RUN && fabsf(err) > 0.5f) {
        if (++encoder_errors > 3) foc_fault(F_ENCODER);
    } else {
        encoder_errors = 0;
    }

    pll_speed += pll_ki * TS * err;
    pll_theta = wrap_2pi(pll_theta + TS * (pll_speed + pll_kp * err));

    foc.theta_m = theta_m;
    foc.omega_m = pll_speed;
    return raw;
}

/* Put the drive into run mode with fresh loop state. */
static void start_run(uint8_t mode)
{
    loops_reset();
    foc.mode = mode;
    svpwm_reset();
    foc.state = ST_RUN;
    hal_pwm_enable(1);
}

/* Act on a state change requested by the main loop. */
static void handle_request(void)
{
    uint8_t req = foc.req;
    if (req == REQ_NONE) return;
    foc.req = REQ_NONE;

    int can_start = foc.state == ST_IDLE && !foc.faults && foc.vbus >= VBUS_MIN;

    if (req == REQ_IDLE) {
        if (foc.state == ST_RUN || foc.state == ST_CAL) {
            hal_pwm_enable(0);
            foc.state = ST_IDLE;
        }
    } else if (req == REQ_CAL) {
        if (can_start) cal_start();
    } else {
        uint8_t mode = req == REQ_SPEED ? MODE_SPEED : MODE_TORQUE;
        if (!cfg.cal_valid) {
            foc_fault(F_NOT_CAL);
        } else if (can_start) {
            start_run(mode);
        } else if (foc.state == ST_RUN && foc.mode != mode) {
            if (mode == MODE_SPEED) loops_speed_bumpless(pll_speed);
            foc.mode = mode;
        }
    }
}

/* Called by the HAL every PWM period with the latest sample. */
void app_control_isr(const hal_sample_t *s)
{
    isr_count++;
    foc.vbus = s->vbus * V_PER_COUNT;

    if (foc.state == ST_BOOT) {
        measure_offsets(s);
        return;
    }

    float ia_raw;
    read_currents(s, &ia_raw);
    float raw_angle = read_angle(s->enc);

    if (fabsf(foc.ia) > I_TRIP || fabsf(foc.ib) > I_TRIP || fabsf(foc.ic) > I_TRIP)
        foc_fault(F_OVERCURRENT);
    if (foc.vbus > VBUS_MAX)
        foc_fault(F_OVERVOLT);

    /* Clarke + Park */
    float theta_e = wrap_2pi(POLE_PAIRS * foc.theta_m - cfg.enc_offset);
    float sn, cs;
    sincos_lut(theta_e, &sn, &cs);
    float ibeta = (foc.ib - foc.ic) * INV_SQRT3;
    float id = foc.ia * cs + ibeta * sn;
    float iq = -foc.ia * sn + ibeta * cs;
    foc.theta_e = theta_e;
    foc.id = id;
    foc.iq = iq;

    handle_request();

    if (foc.state == ST_CAL) {
        if (foc.vbus < VBUS_MIN) foc_fault(F_UNDERVOLT);
        else cal_run(ia_raw, raw_angle);
        return;
    }
    if (foc.state != ST_RUN) return;

    if (foc.vbus < VBUS_MIN) { foc_fault(F_UNDERVOLT); return; }
    if (fabsf(pll_speed) > 1.25f * SPEED_MAX) { foc_fault(F_OVERSPEED); return; }

    loops_run(id, iq, theta_e, POLE_PAIRS * pll_speed);
}
