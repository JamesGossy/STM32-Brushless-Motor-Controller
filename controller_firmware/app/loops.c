/*
 * loops.c - the control loops.
 *
 *   20 kHz  d/q current PI with cross-coupling feed-forward, then SVPWM
 *   2 kHz   field weakening PI (sets id_ref) and speed PI (sets iq_ref)
 *
 * Gains come from the motor parameters in config.h:
 *   current: Kp = L * wc, Ki = R * wc (the PI zero cancels the motor pole)
 *   speed:   Kp = J * ws / Kt, Ki = Kp * ws / 4
 */
#include "loops.h"
#include "foc.h"
#include "svpwm.h"
#include "mathx.h"
#include "config.h"
#include <math.h>

static pi_t pi_d, pi_q, pi_speed, pi_fw;
static int slow_count, stall_count;
static float speed_sum, vmag_sum;   /* averaged over one slow period */

/* Calculate the PI gains. */
void loops_init(void)
{
    float wc = TWO_PI * CURRENT_BW_HZ;
    pi_d.kp = LD * wc;
    pi_d.ki = RS * wc;
    pi_q.kp = LQ * wc;
    pi_q.ki = RS * wc;

    float ws = TWO_PI * SPEED_BW_HZ;
    pi_speed.kp = J_ROTOR * ws / KT;
    pi_speed.ki = pi_speed.kp * ws / 4.0f;

    pi_fw.kp = FW_KP;
    pi_fw.ki = FW_KI;
}

/* Clear all loop state before the drive starts running. */
void loops_reset(void)
{
    pi_d.i = pi_q.i = pi_fw.i = pi_speed.i = 0.0f;
    foc.id_ref = foc.iq_ref = 0.0f;
    foc.speed_ref = foc.omega_m;
    slow_count = stall_count = 0;
    speed_sum = vmag_sum = 0.0f;
}

/* Switch from torque to speed mode without a jump in current. */
void loops_speed_bumpless(float speed)
{
    foc.speed_ref = speed;
    pi_speed.i = foc.iq_ref;
}

/* 2 kHz: field weakening, then the speed or torque reference, then stall check. */
static void slow_loop(float vmax)
{
    float speed = speed_sum / SPEED_DIV;
    float vmag = vmag_sum / SPEED_DIV;
    speed_sum = vmag_sum = 0.0f;

    /* field weakening: pull id negative when the voltage gets close to the limit */
    float ilim = minf(foc.i_limit, I_MAX);
    float id_min = maxf(FW_ID_MIN, -ilim);
    foc.id_ref = pi_run(&pi_fw, FW_VFRAC * vmax - vmag, 0.0f, id_min, 0.0f, TS_SPEED);

    /* whatever current is left in the limit circle is available for torque */
    float iq_lim = sqrtf(maxf(ilim * ilim - foc.id_ref * foc.id_ref, 0.0f));

    if (foc.mode == MODE_SPEED) {
        float lim = minf(foc.speed_limit, SPEED_MAX);
        float target = clampf(foc.cmd_speed, -lim, lim);
        float step = SPEED_RAMP * TS_SPEED;
        foc.speed_ref += clampf(target - foc.speed_ref, -step, step);
        foc.iq_ref = pi_run(&pi_speed, foc.speed_ref - speed, 0.0f, -iq_lim, iq_lim, TS_SPEED);
    } else {
        foc.iq_ref = clampf(foc.cmd_iq, -iq_lim, iq_lim);
    }

    /* locked rotor or lost position: full current demand but no motion */
    int stalled = iq_lim > 1.0f && fabsf(foc.iq_ref) >= 0.95f * iq_lim && fabsf(speed) < STALL_SPEED;
    if (!stalled)
        stall_count = 0;
    else if (++stall_count > STALL_MS * 1000 / (int)(TS_SPEED * 1e6f))
        foc_fault(F_STALL);
}

/* 20 kHz: d/q current control. theta_e is the electrical angle at the
   current sample, we the electrical speed. */
void loops_run(float id, float iq, float theta_e, float we)
{
    float vmax = MOD_MAX * foc.vbus * INV_SQRT3;

    speed_sum += foc.omega_m;
    vmag_sum += sqrtf(foc.vd * foc.vd + foc.vq * foc.vq);
    if (++slow_count >= SPEED_DIV) {
        slow_count = 0;
        slow_loop(vmax);
    }

    /* feed-forward cancels the speed-dependent coupling between the axes */
    float ff_d = -we * LQ * iq;
    float ff_q = we * (LD * id + FLUX_PM);

    /* d axis gets priority, q gets what is left of the voltage circle */
    float vd = pi_run(&pi_d, foc.id_ref - id, ff_d, -vmax, vmax, TS);
    float vq_max = sqrtf(maxf(vmax * vmax - vd * vd, 0.0f));
    float vq = pi_run(&pi_q, foc.iq_ref - iq, ff_q, -vq_max, vq_max, TS);

    if (!isfinite(vd + vq)) {
        foc_fault(F_NAN);
        return;
    }
    foc.vd = vd;
    foc.vq = vq;

    /* the new duty takes effect next period and is centred half a period
       after that, so the rotor will have moved 1.5 periods further */
    svpwm_apply(vd, vq, theta_e + DELAY_COMP * we * TS, foc.vbus);
}
