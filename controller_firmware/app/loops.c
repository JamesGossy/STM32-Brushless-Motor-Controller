#include "loops.h"
#include "foc.h"
#include "svpwm.h"
#include "mathx.h"
#include "config.h"
#include <math.h>

static pi_t pi_d, pi_q, pi_spd, pi_fw;
static int div_cnt, stall_cnt;
static float w_acc, vs_acc;

void loops_init(void)
{
    float wc = TWO_PI * CURRENT_BW_HZ;
    pi_d.kp = LD * wc;  pi_d.ki = RS * wc;
    pi_q.kp = LQ * wc;  pi_q.ki = RS * wc;

    float ws = TWO_PI * SPEED_BW_HZ;
    pi_spd.kp = J_ROTOR * ws / KT;
    pi_spd.ki = pi_spd.kp * ws / 4.0f;

    pi_fw.kp = FW_KP;
    pi_fw.ki = FW_KI;
}

void loops_reset(void)
{
    pi_d.i = pi_q.i = pi_fw.i = pi_spd.i = 0.0f;
    foc.id_ref = foc.iq_ref = 0.0f;
    foc.speed_ref = foc.omega_m;
    div_cnt = stall_cnt = 0;
    w_acc = vs_acc = 0.0f;
}

void loops_speed_bumpless(float w)
{
    foc.speed_ref = w;
    pi_spd.i = foc.iq_ref;
}

/* 2 kHz: field weakening then speed or torque reference */
static void slow_loop(float vmax)
{
    float w = w_acc / SPEED_DIV;
    float vs = vs_acc / SPEED_DIV;
    w_acc = vs_acc = 0.0f;

    float ilim = minf(foc.i_limit, I_MAX);
    float id_min = maxf(FW_ID_MIN, -ilim);
    foc.id_ref = pi_run(&pi_fw, FW_VFRAC * vmax - vs, 0.0f, id_min, 0.0f, TS_SPEED);
    float iq_lim = sqrtf(maxf(ilim * ilim - foc.id_ref * foc.id_ref, 0.0f));

    if (foc.mode == MODE_SPEED) {
        float lim = minf(foc.speed_limit, SPEED_MAX);
        float tgt = clampf(foc.cmd_speed, -lim, lim);
        float step = SPEED_RAMP * TS_SPEED;
        foc.speed_ref += clampf(tgt - foc.speed_ref, -step, step);
        foc.iq_ref = pi_run(&pi_spd, foc.speed_ref - w, 0.0f, -iq_lim, iq_lim, TS_SPEED);
    } else {
        foc.iq_ref = clampf(foc.cmd_iq, -iq_lim, iq_lim);
    }

    /* locked rotor or lost position: full current demand but no motion */
    int stalled = iq_lim > 1.0f && fabsf(foc.iq_ref) >= 0.95f * iq_lim && fabsf(w) < STALL_SPEED;
    if (!stalled) stall_cnt = 0;
    else if (++stall_cnt > STALL_MS * 1000 / (int)(TS_SPEED * 1e6f)) foc_fault(F_STALL);
}

/* 20 kHz: d/q current PI with decoupling feedforward */
void loops_run(float id, float iq, float th, float we)
{
    float vmax = MOD_MAX * foc.vbus * INV_SQRT3;
    w_acc += foc.omega_m;
    vs_acc += sqrtf(foc.vd * foc.vd + foc.vq * foc.vq);
    if (++div_cnt >= SPEED_DIV) { div_cnt = 0; slow_loop(vmax); }

    float ffd = -we * LQ * iq;
    float ffq = we * (LD * id + FLUX_PM);
    float vd = pi_run(&pi_d, foc.id_ref - id, ffd, -vmax, vmax, TS);
    float vq_max = sqrtf(maxf(vmax * vmax - vd * vd, 0.0f));
    float vq = pi_run(&pi_q, foc.iq_ref - iq, ffq, -vq_max, vq_max, TS);
    if (!isfinite(vd + vq)) { foc_fault(F_NAN); return; }
    foc.vd = vd;
    foc.vq = vq;

    /* voltage lands on average 1.5 Ts after the current sample */
    svpwm_apply(vd, vq, th + DELAY_COMP * we * TS, foc.vbus);
}
