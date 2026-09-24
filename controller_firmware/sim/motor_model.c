#include "motor_model.h"
#include "config.h"
#include <math.h>

#define TWO_PI_3 2.0943951023931953

void motor_init(motor_t *m)
{
    m->rs = RS; m->ld = LD; m->lq = LQ; m->psi = FLUX_PM;
    m->j = J_ROTOR; m->b = B_VISC; m->coulomb = 0.002;
    m->p = POLE_PAIRS;
    m->th_m = 1.234; m->w_m = 0.0; m->id = m->iq = 0.0; m->t_load = 0.0;
    m->vbus = VBUS_NOMINAL;
    m->t_dead = DEADTIME_NS * 1e-9;
    m->t_pwm = 1.0 / F_PWM;
    m->locked = 0;
}

void motor_currents(const motor_t *m, double i[3])
{
    double th = m->p * m->th_m;
    for (int k = 0; k < 3; k++) {
        double a = th - k * TWO_PI_3;
        i[k] = m->id * cos(a) - m->iq * sin(a);
    }
}

double motor_torque(const motor_t *m)
{
    return 1.5 * m->p * (m->psi * m->iq + (m->ld - m->lq) * m->id * m->iq);
}

void motor_step(motor_t *m, const double duty[3], int driven, double dt)
{
    double th = m->p * m->th_m, we = m->p * m->w_m;

    if (driven) {
        double iabc[3], v[3];
        motor_currents(m, iabc);
        double dv = m->vbus * m->t_dead / m->t_pwm;
        for (int k = 0; k < 3; k++)
            v[k] = duty[k] * m->vbus - (iabc[k] > 0 ? dv : (iabc[k] < 0 ? -dv : 0.0));
        double va = (2 * v[0] - v[1] - v[2]) / 3.0, vb = (v[1] - v[2]) / sqrt(3.0);
        double vd = va * cos(th) + vb * sin(th), vq = -va * sin(th) + vb * cos(th);
        m->id += (vd - m->rs * m->id + we * m->lq * m->iq) / m->ld * dt;
        m->iq += (vq - m->rs * m->iq - we * m->ld * m->id - we * m->psi) / m->lq * dt;
    } else {
        double k = exp(-dt * 5000.0);   /* freewheel diodes: current collapses */
        m->id *= k;
        m->iq *= k;
    }

    if (m->locked) { m->w_m = 0.0; return; }
    double tl = m->t_load + m->b * m->w_m;
    if (fabs(m->w_m) > 1e-3) tl += m->w_m > 0 ? m->coulomb : -m->coulomb;
    m->w_m += (motor_torque(m) - tl) / m->j * dt;
    m->th_m += m->w_m * dt;
}
