#pragma once

/* PMSM in the rotor dq frame plus an averaged inverter with dead time */
typedef struct {
    double rs, ld, lq, psi, j, b, coulomb;
    int p;
    double th_m, w_m, id, iq, t_load;
    double vbus, t_dead, t_pwm;
    int locked;
} motor_t;

void motor_init(motor_t *m);
void motor_step(motor_t *m, const double duty[3], int driven, double dt);
void motor_currents(const motor_t *m, double iabc[3]);
double motor_torque(const motor_t *m);
