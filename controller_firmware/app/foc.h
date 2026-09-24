/*
 * foc.h - motor control state shared between the 20 kHz loop and the rest
 * of the application.
 *
 * The main loop never touches the PWM directly. It writes setpoints and a
 * state request into `foc`, and the control interrupt acts on them.
 */
#pragma once
#include <stdint.h>

/* drive states */
enum { ST_BOOT, ST_IDLE, ST_CAL, ST_RUN, ST_FAULT };

/* control modes while running */
enum { MODE_TORQUE = 1, MODE_SPEED = 2 };

/* state requests from the main loop */
enum { REQ_NONE, REQ_IDLE, REQ_TORQUE, REQ_SPEED, REQ_CAL };

/* fault bits (latched until cleared) */
#define F_OVERCURRENT  (1u << 0)
#define F_OVERVOLT     (1u << 1)
#define F_UNDERVOLT    (1u << 2)
#define F_OVERTEMP     (1u << 3)
#define F_DRV          (1u << 4)    /* gate driver reported a fault */
#define F_CURRENT_SUM  (1u << 5)    /* phase currents don't sum to zero */
#define F_CAL          (1u << 6)    /* calibration failed */
#define F_DRV_INIT     (1u << 7)    /* gate driver didn't accept its config */
#define F_OVERSPEED    (1u << 8)
#define F_NOT_CAL      (1u << 9)    /* run requested before calibration */
#define F_OFFSET       (1u << 10)   /* current sense offset out of range at boot */
#define F_ENCODER      (1u << 11)   /* encoder angle jumped */
#define F_NAN          (1u << 12)   /* control maths produced NaN */
#define F_STALL        (1u << 13)   /* full current but rotor not moving */

typedef struct {
    /* state */
    volatile uint8_t state, mode, req, cal_done;
    volatile uint16_t faults;

    /* setpoints and limits (written by the main loop) */
    volatile float cmd_iq, cmd_speed, i_limit, speed_limit;

    /* measurements and loop signals (written by the control loop) */
    volatile float ia, ib, ic, id, iq, vd, vq, id_ref, iq_ref, speed_ref;
    volatile float vbus, theta_e, theta_m, omega_m;
    volatile float t_fet, t_amb;
} foc_t;

extern foc_t foc;

void foc_init(void);
void foc_fault(uint16_t f);
void foc_clear_faults(void);
void foc_pll_reset(float theta_m);
uint32_t foc_isr_count(void);
