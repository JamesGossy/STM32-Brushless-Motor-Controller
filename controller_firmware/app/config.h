#pragma once

/* Motor */
#define POLE_PAIRS      2
#define RS              0.0628f     /* ohm */
#define LD              0.147e-3f   /* H */
#define LQ              0.144e-3f   /* H */
#define FLUX_PM         0.00772f    /* Wb */
#define KT              0.0232f     /* Nm/A */
#define J_ROTOR         19.45e-6f   /* kg m^2 (add load inertia here) */
#define B_VISC          0.0371e-3f  /* Nm s/rad */

/* Limits */
#define VBUS_NOMINAL    24.0f
#define I_MAX           20.0f       /* peak phase current limit, A */
#define I_TRIP          30.0f       /* instantaneous overcurrent trip, A */
#define SPEED_MAX       1047.2f     /* mech rad/s (10k rpm) */
#define SPEED_RAMP      5000.0f     /* mech rad/s^2 */
#define VBUS_MIN        8.0f
#define VBUS_MAX        60.0f
#define TEMP_MAX        100.0f      /* degC, FET NTC */
#define STALL_SPEED     5.0f        /* mech rad/s */
#define STALL_MS        500         /* full current demand below STALL_SPEED this long -> fault */

/* Timing */
#define F_SYS           170000000u
#define F_PWM           20000u
#define TS              (1.0f / F_PWM)
#define SPEED_DIV       10          /* speed / FW loop = F_PWM / SPEED_DIV = 2 kHz */
#define TS_SPEED        (TS * SPEED_DIV)
#define PWM_PER         (F_SYS * 4u / F_PWM)   /* HRTIM x4 prescaler -> 34000 */
#define DEADTIME_NS     200

/* Loop bandwidths */
#define CURRENT_BW_HZ   1000.0f
#define SPEED_BW_HZ     50.0f
#define PLL_BW_HZ       300.0f
#define DELAY_COMP      1.5f        /* x Ts, angle advance for inverse Park */

/* Field weakening */
#define FW_VFRAC        0.90f       /* regulate |Vdq| to this fraction of Vmax */
#define FW_KP           0.05f       /* A/V */
#define FW_KI           150.0f      /* A/(V s) */
#define FW_ID_MIN       (-15.0f)    /* most negative d-axis current */

/* Modulation */
#define MOD_MAX         0.95f       /* fraction of linear SVPWM limit Vbus/sqrt3 */
#define DUTY_MIN        0.02f
#define DUTY_MAX        0.98f

/* Sensing */
#define ADC_VREF        3.3f
#define R_SHUNT         0.002f
#define CSA_GAIN        20.0f
#define I_PER_COUNT     (ADC_VREF / 4096.0f / (CSA_GAIN * R_SHUNT))
#define V_PER_COUNT     (ADC_VREF / 4096.0f * (383.0f + 9.76f) / 9.76f)
#define NTC_R_TOP       4700.0f
#define NTC_R25         10000.0f
#define NTC_BETA        3950.0f
#define I_SUM_FAULT     5.0f        /* ia+ib+ic threshold, A */

/* Calibration */
#define CAL_CURRENT     5.0f        /* A */
#define CAL_VMAX        3.0f        /* V, voltage ceiling while ramping */
#define CAL_EREVS       4           /* electrical revs per sweep */
#define CAL_SWEEP_S     2.0f

/* CAN */
#define CAN_NODE_ID     1
#define CAN_TIMEOUT_MS  250         /* drop to idle if no setpoint received */
#define CAN_TELEM_MS    10

/* Serial telemetry (USB CDC on target, TCP in the simulator) */
#define TELEM_MS        5
