/*
 * config.h - motor, board and tuning parameters.
 *
 * Everything that changes between motors or boards lives here. The PI gains
 * are calculated from these values at boot, so normally only this file needs
 * editing when switching motors.
 */
#pragma once

/* ---- Motor ---- */
#define POLE_PAIRS      2
#define RS              0.0628f     /* phase resistance, ohm */
#define LD              0.147e-3f   /* d-axis inductance, H */
#define LQ              0.144e-3f   /* q-axis inductance, H */
#define FLUX_PM         0.00772f    /* magnet flux linkage, Wb */
#define KT              0.0232f     /* torque constant, Nm/A */
#define J_ROTOR         19.45e-6f   /* rotor inertia, kg m^2 (add load inertia here) */
#define B_VISC          0.0371e-3f  /* viscous friction, Nm s/rad */

/* ---- Limits and protection ---- */
#define VBUS_NOMINAL    24.0f
#define I_MAX           20.0f       /* peak phase current limit, A */
#define I_TRIP          30.0f       /* instantaneous overcurrent trip, A */
#define SPEED_MAX       1047.2f     /* mechanical rad/s (10k rpm) */
#define SPEED_RAMP      5000.0f     /* speed setpoint slew rate, rad/s^2 */
#define VBUS_MIN        8.0f
#define VBUS_MAX        60.0f
#define TEMP_MAX        100.0f      /* FET temperature trip, degC */
#define STALL_SPEED     5.0f        /* below this with full current demand ... */
#define STALL_MS        500         /* ... for this long = stall fault */
#define I_SUM_FAULT     5.0f        /* |ia + ib + ic| above this = sensor fault, A */

/* ---- Timing ---- */
#define F_SYS           170000000u
#define F_PWM           20000u
#define TS              (1.0f / F_PWM)
#define SPEED_DIV       10          /* speed loop runs every 10th current loop (2 kHz) */
#define TS_SPEED        (TS * SPEED_DIV)
#define PWM_PER         (F_SYS * 4u / F_PWM)   /* HRTIM ticks per period at x4 clock = 34000 */
#define DEADTIME_NS     200

/* ---- Loop tuning ---- */
#define CURRENT_BW_HZ   1000.0f
#define SPEED_BW_HZ     50.0f
#define PLL_BW_HZ       300.0f
#define DELAY_COMP      1.5f        /* PWM delay to compensate, in periods */

/* ---- Field weakening ---- */
#define FW_VFRAC        0.90f       /* keep |Vdq| at this fraction of the voltage limit */
#define FW_KP           0.05f       /* A/V */
#define FW_KI           150.0f      /* A/(V s) */
#define FW_ID_MIN       (-15.0f)    /* most negative d-axis current allowed, A */

/* ---- Modulation ---- */
#define MOD_MAX         0.95f       /* fraction of the linear SVPWM limit Vbus/sqrt(3) */
#define DUTY_MIN        0.02f
#define DUTY_MAX        0.98f

/* ---- Sensing ---- */
#define ADC_VREF        3.3f
#define R_SHUNT         0.002f
#define CSA_GAIN        20.0f
#define I_PER_COUNT     (ADC_VREF / 4096.0f / (CSA_GAIN * R_SHUNT))
#define V_PER_COUNT     (ADC_VREF / 4096.0f * (383.0f + 9.76f) / 9.76f)
#define NTC_R_TOP       4700.0f
#define NTC_R25         10000.0f
#define NTC_BETA        3950.0f

/* ---- Calibration ---- */
#define CAL_CURRENT     5.0f        /* alignment current, A */
#define CAL_VMAX        3.0f        /* voltage ceiling while ramping up, V */
#define CAL_EREVS       4           /* electrical revolutions per sweep */
#define CAL_SWEEP_S     2.0f        /* seconds per sweep */

/* ---- Communication ---- */
#define CAN_NODE_ID     1
#define CAN_TIMEOUT_MS  250         /* CAN setpoints must refresh this often */
#define CAN_TELEM_MS    10
#define TELEM_MS        5           /* serial telemetry period */
