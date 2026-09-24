/*
 * app.c - everything outside the control interrupt.
 *
 * app_poll() runs as fast as the main loop spins (command input, saving
 * settings). app_tick() runs every millisecond (protection, telemetry,
 * watchdog, LED). The functions below them are the shared command API.
 */
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "cmd.h"
#include "telem.h"
#include "hal.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

volatile uint8_t app_save_req;
volatile uint8_t app_setpoint_src;
volatile uint32_t app_setpoint_ms;

static uint16_t drv_status1, drv_status2;
static uint32_t tick_count, led_ms, last_isr_count;

/* Load settings and bring up the gate driver. Called before the ISR starts. */
void app_init(void)
{
    cfg_load();
    foc_init();
    if (!hal_gate_init()) foc_fault(F_DRV_INIT);
}

/* Main loop work that isn't time critical. */
void app_poll(void)
{
    can_proto_poll();
    cmd_poll();

    /* flash writes only while the motor is stopped */
    int stopped = (foc.state == ST_IDLE || foc.state == ST_FAULT) && foc.req == REQ_NONE;
    if (foc.cal_done && stopped) {
        foc.cal_done = 0;
        if (cfg_save())
            telem_log("calibration done: encoder dir %d, current sign %d, offset %.3f rad (saved)",
                      (int)cfg.enc_dir, (int)cfg.cur_sign, (double)cfg.enc_offset);
        else
            foc_fault(F_CAL);
    }
    if (app_save_req && stopped) {
        app_save_req = 0;
        cfg_save();
    }
}

/* 1 kHz housekeeping. */
void app_tick(void)
{
    tick_count++;
    uint32_t now = hal_millis();

    /* only feed the watchdog while the control interrupt is alive */
    if (foc_isr_count() != last_isr_count) {
        last_isr_count = foc_isr_count();
        hal_wdg_kick();
    }

    hal_temp_poll((float *)&foc.t_fet, (float *)&foc.t_amb);
    if (foc.t_fet > TEMP_MAX) foc_fault(F_OVERTEMP);

    if (tick_count % 10 == 0 && foc.state != ST_BOOT && hal_gate_status(&drv_status1, &drv_status2))
        foc_fault(F_DRV);

    /* a CAN master must keep sending setpoints; serial console setpoints latch */
    if (foc.state == ST_RUN && app_setpoint_src == SRC_CAN && now - app_setpoint_ms > CAN_TIMEOUT_MS)
        foc.req = REQ_IDLE;

    telem_tick(tick_count);
    can_proto_telemetry(tick_count, drv_status1, drv_status2);

    if (now - led_ms >= (foc.state == ST_FAULT ? 100u : 500u)) {
        led_ms = now;
        hal_led_toggle();
    }
}

/* Clamp a setpoint to +/- limit, treating NaN/inf as 0. */
static float clamp_setpoint(float v, float limit)
{
    if (!isfinite(v)) return 0.0f;
    return v > limit ? limit : (v < -limit ? -limit : v);
}

/* Remember who sent the last setpoint and when (for the CAN timeout). */
static void touch(uint8_t src)
{
    app_setpoint_src = src;
    app_setpoint_ms = hal_millis();
}

/* Ask for a new drive state. Returns NULL if accepted, otherwise a short
   reason for the user. */
const char *app_request(uint8_t run, uint8_t src)
{
    touch(src);
    if (run == RUN_IDLE) {
        foc.cmd_iq = foc.cmd_speed = 0.0f;
        foc.req = REQ_IDLE;
        return NULL;
    }

    if (foc.state == ST_BOOT) return "still starting up";
    if (foc.faults) return "clear the faults first ('clear')";
    if (foc.vbus < VBUS_MIN) return "bus voltage too low";

    if (run == RUN_CALIBRATE) {
        if (foc.state == ST_RUN) return "stop the motor first ('motor off')";
        if (foc.state == ST_CAL) return "already calibrating";
        foc.req = REQ_CAL;
        return NULL;
    }

    if (!cfg.cal_valid) return "not calibrated: run 'calibrate' with the motor free to spin";
    if (foc.state == ST_CAL) return "wait for calibration to finish";

    uint8_t mode = run == RUN_SPEED ? MODE_SPEED : MODE_TORQUE;
    if (foc.state == ST_RUN && foc.mode == mode) return NULL;    /* already there */

    foc.cmd_iq = 0.0f;
    foc.cmd_speed = 0.0f;
    foc.req = mode == MODE_SPEED ? REQ_SPEED : REQ_TORQUE;
    return NULL;
}

/* Torque-mode current setpoint, amps. */
void app_set_iq(float amps, uint8_t src)
{
    foc.cmd_iq = clamp_setpoint(amps, I_MAX);
    touch(src);
}

/* Speed-mode setpoint, mechanical rad/s. */
void app_set_speed(float rad_s, uint8_t src)
{
    foc.cmd_speed = clamp_setpoint(rad_s, SPEED_MAX);
    touch(src);
}

/* Runtime current and speed limits (0 .. config.h limits). */
void app_set_limits(float amps, float rad_s)
{
    amps = clamp_setpoint(amps, I_MAX);
    rad_s = clamp_setpoint(rad_s, SPEED_MAX);
    foc.i_limit = amps > 0.0f ? amps : 0.0f;
    foc.speed_limit = rad_s > 0.0f ? rad_s : 0.0f;
}

/* Clear faults, re-initialising the gate driver if that was the problem. */
void app_clear_faults(void)
{
    if (foc.state != ST_FAULT && foc.state != ST_IDLE) return;

    int reinit = foc.faults & F_DRV_INIT;
    int ok = reinit ? hal_gate_init() : 1;
    if (!reinit) hal_gate_clear();

    foc_clear_faults();
    if (!ok) foc_fault(F_DRV_INIT);
}

/* Human readable list of the set fault bits. */
const char *app_fault_text(uint16_t faults)
{
    static const char *names[] = {
        "overcurrent", "overvoltage", "undervoltage", "overtemp", "gate driver",
        "current sum", "calibration", "gate driver init", "overspeed", "not calibrated",
        "adc offset", "encoder", "nan", "stall",
    };
    static char text[160];
    int n = 0;
    text[0] = 0;
    for (unsigned i = 0; i < sizeof names / sizeof names[0]; i++)
        if (faults & (1u << i))
            n += snprintf(text + n, sizeof text - n, "%s%s", n ? ", " : "", names[i]);
    return n ? text : "none";
}
