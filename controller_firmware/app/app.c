#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "cmd.h"
#include "telem.h"
#include "hal.h"
#include "config.h"
#include <math.h>

volatile uint8_t app_save_req;
volatile uint8_t app_setpoint_src;
volatile uint32_t app_setpoint_ms;

static uint16_t drv_s1, drv_s2;
static uint32_t n, led_ms, isr_seen;

static float clamp_cmd(float v, float lim)
{
    if (!isfinite(v)) return 0.0f;
    return v > lim ? lim : (v < -lim ? -lim : v);
}

void app_set_iq(float a, uint8_t src)
{
    foc.cmd_iq = clamp_cmd(a, I_MAX);
    app_setpoint_src = src;
    app_setpoint_ms = hal_millis();
}

void app_set_speed(float w, uint8_t src)
{
    foc.cmd_speed = clamp_cmd(w, SPEED_MAX);
    app_setpoint_src = src;
    app_setpoint_ms = hal_millis();
}

void app_set_state(uint8_t st, uint8_t src)
{
    foc.cmd_iq = 0.0f;
    foc.cmd_speed = 0.0f;
    foc.req = st == 1 ? REQ_TORQUE : st == 2 ? REQ_SPEED : st == 3 ? REQ_CAL : REQ_IDLE;
    app_setpoint_src = src;
    app_setpoint_ms = hal_millis();
}

void app_set_limits(float a, float w)
{
    a = clamp_cmd(a, I_MAX);
    w = clamp_cmd(w, SPEED_MAX);
    foc.i_limit = a > 0.0f ? a : 0.0f;
    foc.speed_limit = w > 0.0f ? w : 0.0f;
}

void app_clear_faults(void)
{
    if (foc.state != ST_FAULT && foc.state != ST_IDLE) return;
    int reinit = foc.faults & F_DRV_INIT;
    int ok = reinit ? hal_gate_init() : 1;
    if (!reinit) hal_gate_clear();
    foc_clear_faults();
    if (!ok) foc_fault(F_DRV_INIT);
}

void app_init(void)
{
    cfg_load();
    foc_init();
    if (!hal_gate_init()) foc_fault(F_DRV_INIT);
}

/* runs as fast as the main loop spins */
void app_poll(void)
{
    can_proto_poll();
    cmd_poll();

    int idle = (foc.state == ST_IDLE || foc.state == ST_FAULT) && foc.req == REQ_NONE;
    if (foc.cal_done && idle) {
        foc.cal_done = 0;
        if (!cfg_save()) foc_fault(F_CAL);
    }
    if (app_save_req && idle) {
        app_save_req = 0;
        cfg_save();
    }
}

/* 1 kHz */
void app_tick(void)
{
    n++;
    uint32_t now = hal_millis();

    if (foc_isr_count() != isr_seen) { isr_seen = foc_isr_count(); hal_wdg_kick(); }

    hal_temp_poll((float *)&foc.t_fet, (float *)&foc.t_amb);
    if (foc.t_fet > TEMP_MAX) foc_fault(F_OVERTEMP);

    if (n % 10 == 0 && foc.state != ST_BOOT && hal_gate_status(&drv_s1, &drv_s2)) foc_fault(F_DRV);

    /* CAN masters must keep sending setpoints; the serial console latches them */
    if (foc.state == ST_RUN && app_setpoint_src == SRC_CAN && now - app_setpoint_ms > CAN_TIMEOUT_MS)
        foc.req = REQ_IDLE;

    telem_tick(n);
    can_proto_telemetry(n, drv_s1, drv_s2);

    if (now - led_ms >= (foc.state == ST_FAULT ? 100u : 500u)) {
        led_ms = now;
        hal_led_toggle();
    }
}
