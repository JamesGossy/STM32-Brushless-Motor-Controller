#include "sim.h"
#include "hal.h"
#include "app.h"
#include "config.h"
#include <math.h>
#include <string.h>

#define SUBSTEPS 20
#define SIM_2PI 6.283185307179586
#define CAN_Q 64
#define SER_Q 65536

typedef struct { uint32_t id; uint8_t d[8], len; } can_msg_t;

static sim_config_t cfg_;
static motor_t motor;
static double duty_active[3] = {0.5, 0.5, 0.5}, duty_next[3] = {0.5, 0.5, 0.5};
static int pwm_en, enc_frozen, gate_fault;
static uint16_t enc_hold;
static double adc_bias[3], t_fet = 25.0, t_amb = 25.0, temp_forced = -1.0;
static uint32_t ms, rng;

static can_msg_t can_rx[CAN_Q], can_tx[CAN_Q];
static unsigned can_rx_h, can_rx_t, can_tx_h, can_tx_t;
static uint8_t ser_rx[SER_Q], ser_tx[SER_Q];
static size_t ser_rx_h, ser_rx_t, ser_tx_h, ser_tx_t;
static uint8_t nv[64];
static int nv_valid;

/* ---- hal.h implementation ---- */

void hal_init(void) {}
void hal_start(void) {}
uint32_t hal_millis(void) { return ms; }
uint32_t hal_irq_save(void) { return 0; }
void hal_irq_restore(uint32_t s) { (void)s; }

void hal_pwm_set(float da, float db, float dc)
{
    duty_next[0] = da; duty_next[1] = db; duty_next[2] = dc;
}

void hal_pwm_enable(int on) { pwm_en = on; }

int hal_gate_init(void) { return 1; }
void hal_gate_clear(void) { gate_fault = 0; }

int hal_gate_status(uint16_t *s1, uint16_t *s2)
{
    *s1 = gate_fault ? 0x400 : 0;
    *s2 = 0;
    return gate_fault;
}

void hal_temp_poll(float *fet, float *amb)
{
    *fet = (float)t_fet;
    *amb = (float)t_amb;
}

int hal_can_send(uint32_t id, const uint8_t *d, uint8_t len)
{
    unsigned h = (can_tx_h + 1) % CAN_Q;
    if (h == can_tx_t) return 0;
    can_tx[can_tx_h].id = id;
    can_tx[can_tx_h].len = len;
    memcpy(can_tx[can_tx_h].d, d, len > 8 ? 8 : len);
    can_tx_h = h;
    return 1;
}

int hal_can_recv(uint32_t *id, uint8_t *d, uint8_t *len)
{
    if (can_rx_t == can_rx_h) return 0;
    *id = can_rx[can_rx_t].id;
    *len = can_rx[can_rx_t].len;
    memcpy(d, can_rx[can_rx_t].d, 8);
    can_rx_t = (can_rx_t + 1) % CAN_Q;
    return 1;
}

int hal_serial_write(const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        size_t h = (ser_tx_h + 1) % SER_Q;
        if (h == ser_tx_t) return 0;
        ser_tx[ser_tx_h] = d[i];
        ser_tx_h = h;
    }
    return 1;
}

size_t hal_serial_read(uint8_t *d, size_t max)
{
    size_t n = 0;
    while (n < max && ser_rx_t != ser_rx_h) {
        d[n++] = ser_rx[ser_rx_t];
        ser_rx_t = (ser_rx_t + 1) % SER_Q;
    }
    return n;
}

int hal_nv_read(void *d, size_t n)
{
    if (!nv_valid || n > sizeof nv) return 0;
    memcpy(d, nv, n);
    return 1;
}

int hal_nv_write(const void *d, size_t n)
{
    if (n > sizeof nv) return 0;
    memcpy(nv, d, n);
    nv_valid = 1;
    return 1;
}

void hal_led_toggle(void) {}
void hal_wdg_kick(void) {}

/* ---- sensors ---- */

static int noise(void)
{
    rng = rng * 1103515245u + 12345u;
    return (int)((rng >> 16) % 5) - 2;
}

static uint16_t adc(double counts)
{
    counts += noise();
    if (counts < 0) counts = 0;
    if (counts > 4095) counts = 4095;
    return (uint16_t)lround(counts);
}

static void sample(hal_sample_t *s)
{
    double i[3];
    motor_currents(&motor, i);
    double k = cfg_.cur_sign / (double)I_PER_COUNT;
    s->ia = adc(2048 + cfg_.adc_offset[0] + (i[0] + adc_bias[0]) * k);
    s->ib = adc(2048 + cfg_.adc_offset[1] + (i[1] + adc_bias[1]) * k);
    s->ic = adc(2048 + cfg_.adc_offset[2] + (i[2] + adc_bias[2]) * k);
    s->vbus = adc(motor.vbus / V_PER_COUNT);
    double raw = fmod(cfg_.enc_dir * motor.th_m + cfg_.enc_offset, SIM_2PI);
    if (raw < 0) raw += SIM_2PI;
    uint16_t e = (uint16_t)((uint32_t)(raw / (SIM_2PI) * 65536.0) & 0xFFFCu);   /* 14-bit */
    if (!enc_frozen) enc_hold = e;
    s->enc = enc_hold;
}

static void thermal(double dt)
{
    if (temp_forced >= 0) { t_fet = temp_forced; return; }
    double p = 1.5 * motor.rs * (motor.id * motor.id + motor.iq * motor.iq);
    t_fet += ((t_amb + 2.0 * p) - t_fet) * dt / 20.0;
}

/* ---- harness ---- */

void sim_default_config(sim_config_t *c)
{
    memset(c, 0, sizeof *c);
    c->vbus = VBUS_NOMINAL;
    c->enc_dir = -1;
    c->enc_offset = 2.0;
    c->cur_sign = -1;
    c->adc_offset[0] = 7; c->adc_offset[1] = -5; c->adc_offset[2] = 3;
    c->seed = 1;
}

void sim_init(const sim_config_t *c)
{
    cfg_ = *c;
    rng = c->seed;
    motor_init(&motor);
    motor.vbus = c->vbus;
    hal_init();
    app_init();
    hal_start();
}

void sim_run_ms(uint32_t n)
{
    const double dt = 1.0 / F_PWM / SUBSTEPS;
    while (n--) {
        for (int k = 0; k < (int)(F_PWM / 1000); k++) {
            hal_sample_t s;
            sample(&s);
            app_control_isr(&s);
            for (int j = 0; j < SUBSTEPS; j++) motor_step(&motor, duty_active, pwm_en, dt);
            memcpy(duty_active, duty_next, sizeof duty_active);
        }
        thermal(1e-3);
        app_poll();
        ms++;
        app_tick();
    }
}

uint32_t sim_millis(void) { return ms; }
motor_t *sim_motor(void) { return &motor; }
int sim_pwm_enabled(void) { return pwm_en; }
const double *sim_duty(void) { return duty_active; }

void sim_set_vbus(double v) { motor.vbus = v; }
void sim_set_load(double nm) { motor.t_load = nm; }
void sim_lock_rotor(int on) { motor.locked = on; }
void sim_set_adc_bias(int ph, double a) { if (ph >= 0 && ph < 3) adc_bias[ph] = a; }
void sim_freeze_encoder(int on) { enc_frozen = on; }
void sim_set_encoder_offset(double rad) { cfg_.enc_offset = rad; }
void sim_set_gate_fault(int on) { gate_fault = on; }
void sim_set_temp(double fet, double amb) { temp_forced = fet; t_fet = fet; t_amb = amb; }

void sim_can_inject(uint32_t id, const void *d, uint8_t len)
{
    unsigned h = (can_rx_h + 1) % CAN_Q;
    if (h == can_rx_t) return;
    can_rx[can_rx_h].id = id;
    can_rx[can_rx_h].len = len;
    memset(can_rx[can_rx_h].d, 0, 8);
    memcpy(can_rx[can_rx_h].d, d, len > 8 ? 8 : len);
    can_rx_h = h;
}

int sim_can_pop(uint32_t *id, uint8_t *d, uint8_t *len)
{
    if (can_tx_t == can_tx_h) return 0;
    *id = can_tx[can_tx_t].id;
    *len = can_tx[can_tx_t].len;
    memcpy(d, can_tx[can_tx_t].d, 8);
    can_tx_t = (can_tx_t + 1) % CAN_Q;
    return 1;
}

void sim_serial_inject(const char *s)
{
    while (*s) {
        size_t h = (ser_rx_h + 1) % SER_Q;
        if (h == ser_rx_t) return;
        ser_rx[ser_rx_h] = (uint8_t)*s++;
        ser_rx_h = h;
    }
}

size_t sim_serial_take(uint8_t *d, size_t max)
{
    size_t n = 0;
    while (n < max && ser_tx_t != ser_tx_h) {
        d[n++] = ser_tx[ser_tx_t];
        ser_tx_t = (ser_tx_t + 1) % SER_Q;
    }
    return n;
}
