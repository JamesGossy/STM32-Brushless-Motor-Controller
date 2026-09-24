/*
 * sim.c - simulated board (hal.h) and the harness that steps it.
 *
 * Each PWM period: sample the plant -> run the control ISR -> integrate the
 * plant over the period with the duties from the PREVIOUS ISR (the real
 * HRTIM applies new duties at the next period, which is the delay the
 * firmware compensates). Every millisecond the main-loop functions run.
 */
#include "sim.h"
#include "hal.h"
#include "app.h"
#include "config.h"
#include <math.h>
#include <string.h>

#define SUBSTEPS   20                   /* plant integration steps per PWM period */
#define TWO_PI_D   6.283185307179586
#define CAN_DEPTH  64
#define SER_DEPTH  65536

typedef struct { uint32_t id; uint8_t data[8], len; } can_frame_t;
typedef struct { can_frame_t f[CAN_DEPTH]; unsigned head, tail; } can_queue_t;
typedef struct { uint8_t b[SER_DEPTH]; size_t head, tail; } byte_queue_t;

static sim_config_t config;
static motor_t motor;
static double duty_now[3] = {0.5, 0.5, 0.5}, duty_next[3] = {0.5, 0.5, 0.5};
static int pwm_enabled, encoder_frozen, gate_fault;
static uint16_t encoder_hold;
static double adc_bias[3];
static double t_fet = 25.0, t_amb = 25.0, t_forced = -1.0;
static uint32_t now_ms, rng;

static can_queue_t can_rx, can_tx;
static byte_queue_t serial_rx, serial_tx;
static uint8_t nv[64];
static int nv_written;

/* ---- queue helpers ---- */

static int can_push(can_queue_t *q, uint32_t id, const void *data, uint8_t len)
{
    unsigned next = (q->head + 1) % CAN_DEPTH;
    if (next == q->tail) return 0;
    can_frame_t *f = &q->f[q->head];
    f->id = id;
    f->len = len > 8 ? 8 : len;
    memset(f->data, 0, 8);
    memcpy(f->data, data, f->len);
    q->head = next;
    return 1;
}

static int can_pop(can_queue_t *q, uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (q->tail == q->head) return 0;
    can_frame_t *f = &q->f[q->tail];
    *id = f->id;
    *len = f->len;
    memcpy(data, f->data, 8);
    q->tail = (q->tail + 1) % CAN_DEPTH;
    return 1;
}

static int bytes_push(byte_queue_t *q, const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        size_t next = (q->head + 1) % SER_DEPTH;
        if (next == q->tail) return 0;
        q->b[q->head] = d[i];
        q->head = next;
    }
    return 1;
}

static size_t bytes_pop(byte_queue_t *q, uint8_t *d, size_t max)
{
    size_t n = 0;
    while (n < max && q->tail != q->head) {
        d[n++] = q->b[q->tail];
        q->tail = (q->tail + 1) % SER_DEPTH;
    }
    return n;
}

/* ---- hal.h ---- */

void hal_init(void) {}
void hal_start(void) {}
uint32_t hal_millis(void) { return now_ms; }
uint32_t hal_irq_save(void) { return 0; }
void hal_irq_restore(uint32_t state) { (void)state; }

void hal_pwm_set(float da, float db, float dc)
{
    duty_next[0] = da;
    duty_next[1] = db;
    duty_next[2] = dc;
}

void hal_pwm_enable(int on) { pwm_enabled = on; }

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

int hal_can_send(uint32_t id, const uint8_t *d, uint8_t len) { return can_push(&can_tx, id, d, len); }
int hal_can_recv(uint32_t *id, uint8_t *d, uint8_t *len) { return can_pop(&can_rx, id, d, len); }

int hal_serial_write(const uint8_t *d, size_t n) { return bytes_push(&serial_tx, d, n); }
size_t hal_serial_read(uint8_t *d, size_t max) { return bytes_pop(&serial_rx, d, max); }

int hal_nv_read(void *d, size_t n)
{
    if (!nv_written || n > sizeof nv) return 0;
    memcpy(d, nv, n);
    return 1;
}

int hal_nv_write(const void *d, size_t n)
{
    if (n > sizeof nv) return 0;
    memcpy(nv, d, n);
    nv_written = 1;
    return 1;
}

void hal_led_toggle(void) {}
void hal_wdg_kick(void) {}

/* ---- sensors ---- */

/* +/- 2 counts of repeatable pseudo-random ADC noise. */
static int noise(void)
{
    rng = rng * 1103515245u + 12345u;
    return (int)((rng >> 16) % 5) - 2;
}

static uint16_t to_adc(double counts)
{
    counts += noise();
    if (counts < 0) counts = 0;
    if (counts > 4095) counts = 4095;
    return (uint16_t)lround(counts);
}

/* What the ADCs and encoder would read right now. */
static void take_sample(hal_sample_t *s)
{
    double i[3];
    motor_currents(&motor, i);

    double counts_per_amp = config.cur_sign / (double)I_PER_COUNT;
    s->ia = to_adc(2048 + config.adc_offset[0] + (i[0] + adc_bias[0]) * counts_per_amp);
    s->ib = to_adc(2048 + config.adc_offset[1] + (i[1] + adc_bias[1]) * counts_per_amp);
    s->ic = to_adc(2048 + config.adc_offset[2] + (i[2] + adc_bias[2]) * counts_per_amp);
    s->vbus = to_adc(motor.vbus / V_PER_COUNT);

    double angle = fmod(config.enc_dir * motor.th_m + config.enc_offset, TWO_PI_D);
    if (angle < 0) angle += TWO_PI_D;
    uint16_t enc = (uint16_t)((uint32_t)(angle / TWO_PI_D * 65536.0) & 0xFFFCu);    /* 14-bit sensor */
    if (!encoder_frozen) encoder_hold = enc;
    s->enc = encoder_hold;
}

/* First-order FET temperature: rises with copper loss, 20 s time constant. */
static void update_temperature(double dt)
{
    if (t_forced >= 0) {
        t_fet = t_forced;
        return;
    }
    double loss = 1.5 * motor.rs * (motor.id * motor.id + motor.iq * motor.iq);
    t_fet += ((t_amb + 2.0 * loss) - t_fet) * dt / 20.0;
}

/* ---- harness ---- */

/* Deliberately "wrong" sensors so calibration has something to find. */
void sim_default_config(sim_config_t *c)
{
    memset(c, 0, sizeof *c);
    c->vbus = VBUS_NOMINAL;
    c->enc_dir = -1;
    c->enc_offset = 2.0;
    c->cur_sign = -1;
    c->adc_offset[0] = 7;
    c->adc_offset[1] = -5;
    c->adc_offset[2] = 3;
    c->seed = 1;
}

/* Reset the plant and boot the firmware. */
void sim_init(const sim_config_t *c)
{
    config = *c;
    rng = c->seed;
    motor_init(&motor);
    motor.vbus = c->vbus;

    hal_init();
    app_init();
    hal_start();
}

/* Run `ms` milliseconds of simulated time. */
void sim_run_ms(uint32_t ms)
{
    const double dt = 1.0 / F_PWM / SUBSTEPS;

    while (ms--) {
        for (unsigned k = 0; k < F_PWM / 1000; k++) {
            hal_sample_t s;
            take_sample(&s);
            app_control_isr(&s);

            for (int j = 0; j < SUBSTEPS; j++)
                motor_step(&motor, duty_now, pwm_enabled, dt);
            memcpy(duty_now, duty_next, sizeof duty_now);   /* new duties take effect next period */
        }

        update_temperature(1e-3);
        app_poll();
        now_ms++;
        app_tick();
    }
}

uint32_t sim_millis(void) { return now_ms; }
motor_t *sim_motor(void) { return &motor; }
int sim_pwm_enabled(void) { return pwm_enabled; }

void sim_set_vbus(double volts) { motor.vbus = volts; }
void sim_set_load(double nm) { motor.t_load = nm; }
void sim_lock_rotor(int on) { motor.locked = on; }
void sim_freeze_encoder(int on) { encoder_frozen = on; }
void sim_set_encoder_offset(double rad) { config.enc_offset = rad; }
void sim_set_gate_fault(int on) { gate_fault = on; }

void sim_set_adc_bias(int phase, double amps)
{
    if (phase >= 0 && phase < 3) adc_bias[phase] = amps;
}

void sim_set_temp(double fet, double amb)
{
    t_forced = fet;
    t_fet = fet;
    t_amb = amb;
}

void sim_can_inject(uint32_t id, const void *d, uint8_t len) { can_push(&can_rx, id, d, len); }
int sim_can_pop(uint32_t *id, uint8_t *d, uint8_t *len) { return can_pop(&can_tx, id, d, len); }

void sim_serial_inject(const char *text) { bytes_push(&serial_rx, (const uint8_t *)text, strlen(text)); }
size_t sim_serial_take(uint8_t *d, size_t max) { return bytes_pop(&serial_tx, d, max); }
