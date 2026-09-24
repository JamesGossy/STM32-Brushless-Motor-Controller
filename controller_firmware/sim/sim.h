#pragma once
#include <stdint.h>
#include <stddef.h>
#include "motor_model.h"

/* Software-in-the-loop harness: the real app code runs against a simulated
   board (hal.h implemented here) and a PMSM + inverter + sensor model. */

typedef struct {
    double vbus;
    int enc_dir;            /* true encoder direction vs rotor */
    double enc_offset;      /* true encoder offset, rad */
    int cur_sign;           /* true current sense polarity */
    double adc_offset[3];   /* ADC offset error, counts */
    unsigned seed;
} sim_config_t;

void sim_default_config(sim_config_t *c);
void sim_init(const sim_config_t *c);
void sim_run_ms(uint32_t ms);
uint32_t sim_millis(void);

motor_t *sim_motor(void);
int sim_pwm_enabled(void);
const double *sim_duty(void);

void sim_set_vbus(double v);
void sim_set_load(double nm);
void sim_lock_rotor(int on);
void sim_set_adc_bias(int phase, double amps);
void sim_freeze_encoder(int on);
void sim_set_encoder_offset(double rad);
void sim_set_gate_fault(int on);
void sim_set_temp(double fet, double amb);

void sim_can_inject(uint32_t id, const void *d, uint8_t len);
int sim_can_pop(uint32_t *id, uint8_t *d, uint8_t *len);
void sim_serial_inject(const char *s);
size_t sim_serial_take(uint8_t *d, size_t max);
