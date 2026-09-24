/*
 * sim.h - software-in-the-loop harness.
 *
 * Runs the real app/ code against a simulated board: this module implements
 * hal.h on top of the motor model and adds hooks to inject setpoints and
 * faults. Used by the tests and by foc_sim.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "motor_model.h"

typedef struct {
    double vbus;
    int enc_dir;            /* true encoder direction (calibration should find it) */
    double enc_offset;      /* true encoder offset, rad */
    int cur_sign;           /* true current sense polarity */
    double adc_offset[3];   /* current sense offset error, ADC counts */
    unsigned seed;          /* ADC noise seed */
} sim_config_t;

void sim_default_config(sim_config_t *c);
void sim_init(const sim_config_t *c);
void sim_run_ms(uint32_t ms);
uint32_t sim_millis(void);

/* observe */
motor_t *sim_motor(void);
int sim_pwm_enabled(void);

/* plant and fault injection */
void sim_set_vbus(double volts);
void sim_set_load(double nm);
void sim_lock_rotor(int on);
void sim_set_adc_bias(int phase, double amps);
void sim_freeze_encoder(int on);
void sim_set_encoder_offset(double rad);
void sim_set_gate_fault(int on);
void sim_set_temp(double fet, double amb);

/* CAN and serial link */
void sim_can_inject(uint32_t id, const void *data, uint8_t len);
int sim_can_pop(uint32_t *id, uint8_t *data, uint8_t *len);
void sim_serial_inject(const char *text);
size_t sim_serial_take(uint8_t *data, size_t max);
