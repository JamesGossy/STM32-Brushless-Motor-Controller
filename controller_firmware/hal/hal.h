/*
 * hal.h - hardware abstraction layer.
 *
 * Everything the application needs from a board. There are two
 * implementations: hal/stm32g474 for the real controller and sim/ for the
 * software-in-the-loop simulator. The app/ code only ever includes this file.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* One sample, taken at the centre of the low-side on time each PWM period. */
typedef struct {
    uint16_t ia, ib, ic;    /* phase current ADC counts (mid-scale = 0 A) */
    uint16_t vbus;          /* bus voltage ADC counts */
    uint16_t enc;           /* encoder angle, 65536 = one mechanical turn */
} hal_sample_t;

/* Provided by the app, called by the HAL at F_PWM with each new sample. */
void app_control_isr(const hal_sample_t *s);

/* start-up */
void hal_init(void);            /* clocks and peripherals, PWM outputs off */
void hal_start(void);           /* start the PWM timebase, control interrupt and watchdog */
uint32_t hal_millis(void);

/* critical sections */
uint32_t hal_irq_save(void);
void hal_irq_restore(uint32_t state);

/* power stage: high-side duty per phase 0..1, takes effect next period */
void hal_pwm_set(float da, float db, float dc);
void hal_pwm_enable(int on);

/* gate driver */
int hal_gate_init(void);                            /* returns 1 if configured OK */
int hal_gate_status(uint16_t *s1, uint16_t *s2);    /* returns 1 if the driver reports a fault */
void hal_gate_clear(void);

/* slow sensors */
void hal_temp_poll(float *t_fet, float *t_amb);

/* CAN: standard 11-bit ids, up to 8 bytes. Both return 1 on success. */
int hal_can_send(uint32_t id, const uint8_t *data, uint8_t len);
int hal_can_recv(uint32_t *id, uint8_t *data, uint8_t *len);

/* serial link (USB CDC on the board, TCP in the simulator) */
int hal_serial_write(const uint8_t *data, size_t len);
size_t hal_serial_read(uint8_t *data, size_t max);

/* non-volatile settings storage */
int hal_nv_read(void *data, size_t len);
int hal_nv_write(const void *data, size_t len);

/* misc */
void hal_led_toggle(void);
void hal_wdg_kick(void);
