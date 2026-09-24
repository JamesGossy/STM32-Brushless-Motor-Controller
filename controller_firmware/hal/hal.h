#pragma once
#include <stdint.h>
#include <stddef.h>

/* Everything the application needs from a board. Implemented by
   hal/stm32g474 (real hardware) and sim/ (software-in-the-loop). */

typedef struct {
    uint16_t ia, ib, ic;    /* phase current ADC counts */
    uint16_t vbus;          /* bus voltage ADC counts */
    uint16_t enc;           /* encoder angle, 16-bit full turn */
} hal_sample_t;

/* called by the HAL at F_PWM, right after the current sample */
void app_control_isr(const hal_sample_t *s);

void hal_init(void);
void hal_start(void);
uint32_t hal_millis(void);

uint32_t hal_irq_save(void);
void hal_irq_restore(uint32_t state);

void hal_pwm_set(float da, float db, float dc);
void hal_pwm_enable(int on);

int hal_gate_init(void);
int hal_gate_status(uint16_t *s1, uint16_t *s2);   /* returns 1 on driver fault */
void hal_gate_clear(void);

void hal_temp_poll(float *t_fet, float *t_amb);

int hal_can_send(uint32_t id, const uint8_t *d, uint8_t len);
int hal_can_recv(uint32_t *id, uint8_t *d, uint8_t *len);

int hal_serial_write(const uint8_t *d, size_t n);
size_t hal_serial_read(uint8_t *d, size_t max);

int hal_nv_read(void *d, size_t n);
int hal_nv_write(const void *d, size_t n);

void hal_led_toggle(void);
void hal_wdg_kick(void);
