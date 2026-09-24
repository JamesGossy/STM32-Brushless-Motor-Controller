#pragma once
#include <stdint.h>

enum { SRC_NONE, SRC_CAN, SRC_SERIAL };

extern volatile uint8_t app_save_req;
extern volatile uint8_t app_setpoint_src;
extern volatile uint32_t app_setpoint_ms;

void app_init(void);
void app_poll(void);
void app_tick(void);

void app_set_iq(float a, uint8_t src);
void app_set_speed(float rad_s, uint8_t src);
void app_set_state(uint8_t state, uint8_t src);   /* 0 idle, 1 torque, 2 speed, 3 calibrate */
void app_set_limits(float amps, float rad_s);
void app_clear_faults(void);
