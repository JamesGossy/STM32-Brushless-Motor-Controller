/*
 * app.h - the application's main loop and the command API used by both the
 * CAN protocol and the serial console.
 */
#pragma once
#include <stdint.h>

/* where the last setpoint came from */
enum { SRC_NONE, SRC_CAN, SRC_SERIAL };

/* requested drive states for app_request() */
enum { RUN_IDLE, RUN_TORQUE, RUN_SPEED, RUN_CALIBRATE };

extern volatile uint8_t app_save_req;
extern volatile uint8_t app_setpoint_src;
extern volatile uint32_t app_setpoint_ms;

void app_init(void);
void app_poll(void);
void app_tick(void);

const char *app_request(uint8_t run, uint8_t src);
void app_set_iq(float amps, uint8_t src);
void app_set_speed(float rad_s, uint8_t src);
void app_set_limits(float amps, float rad_s);
void app_clear_faults(void);
const char *app_fault_text(uint16_t faults);
