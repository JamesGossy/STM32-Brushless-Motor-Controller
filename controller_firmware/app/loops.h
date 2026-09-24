/*
 * loops.h - current, speed and field-weakening controllers.
 */
#pragma once

void loops_init(void);
void loops_reset(void);
void loops_speed_bumpless(float speed);
void loops_run(float id, float iq, float theta_e, float we);
