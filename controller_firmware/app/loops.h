#pragma once
#include <stdint.h>

void loops_init(void);
void loops_reset(void);
void loops_speed_bumpless(float w);
void loops_run(float id, float iq, float th, float we);
