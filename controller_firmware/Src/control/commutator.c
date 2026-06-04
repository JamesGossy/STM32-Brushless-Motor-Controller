#include "control/commutator.h"
#include <math.h>

#define TWO_PI  6.28318530f
#define STEP_ANGLE (TWO_PI / 6.0f)   /* 60° per step */

void Commutator_init(Commutator_t *c)
{
    c->angle = 0.0f;
    c->step  = 0u;
}

void Commutator_update(Commutator_t *c, float freqHz, float ts)
{
    c->angle += freqHz * TWO_PI * ts;
    if (c->angle >= TWO_PI)
        c->angle -= TWO_PI;

    c->step = (uint8_t)(c->angle / STEP_ANGLE) % 6u;
}
