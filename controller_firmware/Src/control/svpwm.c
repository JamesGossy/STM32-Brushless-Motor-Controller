#include "control/svpwm.h"

/*
 * Midpoint-clamp SVPWM (equivalent to conventional sector SVPWM).
 *
 * The zero-sequence injection v0 = -(Vmax + Vmin)/2 shifts the three phase
 * voltages so the average of the max and min lands at zero (midpoint of the
 * DC bus). This maximises linear modulation range to Vdc/sqrt(3) without a
 * lookup table and is identical in output to the classic 6-sector SVPWM.
 *
 * duty = (Va_shifted / Vdc) + 0.5
 *       in [0, 1] by construction when |V| <= Vdc/sqrt(3).
 */
void SVPWM_calc(float Va, float Vb, float Vc, float Vdc,
                float *dutyA, float *dutyB, float *dutyC)
{
    float Vmax = Va;
    if (Vb > Vmax) Vmax = Vb;
    if (Vc > Vmax) Vmax = Vc;

    float Vmin = Va;
    if (Vb < Vmin) Vmin = Vb;
    if (Vc < Vmin) Vmin = Vc;

    /* Zero-sequence injection (midpoint clamp) */
    float v0 = -0.5f * (Vmax + Vmin);

    float invVdc = 1.0f / Vdc;
    *dutyA = (Va + v0) * invVdc + 0.5f;
    *dutyB = (Vb + v0) * invVdc + 0.5f;
    *dutyC = (Vc + v0) * invVdc + 0.5f;

    /* Clamp in case of slight numerical overshoot */
    if (*dutyA > 1.0f) *dutyA = 1.0f; else if (*dutyA < 0.0f) *dutyA = 0.0f;
    if (*dutyB > 1.0f) *dutyB = 1.0f; else if (*dutyB < 0.0f) *dutyB = 0.0f;
    if (*dutyC > 1.0f) *dutyC = 1.0f; else if (*dutyC < 0.0f) *dutyC = 0.0f;
}
