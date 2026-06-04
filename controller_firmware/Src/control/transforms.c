#include "control/transforms.h"
#include <math.h>

/*
 * Inverse Park transform: rotate (Vd, Vq) by theta back to stationary frame.
 *   Valpha =  Vd*cos(theta) - Vq*sin(theta)
 *   Vbeta  =  Vd*sin(theta) + Vq*cos(theta)
 */
void ParkInv(float Vd, float Vq, float theta, float *Valpha, float *Vbeta)
{
    float cosT = cosf(theta);
    float sinT = sinf(theta);
    *Valpha = Vd * cosT - Vq * sinT;
    *Vbeta  = Vd * sinT + Vq * cosT;
}

/*
 * Inverse Clarke transform: stationary alpha/beta -> three-phase.
 *   Va =  Valpha
 *   Vb = -Valpha/2 + Vbeta*sqrt(3)/2
 *   Vc = -Valpha/2 - Vbeta*sqrt(3)/2
 *
 * Assumes balanced (Va + Vb + Vc = 0) so no zero-sequence component.
 */
void ClarkeInv(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc)
{
    static const float SQRT3_2 = 0.86602540378f;  /* sqrt(3)/2 */
    *Va =  Valpha;
    *Vb = -0.5f * Valpha + SQRT3_2 * Vbeta;
    *Vc = -0.5f * Valpha - SQRT3_2 * Vbeta;
}
