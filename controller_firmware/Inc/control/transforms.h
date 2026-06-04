#pragma once

/*
 * Inverse Park: (Vd, Vq, theta) -> (Valpha, Vbeta)
 * Inverse Clarke: (Valpha, Vbeta) -> (Va, Vb, Vc)
 */

void ParkInv(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);
void ClarkeInv(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc);
