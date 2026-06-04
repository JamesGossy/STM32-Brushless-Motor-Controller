#pragma once

#include <stdint.h>

/*
 * hal_adc.h — ADC for current sense and DC bus voltage
 *
 * Pin / ADC mapping (from peripheral_setup.txt):
 *   PA1 / ADC2_IN2  -> SOA  (phase A shunt amp output)
 *   PA0 / ADC1_IN1  -> SOB  (phase B shunt amp output)  [ADC1 not yet wired; use ADC2_IN1 if rerouted]
 *   PB1 / ADC3_IN1  -> VSENVM  (DC bus voltage divider)
 *   PB11/ ADC2_IN14 -> AMBIENT_TMP
 *   PB12/ ADC4_IN3  -> PHASE_TMP
 *
 * All conversions are software-triggered (polling), injected into the background loop.
 *
 * DC bus voltage conversion factor:
 *   Board uses a resistor divider — adjust ADC_VBUS_SCALE to match your divider ratio.
 *   Default assumes 100k / 4.7k -> Vbus = Vadc * (100 + 4.7) / 4.7 ≈ Vadc * 22.28
 *   (Tune this to your actual resistor values.)
 */

#define ADC_VREF_V          3.3f
#define ADC_FULL_SCALE      4095.0f

/* NTC thermistor scaling: 10kΩ B=3950 on bottom of divider, 4.7kΩ pull-up to 3.3V
 * R_NTC = 4700 * Vadc / (3.3 - Vadc)
 * T(°C) = 1 / (ln(R_NTC / 10000) / 3950 + 1/298.15) - 273.15 */
#define ADC_NTC_RPULLUP     4700.0f
#define ADC_NTC_R25         10000.0f
#define ADC_NTC_B           3950.0f
#define ADC_NTC_T25         298.15f   /* 25°C in Kelvin */
#define ADC_VBUS_SCALE      40.25f   /* 383k / 9.76k divider: (383 + 9.76) / 9.76 */

/* Current sense scaling: DRV8353 CSA gain=10 V/V, 7 mΩ shunt, VREF=3.3V
 * SOx = VREF/2 + (I * Rshunt * gain)  =>  I = (SOx - VREF/2) / (Rshunt * gain) */
#define ADC_CSA_VBIAS       (ADC_VREF_V / 2.0f)   /* 1.65 V output at zero current */
#define ADC_CSA_GAIN        10.0f
#define ADC_CSA_RSHUNT      0.007f                 /* 7 mΩ */
#define ADC_CSA_SCALE       (1.0f / (ADC_CSA_RSHUNT * ADC_CSA_GAIN))  /* V -> A */

void  ADC_Init(void);
void  ADC_ConvertCurrents(void);                 /* call from TIM1 ISR — converts DMA result to Amperes */
void  ADC_Sample(void);                          /* call from main loop — updates Vbus and temperatures */
void  ADC_Sample_Raw(float *vsoA, float *vsoB);  /* accumulates raw SOx volts, used during CSA calibration */
void  ADC_RearmHwTrigger(void);                  /* call once after CSA calibration loop */

extern volatile float g_adcSoA;          /* phase A current (A), offset-corrected */
extern volatile float g_adcSoB;          /* phase B current (A), offset-corrected */
extern volatile float g_adcSoA_offset;   /* amp offset in volts, set by DRV835x_CalibrateCSA */
extern volatile float g_adcSoB_offset;
extern volatile float g_adcVbus;      /* DC bus voltage (V) */
extern volatile float g_adcAmbTemp;   /* ambient temp sense (V) */
extern volatile float g_adcPhaseTemp; /* phase temp sense (V) */
