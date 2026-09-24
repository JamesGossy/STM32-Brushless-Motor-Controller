/*
 * adc.h - current, bus voltage and temperature measurement.
 *
 * Injected results each PWM period:
 *   ADC2 JDR1 = IA, ADC1 JDR1 = IB, ADC3 JDR1 = IC, ADC1 JDR2 = VBUS
 */
#pragma once

void adc_init(void);
void adc_irq_enable(void);
void temp_poll(float *t_fet, float *t_amb);
