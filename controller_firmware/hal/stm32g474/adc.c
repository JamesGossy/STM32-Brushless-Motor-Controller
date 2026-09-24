/*
 * adc.c - ADC1/2/3 setup.
 *
 * All three ADCs start together on HRTIM ADC trigger 2, so the phase
 * currents are sampled at the same instant. The NTCs are read on ADC1's
 * regular group from the main loop, in between the injected conversions.
 */
#include "adc.h"
#include "system.h"
#include "config.h"
#include <math.h>

/* ADC_CR bits that must never be written back as 1 by accident */
#define ADC_CR_SET_ONLY (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | \
                         ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)

/* injected trigger: HRTIM ADC trigger 2 (JEXTSEL 19), rising edge */
#define JTRIG_HRTIM2 ((19u << ADC_JSQR_JEXTSEL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos))

/* sample time codes */
#define SMP_12_5  2u
#define SMP_47_5  4u

/* Set one of the start/stop bits in ADC_CR without touching the others. */
static void adc_cr_set(ADC_TypeDef *adc, uint32_t bit)
{
    adc->CR = (adc->CR & ~ADC_CR_SET_ONLY) | bit;
}

/* Power up, calibrate and enable one ADC. */
static void adc_enable(ADC_TypeDef *adc)
{
    adc->CR = 0;                    /* leave deep power down */
    adc->CR = ADC_CR_ADVREGEN;
    delay_us(30);                   /* regulator start-up */
    adc_cr_set(adc, ADC_CR_ADCAL);
    while (adc->CR & ADC_CR_ADCAL) {}
    delay_us(5);

    adc->ISR = ADC_ISR_ADRDY;
    adc_cr_set(adc, ADC_CR_ADEN);
    while (!(adc->ISR & ADC_ISR_ADRDY)) {}
    adc->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_OVRMOD;
}

/* Set up the three ADCs and arm the injected conversions. */
void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN;
    ADC12_COMMON->CCR = 3u << ADC_CCR_CKMODE_Pos;      /* HCLK/4 = 42.5 MHz */
    ADC345_COMMON->CCR = 3u << ADC_CCR_CKMODE_Pos;

    adc_enable(ADC1);
    adc_enable(ADC2);
    adc_enable(ADC3);

    /* ADC1: IB (ch5) then VBUS (ch7) injected, NTCs on ch8/ch9 regular */
    ADC1->SMPR1 = (SMP_12_5 << ADC_SMPR1_SMP5_Pos) | (SMP_12_5 << ADC_SMPR1_SMP7_Pos) |
                  (SMP_47_5 << ADC_SMPR1_SMP8_Pos) | (SMP_47_5 << ADC_SMPR1_SMP9_Pos);
    ADC1->JSQR = JTRIG_HRTIM2 | (1u << ADC_JSQR_JL_Pos) | (5u << ADC_JSQR_JSQ1_Pos) | (7u << ADC_JSQR_JSQ2_Pos);

    /* ADC2: IA (ch15) */
    ADC2->SMPR2 = SMP_12_5 << ADC_SMPR2_SMP15_Pos;
    ADC2->JSQR = JTRIG_HRTIM2 | (15u << ADC_JSQR_JSQ1_Pos);

    /* ADC3: IC (ch5) */
    ADC3->SMPR1 = SMP_12_5 << ADC_SMPR1_SMP5_Pos;
    ADC3->JSQR = JTRIG_HRTIM2 | (5u << ADC_JSQR_JSQ1_Pos);

    /* ADC1 has the longest sequence, so its end-of-sequence runs the control loop */
    ADC1->IER = ADC_IER_JEOSIE;
    adc_cr_set(ADC1, ADC_CR_JADSTART);
    adc_cr_set(ADC2, ADC_CR_JADSTART);
    adc_cr_set(ADC3, ADC_CR_JADSTART);
}

/* The control loop interrupt has the highest priority. */
void adc_irq_enable(void)
{
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* NTC temperature from its divider reading (4.7k pull-up, NTC to ground).
   An open or shorted sensor reads as very hot so it trips overtemp. */
static float ntc_temp(uint32_t adc)
{
    if (adc < 10 || adc > 4085) return 999.0f;
    float r = NTC_R_TOP * adc / (4096.0f - adc);
    return 1.0f / (1.0f / 298.15f + logf(r / NTC_R25) / NTC_BETA) - 273.15f;
}

/* Called every millisecond: reads the previous conversion and starts the
   next, alternating ch9 (FET NTC) and ch8 (ambient NTC). */
void temp_poll(float *t_fet, float *t_amb)
{
    static int channel = 8, started;

    if (started) {
        if (!(ADC1->ISR & ADC_ISR_EOC)) return;
        float t = ntc_temp(ADC1->DR);
        if (channel == 9) *t_fet = t;
        else *t_amb = t;
        channel = channel == 8 ? 9 : 8;
    }

    ADC1->SQR1 = (uint32_t)channel << ADC_SQR1_SQ1_Pos;
    adc_cr_set(ADC1, ADC_CR_ADSTART);
    started = 1;
}
