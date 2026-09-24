#include "adc.h"
#include "system.h"
#include "config.h"
#include <math.h>

#define ADC_CR_RS (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)
#define JTRIG_HRTIM2 ((19u << ADC_JSQR_JEXTSEL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos))

static void adc_cr_set(ADC_TypeDef *a, uint32_t bit) { a->CR = (a->CR & ~ADC_CR_RS) | bit; }

static void adc_enable(ADC_TypeDef *a)
{
    a->CR = 0;
    a->CR = ADC_CR_ADVREGEN;
    delay_us(30);
    adc_cr_set(a, ADC_CR_ADCAL);
    while (a->CR & ADC_CR_ADCAL) {}
    delay_us(5);
    a->ISR = ADC_ISR_ADRDY;
    adc_cr_set(a, ADC_CR_ADEN);
    while (!(a->ISR & ADC_ISR_ADRDY)) {}
    a->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_OVRMOD;
}

void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN;
    ADC12_COMMON->CCR = 3u << ADC_CCR_CKMODE_Pos;    /* HCLK/4 = 42.5 MHz */
    ADC345_COMMON->CCR = 3u << ADC_CCR_CKMODE_Pos;

    adc_enable(ADC1);
    adc_enable(ADC2);
    adc_enable(ADC3);

    /* ADC1: IB (ch5), VDC (ch7) injected; temps ch8/ch9 regular */
    ADC1->SMPR1 = (2u << ADC_SMPR1_SMP5_Pos) | (2u << ADC_SMPR1_SMP7_Pos) |
                  (4u << ADC_SMPR1_SMP8_Pos) | (4u << ADC_SMPR1_SMP9_Pos);
    ADC1->JSQR = JTRIG_HRTIM2 | (1u << ADC_JSQR_JL_Pos) | (5u << ADC_JSQR_JSQ1_Pos) | (7u << ADC_JSQR_JSQ2_Pos);
    /* ADC2: IA (ch15) */
    ADC2->SMPR2 = 2u << ADC_SMPR2_SMP15_Pos;
    ADC2->JSQR = JTRIG_HRTIM2 | (15u << ADC_JSQR_JSQ1_Pos);
    /* ADC3: IC (ch5) */
    ADC3->SMPR1 = 2u << ADC_SMPR1_SMP5_Pos;
    ADC3->JSQR = JTRIG_HRTIM2 | (5u << ADC_JSQR_JSQ1_Pos);

    ADC1->IER = ADC_IER_JEOSIE;
    adc_cr_set(ADC1, ADC_CR_JADSTART);
    adc_cr_set(ADC2, ADC_CR_JADSTART);
    adc_cr_set(ADC3, ADC_CR_JADSTART);
}

void adc_irq_enable(void)
{
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

static float ntc(uint32_t adc)
{
    if (adc < 10 || adc > 4085) return 999.0f;   /* open or shorted sensor reads as hot */
    float r = NTC_R_TOP * adc / (4096.0f - adc);
    return 1.0f / (1.0f / 298.15f + logf(r / NTC_R25) / NTC_BETA) - 273.15f;
}

/* one NTC channel per call: ch9 = FET, ch8 = ambient */
void temp_poll(float *t_fet, float *t_amb)
{
    static int ch = 8, busy;
    if (busy) {
        if (!(ADC1->ISR & ADC_ISR_EOC)) return;
        float t = ntc(ADC1->DR);
        if (ch == 9) *t_fet = t; else *t_amb = t;
        ch = (ch == 8) ? 9 : 8;
    }
    ADC1->SQR1 = (uint32_t)ch << ADC_SQR1_SQ1_Pos;
    adc_cr_set(ADC1, ADC_CR_ADSTART);
    busy = 1;
}
