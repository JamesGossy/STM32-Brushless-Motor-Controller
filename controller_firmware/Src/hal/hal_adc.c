/*
 * hal_adc.c — ADC sampling for current sense and bus voltage
 *
 * Current sense (SOA, SOB) — synchronous, hardware-triggered:
 *   ADC1 (master) + ADC2 (slave) run in dual simultaneous mode, triggered by
 *   TIM1 TRGO (update event = counter peak in center-aligned mode 2). DMA1
 *   channel 1 transfers the packed CDR word to s_dma_cdr after each pair.
 *   The TIM1 ISR reads s_dma_cdr and converts to Amperes — sample point is
 *   always at PWM center, where low-side switches are fully on and the shunt
 *   voltage has settled.
 *
 *   ADC1_IN1 (PA0) -> SOB   (master — result in CDR[15:0])
 *   ADC2_IN2 (PA1) -> SOA   (slave  — result in CDR[31:16])
 *
 * Slow channels — software-triggered, background (main loop):
 *   ADC3_IN1  (PB1)  -> VSENVM / DC bus
 *   ADC2_IN14 (PB11) -> AMBIENT_TMP
 *   ADC4_IN3  (PB12) -> PHASE_TMP
 *
 *   ADC2 ambient temp read: temporarily reconfigure ADC2 for sw-trigger single
 *   conversion on ch14, then restore dual-simultaneous trigger for the next PWM
 *   period. The window is the ~20 ms between telemetry frames so there is no
 *   risk of conflicting with a hardware-triggered current sample.
 */

#include "hal/hal_adc.h"
#include "stm32g474xx.h"
#include <math.h>

volatile float g_adcSoA          = 0.0f;
volatile float g_adcSoB          = 0.0f;
volatile float g_adcSoA_offset   = 0.0f;
volatile float g_adcSoB_offset   = 0.0f;
volatile float g_adcVbus         = 0.0f;
volatile float g_adcAmbTemp      = 0.0f;
volatile float g_adcPhaseTemp    = 0.0f;

/* DMA destination: CDR[15:0] = ADC2 (SOA master), CDR[31:16] = ADC1 (SOB slave) */
static volatile uint32_t s_dma_cdr = 0u;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static void adc_calibrate_and_enable(ADC_TypeDef *adc)
{
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    for (volatile uint32_t i = 0; i < 3400u; i++);

    adc->CR &= ~ADC_CR_ADCALDIF;
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL);

    adc->ISR = ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;
    while (!(adc->ISR & ADC_ISR_ADRDY));
}

/* Software-triggered single conversion — used for slow background channels
 * (ADC3 Vbus, ADC4 phase temp) and temporarily for ADC2 ambient temp. */
static uint16_t adc_read_channel(ADC_TypeDef *adc, uint32_t ch)
{
    adc->SQR1 = (ch << ADC_SQR1_SQ1_Pos) | (0u << ADC_SQR1_L_Pos);

    if (ch <= 9u)
        adc->SMPR1 = (3u << (ch * 3u));
    else
        adc->SMPR2 = (3u << ((ch - 10u) * 3u));

    adc->ISR = ADC_ISR_EOC;
    adc->CR |= ADC_CR_ADSTART;
    while (!(adc->ISR & ADC_ISR_EOC));
    return (uint16_t)(adc->DR & 0xFFFu);
}

/* ------------------------------------------------------------------ */
/* ADC_Init                                                            */
/* ------------------------------------------------------------------ */
void ADC_Init(void)
{
    /* GPIO analog mode ------------------------------------------------ */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    GPIOA->MODER |= (3u << GPIO_MODER_MODE0_Pos)   /* PA0 SOB  ADC1_IN1 */
                  | (3u << GPIO_MODER_MODE1_Pos);   /* PA1 SOA  ADC2_IN2 */
    GPIOB->MODER |= (3u << GPIO_MODER_MODE1_Pos)   /* PB1  VSENVM ADC3_IN1  */
                  | (3u << GPIO_MODER_MODE11_Pos)   /* PB11 AMB_TMP ADC2_IN14 */
                  | (3u << GPIO_MODER_MODE12_Pos);  /* PB12 PHS_TMP ADC4_IN3  */

    /* ADC clocks ------------------------------------------------------ */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN;
    (void)RCC->AHB2ENR;

    /* Synchronous clock HCLK/1 for all ADC groups */
    ADC12_COMMON->CCR  = (1u << ADC_CCR_CKMODE_Pos);
    ADC345_COMMON->CCR = (1u << ADC_CCR_CKMODE_Pos);

    /* DMA1 channel 1 -------------------------------------------------- *
     * Source : ADC12_COMMON->CDR (32-bit packed dual result)             *
     * Dest   : s_dma_cdr                                                 *
     * One transfer per dual conversion, circular, 32-bit word width.     *
     * DMA1 ch1 request is mapped to ADC1 on STM32G4 via DMAMUX1 ch0.    */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    (void)RCC->AHB1ENR;

    DMA1_Channel1->CCR = 0u;  /* disable before configuring */

    DMA1_Channel1->CPAR  = (uint32_t)&ADC12_COMMON->CDR;
    DMA1_Channel1->CMAR  = (uint32_t)&s_dma_cdr;
    DMA1_Channel1->CNDTR = 1u;

    /* CCR: circular, mem-inc off, psize=32, msize=32, priority high, read-from-periph */
    DMA1_Channel1->CCR = DMA_CCR_CIRC
                       | DMA_CCR_PL_1          /* priority high */
                       | DMA_CCR_PSIZE_1       /* peripheral size 32-bit */
                       | DMA_CCR_MSIZE_1       /* memory size 32-bit */
                       | DMA_CCR_EN;

    /* DMAMUX1 ch0 -> ADC1 request (request ID = 5 on STM32G4) */
    DMAMUX1_Channel0->CCR = 5u;

    /* Calibrate and enable all four ADCs ------------------------------ */
    adc_calibrate_and_enable(ADC1);
    adc_calibrate_and_enable(ADC2);
    adc_calibrate_and_enable(ADC3);
    adc_calibrate_and_enable(ADC4);

    /* Configure ADC1+ADC2 dual simultaneous, TIM1 TRGO trigger -------- *
     *                                                                    *
     * ADC12_COMMON CCR:                                                  *
     *   DUAL  = 0x06 (regular simultaneous mode)                         *
     *   MDMA  = 0x03 (DMA mode 2: issue DMA request after each dual EOC) *
     *   CKMODE already set above, preserved with RMW                     */
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~(ADC_CCR_DUAL_Msk | ADC_CCR_MDMA_Msk))
                      | (6u  << ADC_CCR_DUAL_Pos)   /* regular simultaneous */
                      | (3u  << ADC_CCR_MDMA_Pos);  /* DMA mode 2 */

    /* ADC2 (slave): channel 2 (SOA), 47.5-cycle sample time, 1 rank */
    ADC2->SQR1  = (2u << ADC_SQR1_SQ1_Pos) | (0u << ADC_SQR1_L_Pos);
    ADC2->SMPR1 = (3u << (2u * 3u));

    /* ADC1 (master): channel 1 (SOB), 47.5-cycle sample time, 1 rank   *
     * External trigger: TIM1 TRGO = EXTSEL 0x09 on STM32G4,            *
     * rising edge trigger (EXTEN = 01).                                  *
     * CFGR: DMAEN=1, DMACFG=1 (circular DMA), single conversion.       */
    ADC1->SQR1  = (1u << ADC_SQR1_SQ1_Pos) | (0u << ADC_SQR1_L_Pos);
    ADC1->SMPR1 = (3u << (1u * 3u));

    ADC1->CFGR = ADC_CFGR_DMAEN
               | ADC_CFGR_DMACFG           /* circular */
               | (9u  << ADC_CFGR_EXTSEL_Pos)  /* TIM1 TRGO */
               | (1u  << ADC_CFGR_EXTEN_Pos);  /* rising edge */

    /* Arm — hardware trigger will fire the first conversion */
    ADC1->CR |= ADC_CR_ADSTART;
}

/* ------------------------------------------------------------------ */
/* ADC_ConvertCurrents                                                 *
 * Called from TIM1 ISR. Reads the last DMA-transferred CDR word and  *
 * converts to Amperes. No waiting — result is always ready because   *
 * DMA completes well before the next ISR fires (47.5+12.5 = 60 ADC  *
 * cycles @ 170 MHz = ~353 ns, ISR period = 50 µs).                  *
 * ------------------------------------------------------------------ */
void ADC_ConvertCurrents(void)
{
    uint32_t cdr = s_dma_cdr;
    const float lsb = ADC_VREF_V / ADC_FULL_SCALE;

    uint16_t rawSoB = (uint16_t)(cdr         & 0xFFFFu);  /* CDR[15:0]  = ADC1 master -> SOB */
    uint16_t rawSoA = (uint16_t)((cdr >> 16) & 0xFFFFu); /* CDR[31:16] = ADC2 slave  -> SOA */

    g_adcSoA = ((float)rawSoA * lsb - ADC_CSA_VBIAS - g_adcSoA_offset) * ADC_CSA_SCALE;
    g_adcSoB = ((float)rawSoB * lsb - ADC_CSA_VBIAS - g_adcSoB_offset) * ADC_CSA_SCALE;
}

/* ------------------------------------------------------------------ */
/* ADC_Sample                                                          *
 * Called from main loop background. Updates Vbus and temperatures.   *
 * Current sense (SOA/SOB) is handled by ADC_ConvertCurrents() in    *
 * the ISR — do not re-sample them here.                              *
 *                                                                    *
 * ADC2 ambient temp: temporarily reconfigure ADC2 for a sw-triggered *
 * single conversion on ch14. ADC1 hardware trigger is still armed;  *
 * if TIM1 TRGO fires during this window ADC2 won't respond (it's    *
 * busy with the sw conversion) but ADC1 will miss that one sample — *
 * acceptable given this runs at 50 Hz with a 50 µs PWM period.      *
 * ------------------------------------------------------------------ */
void ADC_Sample(void)
{
    uint16_t raw;
    const float lsb = ADC_VREF_V / ADC_FULL_SCALE;

    /* VSENVM (DC bus): ADC3 ch1 (PB1) — unaffected, always sw-triggered */
    raw = adc_read_channel(ADC3, 1u);
    g_adcVbus = (float)raw * lsb * ADC_VBUS_SCALE;

    float vadc, rntc;

    /* Ambient temp: reconfigure ADC2 temporarily for sw-triggered ch14 */
    ADC2->CR &= ~ADC_CR_ADSTART;  /* abort any pending hw trigger */
    raw = adc_read_channel(ADC2, 14u);
    /* Restore ADC2 to dual-simultaneous slave config (ch2, no CFGR change
     * needed — slave trigger is controlled by ADC1 master) */
    ADC2->SQR1  = (2u << ADC_SQR1_SQ1_Pos) | (0u << ADC_SQR1_L_Pos);
    ADC2->SMPR1 = (3u << (2u * 3u));

    vadc = (float)raw * lsb;
    rntc = ADC_NTC_RPULLUP * vadc / (ADC_VREF_V - vadc);
    g_adcAmbTemp = 1.0f / (logf(rntc / ADC_NTC_R25) / ADC_NTC_B + 1.0f / ADC_NTC_T25) - 273.15f;

    /* Phase temperature: ADC4 ch3 (PB12) */
    raw = adc_read_channel(ADC4, 3u);
    vadc = (float)raw * lsb;
    rntc = ADC_NTC_RPULLUP * vadc / (ADC_VREF_V - vadc);
    g_adcPhaseTemp = 1.0f / (logf(rntc / ADC_NTC_R25) / ADC_NTC_B + 1.0f / ADC_NTC_T25) - 273.15f;
}

/* ------------------------------------------------------------------ */
/* ADC_Sample_Raw — used only during DRV835x CSA offset calibration   *
 * Temporarily suspends the hw-triggered dual mode, takes CAL_SAMPLES *
 * sw-triggered readings of SOA and SOB, then re-arms the hw trigger. *
 * Called before the ISR is enabled so there is no race.              *
 * ------------------------------------------------------------------ */
void ADC_Sample_Raw(float *vsoA, float *vsoB)
{
    const float lsb = ADC_VREF_V / ADC_FULL_SCALE;

    /* ADC1 master: sw-triggered single on ch1 (SOB) */
    ADC1->CFGR &= ~(ADC_CFGR_EXTEN_Msk);   /* disable hw trigger */
    *vsoA += (float)adc_read_channel(ADC2, 2u) * lsb;
    *vsoB += (float)adc_read_channel(ADC1, 1u) * lsb;
}

/* Called once after calibration loop completes to re-arm hw trigger */
void ADC_RearmHwTrigger(void)
{
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->CFGR = (ADC1->CFGR & ~ADC_CFGR_EXTEN_Msk)
               | (1u << ADC_CFGR_EXTEN_Pos);   /* rising edge */
    ADC1->CR |= ADC_CR_ADSTART;
}
