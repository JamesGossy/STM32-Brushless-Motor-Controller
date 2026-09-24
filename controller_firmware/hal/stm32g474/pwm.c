#include "pwm.h"
#include "board.h"

static const uint8_t timers[3] = {0, 4, 5};   /* HRTIM A, E, F -> phase A, B, C */

void pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;
    (void)RCC->APB2ENR;

    HRTIM1->sCommonRegs.DLLCR = HRTIM_DLLCR_CAL;
    while (!(HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY)) {}
    HRTIM1->sCommonRegs.DLLCR = HRTIM_DLLCR_CALRTE_0 | HRTIM_DLLCR_CALRTE_1 | HRTIM_DLLCR_CALEN;

    uint32_t dt = (uint32_t)(DEADTIME_NS * (F_SYS / 1e9f) + 0.5f);
    for (int i = 0; i < 3; i++) {
        HRTIM_Timerx_TypeDef *t = &HRTIM1->sTimerxRegs[timers[i]];
        t->TIMxCR = (3u << HRTIM_TIMCR_CK_PSC_Pos) | HRTIM_TIMCR_CONT | HRTIM_TIMCR_TRSTU;
        t->PERxR = PWM_PER;
        t->CMP1xR = PWM_PER / 4;
        t->CMP2xR = PWM_PER * 3 / 4;
        /* output 1 = low side: on around counter wrap, off between CMP1..CMP2 */
        t->SETx1R = HRTIM_SET1R_CMP2;
        t->RSTx1R = HRTIM_RST1R_CMP1;
        t->DTxR = (3u << HRTIM_DTR_DTPRSC_Pos) | (dt << HRTIM_DTR_DTR_Pos) | (dt << HRTIM_DTR_DTF_Pos);
        t->OUTxR = HRTIM_OUTR_DTEN;
        t->TIMxCR |= HRTIM_TIMCR_PREEN;
    }

    HRTIM1->sCommonRegs.CR1 = 1u << HRTIM_CR1_ADC2USRC_Pos;
    HRTIM1->sCommonRegs.ADC2R = HRTIM_ADC2R_AD2TAPER;   /* ADC trigger at centre of low-side on time */

    pin_mode(GPIOA, 8, PIN_AF, 13);  /* TA1 INLA */
    pin_mode(GPIOA, 9, PIN_AF, 13);  /* TA2 INHA */
    pin_mode(GPIOC, 8, PIN_AF, 3);   /* TE1 INLB */
    pin_mode(GPIOC, 9, PIN_AF, 3);   /* TE2 INHB */
    pin_mode(GPIOC, 6, PIN_AF, 13);  /* TF1 INLC */
    pin_mode(GPIOC, 7, PIN_AF, 13);  /* TF2 INHC */
}

void pwm_start(void)
{
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN | HRTIM_MCR_TECEN | HRTIM_MCR_TFCEN;
}

void pwm_on(void)
{
    HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN | HRTIM_OENR_TE1OEN |
                               HRTIM_OENR_TE2OEN | HRTIM_OENR_TF1OEN | HRTIM_OENR_TF2OEN;
}

void pwm_off(void)
{
    HRTIM1->sCommonRegs.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS | HRTIM_ODISR_TE1ODIS |
                                HRTIM_ODISR_TE2ODIS | HRTIM_ODISR_TF1ODIS | HRTIM_ODISR_TF2ODIS;
}
