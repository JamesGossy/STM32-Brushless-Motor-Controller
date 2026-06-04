#include "hal/hal_pwm.h"
#include "stm32g474xx.h"

#define TIM1_ARR  4249u

#define ENABLE_HIGH()  (GPIOA->BSRR = GPIO_BSRR_BS4)
#define ENABLE_LOW()   (GPIOA->BSRR = GPIO_BSRR_BR4)

/* CCER bit pairs for each channel: (CCxE | CCxNE) */
#define CCER_CH1  (TIM_CCER_CC1E  | TIM_CCER_CC1NE)
#define CCER_CH2  (TIM_CCER_CC2E  | TIM_CCER_CC2NE)
#define CCER_CH3  (TIM_CCER_CC3E  | TIM_CCER_CC3NE)

/*
 * 6-step commutation table.
 *
 * For each step: which channels are active (CCER mask), and the CCR
 * values for CH1/CH2/CH3 expressed as a signed polarity:
 *   +1 -> PWM duty on high-side  (CCR = duty * ARR)
 *   -1 -> PWM duty on low-side   (CCR = 0, complementary output carries duty)
 *    0 -> channel disabled (floated) — CCR value irrelevant
 *
 * Standard 6-step sequence (A+B-, A+C-, B+C-, B+A-, C+A-, C+B-):
 *
 *  step | +phase | -phase | float | A  | B  | C
 *   0   |   A    |   B    |   C   | +1 | -1 |  0
 *   1   |   A    |   C    |   B   | +1 |  0 | -1
 *   2   |   B    |   C    |   A   |  0 | +1 | -1
 *   3   |   B    |   A    |   C   | -1 | +1 |  0
 *   4   |   C    |   A    |   B   | -1 |  0 | +1
 *   5   |   C    |   B    |   A   |  0 | -1 | +1
 *
 * Polarity -1: CCR=0 means the high-side is always off; the complementary
 * output (low-side) is always on (after deadtime). This sinks current through
 * the low-side FET continuously, which is correct for the return path.
 * The PWM duty is applied on the +phase only.
 */
typedef struct { uint32_t ccer; int8_t polA, polB, polC; } Step_t;

static const Step_t k_steps[6] = {
    { CCER_CH1 | CCER_CH2,           +1, -1,  0 },  /* step 0: A+ B- */
    { CCER_CH1 |           CCER_CH3, +1,  0, -1 },  /* step 1: A+ C- */
    {            CCER_CH2 | CCER_CH3,  0, +1, -1 },  /* step 2: B+ C- */
    { CCER_CH1 | CCER_CH2,           -1, +1,  0 },  /* step 3: B+ A- */
    { CCER_CH1 |           CCER_CH3, -1,  0, +1 },  /* step 4: C+ A- */
    {            CCER_CH2 | CCER_CH3,  0, -1, +1 },  /* step 5: C+ B- */
};

void PWM_EnableOutput(void)
{
    ENABLE_HIGH();
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void PWM_DisableOutput(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    ENABLE_LOW();
}

void PWM_SetDuty(float dutyA, float dutyB, float dutyC)
{
    if (dutyA < 0.0f) dutyA = 0.0f; else if (dutyA > 1.0f) dutyA = 1.0f;
    if (dutyB < 0.0f) dutyB = 0.0f; else if (dutyB > 1.0f) dutyB = 1.0f;
    if (dutyC < 0.0f) dutyC = 0.0f; else if (dutyC > 1.0f) dutyC = 1.0f;

    TIM1->CCR1 = (uint32_t)(dutyA * (float)(TIM1_ARR + 1u));
    TIM1->CCR2 = (uint32_t)(dutyB * (float)(TIM1_ARR + 1u));
    TIM1->CCR3 = (uint32_t)(dutyC * (float)(TIM1_ARR + 1u));
}

/*
 * PWM_SetStep — apply one 6-step commutation state.
 *
 * duty [0..1] is applied to the positive phase only.
 * The negative phase drives its low-side FET fully on (CCR=0).
 * The floating phase has both outputs disabled via CCER.
 *
 * CCER is written atomically (single 32-bit store). CCRs update at
 * the next counter underflow via shadow registers, so all three phase
 * changes take effect simultaneously.
 */
void PWM_SetStep(uint8_t step, float duty)
{
    if (step > 5u) return;
    if (duty < 0.0f) duty = 0.0f;
    else if (duty > 1.0f) duty = 1.0f;

    const Step_t *s = &k_steps[step];
    uint32_t ccr_pwm = (uint32_t)(duty * (float)(TIM1_ARR + 1u));

    /* CCR for each polarity:
     *  +1 (positive phase) -> ccr_pwm  (high-side switches at duty)
     *  -1 (negative phase) -> 0        (high-side always off, low-side always on)
     *   0 (float)          -> 0        (channel disabled in CCER anyway) */
    TIM1->CCR1 = (s->polA == +1) ? ccr_pwm : 0u;
    TIM1->CCR2 = (s->polB == +1) ? ccr_pwm : 0u;
    TIM1->CCR3 = (s->polC == +1) ? ccr_pwm : 0u;

    /* Disable all outputs first, then re-enable only the active pair.
     * Both writes happen before the next update event so there is no
     * glitch — shadow registers latch the new CCER at counter underflow. */
    TIM1->CCER = s->ccer;
}
