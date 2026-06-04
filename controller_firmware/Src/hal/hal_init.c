/*
 * hal_init.c — Clock, GPIO, and TIM1 initialization for STM32G474RE
 *
 * Clock tree: HSE 24 MHz -> PLL (M=4, N=85, R=2) -> SYSCLK = 170 MHz
 *
 * TIM1 (Advanced timer) — 3-phase center-aligned PWM at 20 kHz
 *   ARR  = 4250  (170e6 / (2 * 20e3) = 4250)
 *   Mode = Center-aligned 2 (interrupt on up-count/overflow only, once per period)
 *   All three channel pairs (CH1/CH1N, CH2/CH2N, CH3/CH3N) synchronized
 *
 * Deadtime: DTG = 17 ticks @ 170 MHz -> ~100 ns
 *
 * Pin mapping (from peripheral_setup.txt):
 *   PA8  -> TIM1_CH1
 *   PA9  -> TIM1_CH2
 *   PA10 -> TIM1_CH3
 *   PB13 -> TIM1_CH1N
 *   PB14 -> TIM1_CH2N
 *   PB15 -> TIM1_CH3N
 *
 *   PA4  -> ENABLE (GPIO output, drives gate driver enable)
 */

#include "hal/hal_init.h"
#include "stm32g474xx.h"

/* ------------------------------------------------------------------ */
/* SystemClock_Config                                                  */
/* HSE 24 MHz -> PLLM=4 -> 6 MHz VCO in -> PLLN=85 -> 510 MHz        */
/* PLLR=3 -> SYSCLK = 170 MHz                                         */
/* ------------------------------------------------------------------ */
void SystemClock_Config(void)
{
    /* Enable power control clock, boost VOS to Range 1 (required at 170 MHz) */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;  /* pipeline flush */
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS_Msk) | (0x1u << PWR_CR1_VOS_Pos);

    /* Enable HSE oscillator */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* Configure Flash latency for 170 MHz: 8 wait states, prefetch on */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
               | (8u << FLASH_ACR_LATENCY_Pos)
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (8u << FLASH_ACR_LATENCY_Pos));

    /* Configure PLL: src=HSE, M=4, N=85, R=3 -> 170 MHz
     *   VCO_in  = 24 / 4 = 6 MHz
     *   VCO_out = 6 * 85  = 510 MHz
     *   PLLR    = 510 / 3 = 170 MHz
     */
    RCC->PLLCFGR = (3u  << RCC_PLLCFGR_PLLSRC_Pos)   /* HSE */
                 | ((4u - 1u) << RCC_PLLCFGR_PLLM_Pos)
                 | (85u << RCC_PLLCFGR_PLLN_Pos)
                 | ((3u - 1u) << RCC_PLLCFGR_PLLR_Pos)
                 | RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* AHB/APB prescalers: all /1 */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk))
              | (0u << RCC_CFGR_HPRE_Pos)   /* AHB  /1 */
              | (0u << RCC_CFGR_PPRE1_Pos)  /* APB1 /1 */
              | (0u << RCC_CFGR_PPRE2_Pos); /* APB2 /1 */

    /* Switch system clock to PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (3u << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (3u << RCC_CFGR_SWS_Pos));

    /* Update CMSIS SystemCoreClock variable */
    SystemCoreClock = 170000000u;
}

/* ------------------------------------------------------------------ */
/* GPIO_Init                                                           */
/* ------------------------------------------------------------------ */
void GPIO_Init(void)
{
    /* Enable GPIO clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN
                  | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;

    /* --- PA8 (CH1), PA9 (CH2), PA10 (CH3) -> AF6 (TIM1) --- */
    /* Mode: alternate function (10) */
    GPIOA->MODER = (GPIOA->MODER
        & ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk))
        | (2u << GPIO_MODER_MODE8_Pos)
        | (2u << GPIO_MODER_MODE9_Pos)
        | (2u << GPIO_MODER_MODE10_Pos);
    /* Speed: very high */
    GPIOA->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED8_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED9_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED10_Pos);
    /* AF6 on pins 8-10 -> AFRH (pins 8-15) */
    GPIOA->AFR[1] = (GPIOA->AFR[1]
        & ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL10_Msk))
        | (6u << GPIO_AFRH_AFSEL8_Pos)
        | (6u << GPIO_AFRH_AFSEL9_Pos)
        | (6u << GPIO_AFRH_AFSEL10_Pos);

    /* --- PB13 (CH1N), PB14 (CH2N), PB15 (CH3N) -> TIM1, AF varies per pin ---
     * AF numbers from STM32G474 datasheet Table 13:
     *   PB13 -> AF6  (TIM1_CH1N)
     *   PB14 -> AF12 (TIM1_CH2N)
     *   PB15 -> AF4  (TIM1_CH3N)
     * Confirmed: NUCLEO-G474RE TIM complementary example (STM32Cube_FW_G4_V1.6.2)
     */
    GPIOB->MODER = (GPIOB->MODER
        & ~(GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk))
        | (2u << GPIO_MODER_MODE13_Pos)
        | (2u << GPIO_MODER_MODE14_Pos)
        | (2u << GPIO_MODER_MODE15_Pos);
    GPIOB->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED13_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED14_Pos)
                    | (3u << GPIO_OSPEEDR_OSPEED15_Pos);
    GPIOB->AFR[1] = (GPIOB->AFR[1]
        & ~(GPIO_AFRH_AFSEL13_Msk | GPIO_AFRH_AFSEL14_Msk | GPIO_AFRH_AFSEL15_Msk))
        | (6u  << GPIO_AFRH_AFSEL13_Pos)   /* AF6  — TIM1_CH1N */
        | (6u  << GPIO_AFRH_AFSEL14_Pos)   /* AF6  — TIM1_CH2N */
        | (4u  << GPIO_AFRH_AFSEL15_Pos);  /* AF4  — TIM1_CH3N */

    /* --- PA4 (ENABLE) -> GPIO output, start LOW (gate driver disabled) --- */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE4_Msk)
                 | (1u << GPIO_MODER_MODE4_Pos);
    GPIOA->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED4_Pos);
    GPIOA->BSRR = GPIO_BSRR_BR4;  /* start disabled */

    /* PA5 is SPI1_SCK — configured in SPI_Init, not used as LED */

    /* --- PB0 (nFAULT from DRV835x, active-low) -> input with pull-up --- */
    GPIOB->MODER  &= ~GPIO_MODER_MODE0_Msk;              /* input mode (00) */
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD0_Msk)
                   | (1u << GPIO_PUPDR_PUPD0_Pos);        /* pull-up */
}

/* ------------------------------------------------------------------ */
/* TIM1_Init                                                           */
/*                                                                     */
/* Center-aligned mode 1, 20 kHz, with complementary outputs and      */
/* ~100 ns deadtime on all 3 channel pairs.                            */
/*                                                                     */
/* All channels share one ARR (4250) so duty updates are always        */
/* synchronized — no phase shift between phases.                       */
/*                                                                     */
/* CCR shadow registers update at counter underflow (center-aligned    */
/* mode 1, CMS=01: interrupt on down-count). This ensures the new      */
/* duty value for all three channels takes effect simultaneously at    */
/* the next period start.                                              */
/*                                                                     */
/* Deadtime register (DTG):                                            */
/*   At SYSCLK=170 MHz with CKD=00 (tDTS = 1/170e6 ≈ 5.88 ns),       */
/*   DTG = 17 -> DT = 17 * 5.88 ns ≈ 100 ns.                         */
/*   This is encoded as DTG[7:0] = 17 (value < 128, direct mapping).  */
/* ------------------------------------------------------------------ */
void TIM1_Init(void)
{
    /* Enable TIM1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    TIM1->CR1 = 0u;

    /* Center-aligned mode 2 (CMS=10): interrupt fires at top (overflow) only,
     * giving exactly one update event per PWM period (20 kHz). Mode 1 fires
     * at both top and bottom, doubling the effective ISR rate to 40 kHz. */
    TIM1->CR1 = (2u << TIM_CR1_CMS_Pos);

    /* Master mode: update event drives TRGO (useful for ADC triggering later) */
    TIM1->CR2 = (2u << TIM_CR2_MMS_Pos);

    /* No prescaler — timer clock = SYSCLK = 170 MHz */
    TIM1->PSC = 0u;

    /* ARR = 170e6 / (2 * 20e3) - 1 = 4249
     * Half-period = 4250 counts = 25 µs, full period = 50 µs = 20 kHz */
    TIM1->ARR = 4249u;

    /* Start all CCRs at 50% (neutral) */
    TIM1->CCR1 = 4249u / 2u;
    TIM1->CCR2 = 4249u / 2u;
    TIM1->CCR3 = 4249u / 2u;

    /* CCMR1: CH1 and CH2 in PWM mode 1, preload (shadow) enabled */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /* CCMR2: CH3 in PWM mode 1, preload (shadow) enabled */
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    /* CCER: enable CH1/CH2/CH3 and their complements, active high */
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
               | TIM_CCER_CC2E | TIM_CCER_CC2NE
               | TIM_CCER_CC3E | TIM_CCER_CC3NE;

    /* BDTR: deadtime = 17 ticks (~100 ns), MOE must be set to enable outputs.
     * MOE is set later by PWM_EnableOutput() so outputs stay off at startup. */
    TIM1->BDTR = (17u << TIM_BDTR_DTG_Pos)
               | TIM_BDTR_OSSR   /* off-state: drive inactive level when enabled */
               | TIM_BDTR_OSSI;  /* off-state: drive inactive level when disabled */

    /* Enable ARR preload */
    TIM1->CR1 |= TIM_CR1_ARPE;

    /* Generate an update event to load shadow registers, then clear flags */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0u;

    /* --- TIM1 update ISR (fires at each underflow, i.e. every 50 µs) --- */
    TIM1->DIER = TIM_DIER_UIE;

    /* Configure and enable TIM1_UP_TIM16 interrupt in NVIC at priority 0 */
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0u);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    /* Start the counter */
    TIM1->CR1 |= TIM_CR1_CEN;
}
