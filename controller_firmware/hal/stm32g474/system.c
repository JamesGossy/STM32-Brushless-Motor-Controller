/*
 * system.c - clock tree, SysTick, delays, watchdog and fault handlers.
 *
 * Clocks: 24 MHz crystal -> PLL -> 170 MHz SYSCLK, AHB, APB1 and APB2.
 * FDCAN runs straight from the crystal, USB from HSI48 trimmed by CRS.
 */
#include "system.h"
#include "board.h"
#include "config.h"
#include "pwm.h"
#include "flash.h"

volatile uint32_t ms_ticks;
uint32_t SystemCoreClock = F_SYS;

/* Called by the startup code before main(): enable the FPU. */
void SystemInit(void)
{
    SCB->CPACR |= (0xFu << 20);
    SCB->VTOR = FLASH_BASE;
}

void SysTick_Handler(void) { ms_ticks++; }

/* On any crash: power stage off and gate driver disabled. */
static void safe_stop(void)
{
    pwm_off();
    GPIOB->BRR = 1u << ENABLE_PIN;
}

/* A flash ECC error while reading the settings page just means the page is
   corrupt; return so cfg_load() falls back to defaults. Anything else stops. */
void NMI_Handler(void)
{
    if (flash_reading && (FLASH->ECCR & FLASH_ECCR_ECCD)) {
        FLASH->ECCR |= FLASH_ECCR_ECCD;
        return;
    }
    safe_stop();
    for (;;) {}
}

void HardFault_Handler(void)  { safe_stop(); for (;;) {} }
void MemManage_Handler(void)  { safe_stop(); for (;;) {} }
void BusFault_Handler(void)   { safe_stop(); for (;;) {} }
void UsageFault_Handler(void) { safe_stop(); for (;;) {} }

/* Switch to 170 MHz and start the 1 ms tick. */
void system_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    PWR->CR3 |= PWR_CR3_UCPD_DBDIS;          /* remove USB-C dead battery pull-downs on PB4/PB6 */
    PWR->CR5 &= ~PWR_CR5_R1MODE;             /* voltage range 1 boost, needed above 150 MHz */
    FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    /* 24 MHz / M6 * N85 / R2 = 170 MHz */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (5u << RCC_PLLCFGR_PLLM_Pos) |
                   (85u << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* the reference manual asks for AHB/2 for 1 us when jumping above 80 MHz */
    RCC->CFGR = RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    for (volatile int i = 0; i < 100; i++) {}
    RCC->CFGR = RCC_CFGR_SW_PLL;

    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {}

    /* kernel clocks: FDCAN = HSE, USB = HSI48, ADC = SYSCLK */
    RCC->CCIPR = (0u << RCC_CCIPR_FDCANSEL_Pos) | (0u << RCC_CCIPR_CLK48SEL_Pos) |
                 (2u << RCC_CCIPR_ADC12SEL_Pos) | (2u << RCC_CCIPR_ADC345SEL_Pos);

    SysTick_Config(F_SYS / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 2);
}

/* Busy wait, roughly calibrated for 170 MHz. */
void delay_us(uint32_t us)
{
    uint32_t n = us * (F_SYS / 4000000u);
    while (n--) __NOP();
}

void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while (ms_ticks - start < ms) {}
}

/* Independent watchdog, 100 ms timeout. Frozen while halted in the debugger. */
void wdg_init(void)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
    IWDG->KR = 0xCCCC;      /* start */
    IWDG->KR = 0x5555;      /* unlock registers */
    IWDG->PR = 2;           /* 32 kHz / 16 = 2 kHz */
    IWDG->RLR = 200;
    while (IWDG->SR) {}
    IWDG->KR = 0xAAAA;
}
