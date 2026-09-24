#include "flash.h"
#include "stm32g4xx.h"
#include <string.h>

volatile int flash_reading;

/* last flash page: bank 2 page 127 (dual bank) or page 127 (single bank) */
static uint32_t page_addr(void)
{
    return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x0807F800u : 0x0807F000u;
}

int flash_read(void *d, size_t n)
{
    flash_reading = 1;
    memcpy(d, (const void *)page_addr(), n);
    flash_reading = 0;
    return 1;
}

int flash_write(const void *d, size_t n)
{
    uint32_t a = page_addr();
    const uint32_t *w = d;

    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->SR = FLASH->SR;
    FLASH->CR = FLASH_CR_PER | (127u << FLASH_CR_PNB_Pos) |
                ((FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_CR_BKER : 0);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR = FLASH_CR_PG;
    for (size_t i = 0; i < n / 4; i += 2) {
        *(volatile uint32_t *)(a + i * 4) = w[i];
        *(volatile uint32_t *)(a + i * 4 + 4) = w[i + 1];
        while (FLASH->SR & FLASH_SR_BSY) {}
    }
    uint32_t err = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                                FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR);
    FLASH->CR = FLASH_CR_LOCK;
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
    return !err && memcmp((const void *)a, d, n) == 0;
}
