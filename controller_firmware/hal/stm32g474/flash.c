/*
 * flash.c - erase and program the last flash page.
 *
 * The linker script leaves the last 4 KB of flash free for this. In the
 * default dual-bank mode the page is bank 2 page 127 (2 KB, and writing it
 * doesn't stall code running from bank 1); in single-bank mode it is page 127.
 */
#include "flash.h"
#include "stm32g4xx.h"
#include <string.h>

volatile int flash_reading;

/* Address of the settings page for the current bank mode. */
static uint32_t page_address(void)
{
    return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x0807F800u : 0x0807F000u;
}

int flash_read(void *data, size_t len)
{
    flash_reading = 1;
    memcpy(data, (const void *)page_address(), len);
    flash_reading = 0;
    return 1;
}

/* Erase the page and program `len` bytes. Returns 1 if the data verifies. */
int flash_write(const void *data, size_t len)
{
    uint32_t addr = page_address();
    const uint32_t *words = data;

    FLASH->KEYR = 0x45670123u;      /* unlock */
    FLASH->KEYR = 0xCDEF89ABu;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->SR = FLASH->SR;          /* clear old error flags */

    FLASH->CR = FLASH_CR_PER | (127u << FLASH_CR_PNB_Pos) |
                ((FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_CR_BKER : 0);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) {}

    /* program in 64-bit double words */
    FLASH->CR = FLASH_CR_PG;
    for (size_t i = 0; i < len / 4; i += 2) {
        *(volatile uint32_t *)(addr + i * 4) = words[i];
        *(volatile uint32_t *)(addr + i * 4 + 4) = words[i + 1];
        while (FLASH->SR & FLASH_SR_BSY) {}
    }

    uint32_t err = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                                FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR);
    FLASH->CR = FLASH_CR_LOCK;

    /* flush the data cache so the check below reads the new contents */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;

    return !err && memcmp((const void *)addr, data, len) == 0;
}
