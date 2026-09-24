#include "fdcan.h"
#include "stm32g4xx.h"
#include <string.h>

#define RAM   (SRAMCAN_BASE + 0x350u)   /* FDCAN2 message RAM */
#define RXF0  (RAM + 0x0B0u)
#define TXFQ  (RAM + 0x278u)
#define ELEM  72u

void fdcan_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;

    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT)) {}
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    FDCAN2->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR);

    for (uint32_t *p = (uint32_t *)RAM; p < (uint32_t *)(RAM + 0x350u); p++) *p = 0;

    /* 24 MHz HSE, 24 tq/bit: 1 + 19 + 4, sample 83% -> 1 Mbit */
    FDCAN2->NBTP = (3u << FDCAN_NBTP_NSJW_Pos) | (0u << FDCAN_NBTP_NBRP_Pos) |
                   (18u << FDCAN_NBTP_NTSEG1_Pos) | (3u << FDCAN_NBTP_NTSEG2_Pos);
    FDCAN2->RXGFC = FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE;   /* all std frames -> FIFO0 */
    FDCAN2->TXBC = 0;

    FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN2->CCCR & FDCAN_CCCR_INIT) {}
}

int fdcan_send(uint32_t id, const uint8_t *d, uint8_t len)
{
    if (FDCAN2->TXFQS & FDCAN_TXFQS_TFQF) return 0;
    uint32_t pi = (FDCAN2->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 3u;
    volatile uint32_t *e = (volatile uint32_t *)(TXFQ + pi * ELEM);
    uint32_t w[2] = {0, 0};
    memcpy(w, d, len > 8 ? 8 : len);
    e[0] = (id & 0x7FFu) << 18;
    e[1] = (uint32_t)len << 16;
    e[2] = w[0];
    e[3] = w[1];
    FDCAN2->TXBAR = 1u << pi;
    return 1;
}

int fdcan_recv(uint32_t *id, uint8_t *d, uint8_t *len)
{
    if (FDCAN2->CCCR & FDCAN_CCCR_INIT) FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;   /* bus-off recovery */
    while (FDCAN2->RXF0S & FDCAN_RXF0S_F0FL_Msk) {
        uint32_t gi = (FDCAN2->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 3u;
        volatile uint32_t *e = (volatile uint32_t *)(RXF0 + gi * ELEM);
        uint32_t r0 = e[0], r1 = e[1];
        uint32_t w[2] = {e[2], e[3]};
        FDCAN2->RXF0A = gi;
        if (r0 & (1u << 30)) continue;             /* extended id */
        *id = (r0 >> 18) & 0x7FFu;
        *len = (uint8_t)((r1 >> 16) & 0xFu);
        if (*len > 8) *len = 8;
        memcpy(d, w, 8);
        return 1;
    }
    return 0;
}
