/*
 * fdcan.c - FDCAN2 driver (PB5 RX, PB6 TX, TCAN1462 transceiver).
 *
 * The G4 message RAM has a fixed layout per instance: 0x350 bytes each,
 * with RX FIFO 0 at +0xB0 and the TX FIFO at +0x278, 72-byte elements.
 * Every standard frame goes into RX FIFO 0; the protocol layer filters by node.
 */
#include "fdcan.h"
#include "stm32g4xx.h"
#include <string.h>

#define MSG_RAM   (SRAMCAN_BASE + 0x350u)   /* FDCAN2 is the second instance */
#define RX_FIFO0  (MSG_RAM + 0x0B0u)
#define TX_FIFO   (MSG_RAM + 0x278u)
#define ELEMENT   72u

/* Configure for 1 Mbit/s and join the bus. */
void fdcan_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;

    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT)) {}
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    FDCAN2->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR);   /* classic CAN, auto retransmit */

    for (uint32_t *p = (uint32_t *)MSG_RAM; p < (uint32_t *)(MSG_RAM + 0x350u); p++) *p = 0;

    /* 24 MHz crystal, 24 time quanta per bit: sync 1 + seg1 19 + seg2 4 (83 % sample point) */
    FDCAN2->NBTP = (3u << FDCAN_NBTP_NSJW_Pos) | (0u << FDCAN_NBTP_NBRP_Pos) |
                   (18u << FDCAN_NBTP_NTSEG1_Pos) | (3u << FDCAN_NBTP_NTSEG2_Pos);
    FDCAN2->RXGFC = FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE;   /* reject remote frames, accept the rest */
    FDCAN2->TXBC = 0;                                       /* TX FIFO mode */

    FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN2->CCCR & FDCAN_CCCR_INIT) {}
}

/* Queue a frame. Returns 0 if the TX FIFO is full. */
int fdcan_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (FDCAN2->TXFQS & FDCAN_TXFQS_TFQF) return 0;

    uint32_t index = (FDCAN2->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 3u;
    volatile uint32_t *e = (volatile uint32_t *)(TX_FIFO + index * ELEMENT);
    uint32_t words[2] = {0, 0};
    memcpy(words, data, len > 8 ? 8 : len);

    e[0] = (id & 0x7FFu) << 18;     /* standard id */
    e[1] = (uint32_t)len << 16;     /* DLC */
    e[2] = words[0];
    e[3] = words[1];
    FDCAN2->TXBAR = 1u << index;
    return 1;
}

/* Take the next received standard frame. Returns 0 if there is none. */
int fdcan_recv(uint32_t *id, uint8_t *data, uint8_t *len)
{
    /* after bus-off the controller sets INIT; clearing it starts recovery */
    if (FDCAN2->CCCR & FDCAN_CCCR_INIT) FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;

    while (FDCAN2->RXF0S & FDCAN_RXF0S_F0FL_Msk) {
        uint32_t index = (FDCAN2->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 3u;
        volatile uint32_t *e = (volatile uint32_t *)(RX_FIFO0 + index * ELEMENT);
        uint32_t header = e[0], dlc = e[1];
        uint32_t words[2] = {e[2], e[3]};
        FDCAN2->RXF0A = index;

        if (header & (1u << 30)) continue;     /* skip extended ids */

        *id = (header >> 18) & 0x7FFu;
        *len = (uint8_t)((dlc >> 16) & 0xFu);
        if (*len > 8) *len = 8;
        memcpy(data, words, 8);
        return 1;
    }
    return 0;
}
