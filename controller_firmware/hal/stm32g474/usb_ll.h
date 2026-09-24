#pragma once
#include <stdint.h>
#include "stm32g4xx.h"

#define EPR(n)  (*(volatile uint16_t *)(USB_BASE + 4u * (n)))
#define PMA(o)  (*(volatile uint16_t *)(USB_PMAADDR + (o)))

/* packet memory layout, BTABLE at 0 */
#define EP0_TX  0x40
#define EP0_RX  0x80
#define EP1_TX  0xC0
#define EP1_RX  0x100
#define EP2_TX  0x140
#define RX64    0x8400   /* BL_SIZE=1, NUM_BLOCK=1 -> 64 bytes */

static inline void pma_write(uint16_t off, const uint8_t *s, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t v = s[i];
        if (i + 1 < n) v |= (uint16_t)s[i + 1] << 8;
        PMA(off + i) = v;
    }
}

static inline void pma_read(uint16_t off, uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t v = PMA(off + i);
        d[i] = (uint8_t)v;
        if (i + 1 < n) d[i + 1] = (uint8_t)(v >> 8);
    }
}

static inline void ep_tx_stat(int ep, uint16_t st)
{
    uint16_t v = EPR(ep) & (USB_EPREG_MASK | USB_EPTX_STAT);
    EPR(ep) = (v ^ st) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

static inline void ep_rx_stat(int ep, uint16_t st)
{
    uint16_t v = EPR(ep) & (USB_EPREG_MASK | USB_EPRX_STAT);
    EPR(ep) = (v ^ st) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

static inline void ep_clr_rx(int ep) { EPR(ep) = (EPR(ep) & USB_EPREG_MASK & ~USB_EP_CTR_RX) | USB_EP_CTR_TX; }
static inline void ep_clr_tx(int ep) { EPR(ep) = (EPR(ep) & USB_EPREG_MASK & ~USB_EP_CTR_TX) | USB_EP_CTR_RX; }

static inline void ep_init(int ep, uint16_t type, uint16_t rx, uint16_t tx)
{
    uint16_t v = EPR(ep);
    uint16_t w = type | (uint16_t)ep | USB_EP_CTR_RX | USB_EP_CTR_TX;
    w |= v & (USB_EP_DTOG_RX | USB_EP_DTOG_TX);
    w |= (v & (USB_EPRX_STAT | USB_EPTX_STAT)) ^ (rx | tx);
    EPR(ep) = w;
}
