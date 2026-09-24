#include "usb_cdc.h"
#include "usb_ll.h"
#include "usb_desc.h"
#include "system.h"

static uint8_t line_coding[7] = {0x00, 0x10, 0x0E, 0x00, 0, 0, 8};   /* 921600 8N1 */
static uint8_t cfg_val, pend_addr, expect_lc;
static const uint8_t *ep0_ptr;
static uint16_t ep0_left;
static uint8_t ep0_zlp;

#define TXBUF 2048u
static uint8_t txbuf[TXBUF];
static volatile uint32_t tx_head, tx_tail;
static volatile uint8_t ep1_busy;

#define RXBUF 256u
static uint8_t rxbuf[RXBUF];
static volatile uint32_t rx_head, rx_tail;

static void ep0_next(void)
{
    uint16_t n = ep0_left > 64 ? 64 : ep0_left;
    pma_write(EP0_TX, ep0_ptr, n);
    PMA(2) = n;
    ep0_ptr += n;
    ep0_left -= n;
    ep_tx_stat(0, USB_EP_TX_VALID);
}

static void ep0_send(const uint8_t *p, uint16_t len, uint16_t wlen)
{
    if (len > wlen) len = wlen;
    ep0_ptr = p;
    ep0_left = len;
    ep0_zlp = len && len < wlen && (len % 64) == 0;
    ep0_next();
}

static void ep0_ack(void) { ep0_send(0, 0, 0); }

static void setup(void)
{
    static const uint8_t zero[2];
    uint8_t s[8];
    pma_read(EP0_RX, s, 8);
    uint16_t val = s[2] | (s[3] << 8), len = s[6] | (s[7] << 8);
    expect_lc = 0;

    if ((s[0] & 0x60) == 0x00) {
        switch (s[1]) {
        case 0x06: {
            const uint8_t *p;
            int n = usb_get_descriptor(val, &p);
            if (n) { ep0_send(p, (uint16_t)n, len); return; }
            break;
        }
        case 0x05: pend_addr = val & 0x7F; ep0_ack(); return;
        case 0x09:
            cfg_val = (uint8_t)val;
            if (cfg_val) {
                ep_init(1, USB_EP_BULK, USB_EP_RX_VALID, USB_EP_TX_NAK);
                ep_init(2, USB_EP_INTERRUPT, 0, USB_EP_TX_NAK);
                ep1_busy = 0;
            }
            ep0_ack();
            return;
        case 0x08: ep0_send(&cfg_val, 1, len); return;
        case 0x0A: ep0_send(zero, 1, len); return;
        case 0x00: ep0_send(zero, 2, len); return;
        case 0x01: case 0x03: case 0x0B: ep0_ack(); return;
        }
    } else if ((s[0] & 0x60) == 0x20) {
        switch (s[1]) {
        case 0x20: expect_lc = 1; return;
        case 0x21: ep0_send(line_coding, 7, len); return;
        case 0x22: case 0x23: ep0_ack(); return;
        }
    }
    ep_tx_stat(0, USB_EP_TX_STALL);
}

static void tx_kick(void)
{
    if (!cfg_val || ep1_busy) return;
    uint32_t h = tx_head, t = tx_tail;
    if (h == t) return;
    uint8_t pkt[64];
    uint16_t n = 0;
    while (t != h && n < 64) { pkt[n++] = txbuf[t]; t = (t + 1) % TXBUF; }
    pma_write(EP1_TX, pkt, n);
    PMA(8 + 2) = n;
    tx_tail = t;
    ep1_busy = 1;
    ep_tx_stat(1, USB_EP_TX_VALID);
}

static void usb_reset(void)
{
    USB->BTABLE = 0;
    PMA(0) = EP0_TX;  PMA(2) = 0;  PMA(4) = EP0_RX;  PMA(6) = RX64;
    PMA(8) = EP1_TX;  PMA(10) = 0; PMA(12) = EP1_RX; PMA(14) = RX64;
    PMA(16) = EP2_TX; PMA(18) = 0; PMA(20) = 0;      PMA(22) = 0;
    ep_init(0, USB_EP_CONTROL, USB_EP_RX_VALID, USB_EP_TX_NAK);
    USB->DADDR = USB_DADDR_EF;
    cfg_val = pend_addr = expect_lc = 0;
    ep1_busy = 0;
    tx_tail = tx_head;
}

void USB_LP_IRQHandler(void)
{
    if (USB->ISTR & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        usb_reset();
        return;
    }
    uint16_t istr;
    while ((istr = USB->ISTR) & USB_ISTR_CTR) {
        int ep = istr & USB_ISTR_EP_ID;
        uint16_t v = EPR(ep);
        if (ep == 0) {
            if (v & USB_EP_CTR_TX) {
                ep_clr_tx(0);
                if (pend_addr) { USB->DADDR = USB_DADDR_EF | pend_addr; pend_addr = 0; }
                if (ep0_left) ep0_next();
                else if (ep0_zlp) { ep0_zlp = 0; ep0_next(); }
            }
            if (v & USB_EP_CTR_RX) {
                if (v & USB_EP_SETUP) {
                    setup();
                } else if (expect_lc) {
                    uint16_t n = PMA(6) & 0x3FF;
                    pma_read(EP0_RX, line_coding, n < 7 ? n : 7);
                    expect_lc = 0;
                    ep0_ack();
                }
                ep_clr_rx(0);
                ep_rx_stat(0, USB_EP_RX_VALID);
            }
        } else if (ep == 1) {
            if (v & USB_EP_CTR_TX) { ep_clr_tx(1); ep1_busy = 0; }
            if (v & USB_EP_CTR_RX) {
                uint8_t pkt[64];
                uint16_t n = PMA(8 + 6) & 0x3FF;
                if (n > 64) n = 64;
                pma_read(EP1_RX, pkt, n);
                for (uint16_t i = 0; i < n; i++) {
                    uint32_t h = (rx_head + 1) % RXBUF;
                    if (h == rx_tail) break;
                    rxbuf[rx_head] = pkt[i];
                    rx_head = h;
                }
                ep_clr_rx(1);
                ep_rx_stat(1, USB_EP_RX_VALID);
            }
        } else {
            if (v & USB_EP_CTR_TX) ep_clr_tx(ep);
            if (v & USB_EP_CTR_RX) ep_clr_rx(ep);
        }
    }
    tx_kick();
}

void usb_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;
    (void)RCC->APB1ENR1;
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;   /* trim HSI48 to USB SOF */

    USB->CNTR = USB_CNTR_FRES;
    delay_us(10);
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
    USB->BCDR |= USB_BCDR_DPPU;

    NVIC_SetPriority(USB_LP_IRQn, 3);
    NVIC_EnableIRQ(USB_LP_IRQn);
}

int usb_write(const void *data, uint32_t len)
{
    if (!cfg_val) return 0;
    const uint8_t *d = data;
    uint32_t h = tx_head, t = tx_tail;
    if (len > (t + TXBUF - h - 1) % TXBUF) return 0;
    for (uint32_t i = 0; i < len; i++) { txbuf[h] = d[i]; h = (h + 1) % TXBUF; }
    __DMB();
    tx_head = h;
    NVIC_SetPendingIRQ(USB_LP_IRQn);
    return 1;
}

size_t usb_read(uint8_t *d, size_t max)
{
    size_t n = 0;
    while (n < max && rx_tail != rx_head) {
        d[n++] = rxbuf[rx_tail];
        rx_tail = (rx_tail + 1) % RXBUF;
    }
    return n;
}
