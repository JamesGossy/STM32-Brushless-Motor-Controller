#include "usb_desc.h"
#include "stm32g4xx.h"

/* CDC ACM, ST VCP VID/PID */
static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
    0x83, 0x04, 0x40, 0x57, 0x00, 0x02, 1, 2, 3, 1
};

static const uint8_t cfg_desc[67] = {
    9, 2, 67, 0, 2, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 0x02, 0x02, 0x00, 0,
    5, 0x24, 0x00, 0x10, 0x01,
    5, 0x24, 0x01, 0x00, 0x01,
    4, 0x24, 0x02, 0x02,
    5, 0x24, 0x06, 0x00, 0x01,
    7, 5, 0x82, 0x03, 8, 0, 16,
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, 0x01, 0x02, 64, 0, 0,
    7, 5, 0x81, 0x02, 64, 0, 0,
};

static const uint8_t lang_desc[4] = {4, 3, 0x09, 0x04};
static uint8_t str_buf[64];
static void string_desc(const char *s)
{
    uint8_t n = 0;
    while (s[n] && n < 30) { str_buf[2 + 2 * n] = (uint8_t)s[n]; str_buf[3 + 2 * n] = 0; n++; }
    str_buf[0] = 2 + 2 * n;
    str_buf[1] = 3;
}

int usb_get_descriptor(uint16_t val, const uint8_t **p)
{
    static char serial[9];
    switch (val >> 8) {
    case 1: *p = dev_desc; return sizeof dev_desc;
    case 2: *p = cfg_desc; return sizeof cfg_desc;
    case 3:
        switch (val & 0xFF) {
        case 0: *p = lang_desc; return sizeof lang_desc;
        case 1: string_desc("STM32G474 ESC"); break;
        case 2: string_desc("FOC Controller"); break;
        case 3: {
            uint32_t u = ((uint32_t *)UID_BASE)[0] ^ ((uint32_t *)UID_BASE)[1] ^ ((uint32_t *)UID_BASE)[2];
            for (int i = 0; i < 8; i++) serial[i] = "0123456789ABCDEF"[(u >> (28 - 4 * i)) & 0xF];
            string_desc(serial);
            break;
        }
        default: return 0;
        }
        *p = str_buf;
        return str_buf[0];
    }
    return 0;
}
