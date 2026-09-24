#include "telem.h"
#include "foc.h"
#include "svpwm.h"
#include "hal.h"
#include "config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RPM_PER_RADS 9.5492966f

int telem_enabled = 1;

uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t telem_frame(uint8_t type, const void *payload, uint16_t len, uint8_t *out)
{
    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = type;
    out[3] = (uint8_t)len;
    out[4] = (uint8_t)(len >> 8);
    memcpy(out + 5, payload, len);
    uint16_t crc = crc16_ccitt(out + 2, 3u + len);
    out[5 + len] = (uint8_t)crc;
    out[6 + len] = (uint8_t)(crc >> 8);
    return 7u + len;
}

void telem_send(uint8_t type, const void *payload, uint16_t len)
{
    uint8_t f[TELEM_MAX_PAYLOAD + 7];
    if (len > TELEM_MAX_PAYLOAD) len = TELEM_MAX_PAYLOAD;
    hal_serial_write(f, telem_frame(type, payload, len, f));
}

void telem_log(const char *fmt, ...)
{
    char s[TELEM_MAX_PAYLOAD];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof s) n = sizeof s - 1;
    telem_send(TELEM_LOG, s, (uint16_t)n);
}

static void put_f32(uint8_t **p, float v) { memcpy(*p, &v, 4); *p += 4; }

void telem_tick(uint32_t n)
{
    if (!telem_enabled) return;
    uint8_t b[32], *p;

    if (n % TELEM_MS == 0) {
        p = b;
        *p++ = foc.state; *p++ = foc.mode;
        uint16_t f = foc.faults;
        memcpy(p, &f, 2); p += 2;
        put_f32(&p, foc.iq); put_f32(&p, foc.id); put_f32(&p, foc.iq_ref); put_f32(&p, foc.id_ref);
        telem_send(TELEM_MOTOR, b, (uint16_t)(p - b));

        p = b;
        float ref = foc.mode == MODE_SPEED ? foc.speed_ref : foc.omega_m;
        put_f32(&p, foc.omega_m * RPM_PER_RADS); put_f32(&p, ref * RPM_PER_RADS); put_f32(&p, foc.theta_e);
        telem_send(TELEM_SPEED, b, (uint16_t)(p - b));

        p = b;
        put_f32(&p, foc.vbus); put_f32(&p, foc.vd); put_f32(&p, foc.vq);
        put_f32(&p, duty[0]); put_f32(&p, duty[1]); put_f32(&p, duty[2]);
        telem_send(TELEM_VOLT, b, (uint16_t)(p - b));

        p = b;
        put_f32(&p, foc.ia); put_f32(&p, foc.ib); put_f32(&p, foc.ic);
        telem_send(TELEM_PHASE, b, (uint16_t)(p - b));
    }
    if (n % 100 == 0) {
        int16_t t[2] = {(int16_t)(foc.t_fet * 100.0f), (int16_t)(foc.t_amb * 100.0f)};
        telem_send(TELEM_TEMP, t, 4);
    }
}
