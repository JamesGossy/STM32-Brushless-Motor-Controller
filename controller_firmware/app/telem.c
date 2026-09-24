/*
 * telem.c - builds telemetry frames and streams them on the serial link.
 */
#include "telem.h"
#include "foc.h"
#include "svpwm.h"
#include "hal.h"
#include "config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RPM_PER_RAD_S 9.5492966f

int telem_enabled = 1;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Write a complete frame to `out` (needs len + 7 bytes). Returns its size. */
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

/* Frame a payload and send it. */
void telem_send(uint8_t type, const void *payload, uint16_t len)
{
    uint8_t frame[TELEM_MAX_PAYLOAD + 7];
    if (len > TELEM_MAX_PAYLOAD) len = TELEM_MAX_PAYLOAD;
    hal_serial_write(frame, telem_frame(type, payload, len, frame));
}

/* printf-style text message, shown in the dashboard's log panel. */
void telem_log(const char *fmt, ...)
{
    char text[TELEM_MAX_PAYLOAD];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n >= (int)sizeof text) n = sizeof text - 1;
    telem_send(TELEM_LOG, text, (uint16_t)n);
}

/* Called every millisecond: sends the data frames every TELEM_MS and the
   temperatures every 100 ms. */
void telem_tick(uint32_t tick)
{
    if (!telem_enabled) return;

    if (tick % TELEM_MS == 0) {
        uint8_t motor[20];
        uint16_t faults = foc.faults;
        float currents[4] = {foc.iq, foc.id, foc.iq_ref, foc.id_ref};
        motor[0] = foc.state;
        motor[1] = foc.mode;
        memcpy(motor + 2, &faults, 2);
        memcpy(motor + 4, currents, sizeof currents);
        telem_send(TELEM_MOTOR, motor, sizeof motor);

        /* in torque mode there is no speed setpoint, so show the actual speed */
        float ref = foc.mode == MODE_SPEED ? foc.speed_ref : foc.omega_m;
        float speed[3] = {foc.omega_m * RPM_PER_RAD_S, ref * RPM_PER_RAD_S, foc.theta_e};
        telem_send(TELEM_SPEED, speed, sizeof speed);

        float volt[6] = {foc.vbus, foc.vd, foc.vq, duty[0], duty[1], duty[2]};
        telem_send(TELEM_VOLT, volt, sizeof volt);

        float phase[3] = {foc.ia, foc.ib, foc.ic};
        telem_send(TELEM_PHASE, phase, sizeof phase);
    }

    if (tick % 100 == 0) {
        int16_t temps[2] = {(int16_t)(foc.t_fet * 100.0f), (int16_t)(foc.t_amb * 100.0f)};
        telem_send(TELEM_TEMP, temps, sizeof temps);
    }
}
