#pragma once
#include <stdint.h>
#include <string.h>
#include "telem.h"

/* minimal decoder for the telemetry stream, used by tests */
typedef struct { uint8_t type; uint16_t len; uint8_t payload[TELEM_MAX_PAYLOAD]; } frame_t;

/* parses frames from buf[0..n); returns count written, *bad = CRC failures */
static inline int parse_frames(const uint8_t *buf, size_t n, frame_t *out, int max, int *bad)
{
    int count = 0;
    size_t i = 0;
    *bad = 0;
    while (i + 7 <= n && count < max) {
        if (buf[i] != 0xAA || buf[i + 1] != 0x55) { i++; continue; }
        uint16_t len = (uint16_t)(buf[i + 3] | (buf[i + 4] << 8));
        if (len > TELEM_MAX_PAYLOAD) { i += 2; continue; }
        if (i + 7 + len > n) break;
        uint16_t crc = (uint16_t)(buf[i + 5 + len] | (buf[i + 6 + len] << 8));
        if (crc16_ccitt(buf + i + 2, 3u + len) != crc) { (*bad)++; i += 2; continue; }
        out[count].type = buf[i + 2];
        out[count].len = len;
        memcpy(out[count].payload, buf + i + 5, len);
        count++;
        i += 7u + len;
    }
    return count;
}

static inline float frame_f32(const frame_t *f, int off) { float v; memcpy(&v, f->payload + off, 4); return v; }
