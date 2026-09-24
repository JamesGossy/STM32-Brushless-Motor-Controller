/*
 * frames.h - decodes the serial telemetry stream so tests can check it.
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include "telem.h"

typedef struct {
    uint8_t type;
    uint16_t len;
    uint8_t payload[TELEM_MAX_PAYLOAD];
} frame_t;

/* Parse up to `max` frames from buf. Skips garbage between frames and
   counts frames with a bad CRC in *bad. Returns the number parsed. */
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

/* Float at byte offset `off` of a frame payload. */
static inline float frame_f32(const frame_t *f, int off)
{
    float v;
    memcpy(&v, f->payload + off, 4);
    return v;
}

/* True if any LOG frame in the list starts with `prefix`. */
static inline int has_log(const frame_t *f, int n, const char *prefix)
{
    size_t k = strlen(prefix);
    for (int i = 0; i < n; i++)
        if (f[i].type == TELEM_LOG && f[i].len >= k && !memcmp(f[i].payload, prefix, k)) return 1;
    return 0;
}
