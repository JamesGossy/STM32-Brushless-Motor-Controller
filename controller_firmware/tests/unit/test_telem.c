/*
 * test_telem.c - unit tests for the telemetry frame format.
 */
#include "test.h"
#include "frames.h"
#include "telem.h"
#include "sim.h"

/* CRC matches the published CRC-16/CCITT-FALSE check value. */
static void crc_reference_vector(void)
{
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1);   /* CRC-16/CCITT-FALSE check value */
    CHECK(crc16_ccitt(0, 0) == 0xFFFF);
}

/* Header, payload and CRC land where the dashboard expects them. */
static void frame_layout(void)
{
    uint8_t p[3] = {1, 2, 3}, f[16];
    size_t n = telem_frame(0x42, p, 3, f);
    CHECK(n == 10);
    CHECK(f[0] == 0xAA && f[1] == 0x55 && f[2] == 0x42 && f[3] == 3 && f[4] == 0);
    CHECK(f[5] == 1 && f[6] == 2 && f[7] == 3);
    uint16_t crc = crc16_ccitt(f + 2, 6);
    CHECK(f[8] == (crc & 0xFF) && f[9] == (crc >> 8));
}

/* Frames decode correctly with noise between them. */
static void roundtrip_with_garbage(void)
{
    uint8_t buf[64], p[4] = {9, 8, 7, 6};
    size_t n = 0;
    buf[n++] = 0x13; buf[n++] = 0xAA;                  /* noise, lone sync byte */
    n += telem_frame(TELEM_LOG, "hi", 2, buf + n);
    buf[n++] = 0x55;
    n += telem_frame(TELEM_TEMP, p, 4, buf + n);
    frame_t fr[4];
    int bad;
    int k = parse_frames(buf, n, fr, 4, &bad);
    CHECK(k == 2 && bad == 0);
    CHECK(fr[0].type == TELEM_LOG && fr[0].len == 2 && !memcmp(fr[0].payload, "hi", 2));
    CHECK(fr[1].type == TELEM_TEMP && fr[1].payload[3] == 6);
}

/* A flipped byte is caught by the CRC. */
static void corrupted_frame_rejected(void)
{
    uint8_t buf[32], p[4] = {1, 2, 3, 4};
    size_t n = telem_frame(TELEM_TEMP, p, 4, buf);
    buf[6] ^= 0xFF;
    frame_t fr[2];
    int bad;
    CHECK(parse_frames(buf, n, fr, 2, &bad) == 0 && bad == 1);
}

/* telem_log() sends a LOG frame on the serial link. */
static void log_goes_to_serial(void)
{
    sim_config_t c;
    sim_default_config(&c);
    sim_init(&c);
    uint8_t junk[4096];
    while (sim_serial_take(junk, sizeof junk)) {}
    telem_log("value=%d", 42);
    uint8_t buf[256];
    size_t n = sim_serial_take(buf, sizeof buf);
    frame_t fr[2];
    int bad;
    CHECK(parse_frames(buf, n, fr, 2, &bad) == 1);
    CHECK(fr[0].type == TELEM_LOG && fr[0].len == 8 && !memcmp(fr[0].payload, "value=42", 8));
}

int main(void)
{
    test_case t[] = {T(crc_reference_vector), T(frame_layout), T(roundtrip_with_garbage),
                     T(corrupted_frame_rejected), T(log_goes_to_serial)};
    return run_tests(t, sizeof t / sizeof t[0]);
}
