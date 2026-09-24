/*
 * telem.h - framed binary telemetry on the serial link.
 *
 * Frame (little endian):
 *   0xAA 0x55 | TYPE u8 | LEN u16 | PAYLOAD[LEN] | CRC16 u16
 * CRC16 is CRC-16/CCITT-FALSE over TYPE, LEN and PAYLOAD. The dashboard
 * (tools/dashboard) decodes exactly this format.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#define TELEM_MOTOR  0x01   /* u8 state, u8 mode, u16 faults, f32 iq, id, iq_ref, id_ref */
#define TELEM_SPEED  0x02   /* f32 speed_rpm, speed_ref_rpm, theta_e */
#define TELEM_VOLT   0x03   /* f32 vbus, vd, vq, duty_a, duty_b, duty_c */
#define TELEM_PHASE  0x04   /* f32 ia, ib, ic */
#define TELEM_TEMP   0x05   /* i16 FET degC*100, i16 ambient degC*100 */
#define TELEM_LOG    0x10   /* ascii text */

#define TELEM_MAX_PAYLOAD 128

extern int telem_enabled;

uint16_t crc16_ccitt(const uint8_t *data, size_t len);
size_t telem_frame(uint8_t type, const void *payload, uint16_t len, uint8_t *out);
void telem_send(uint8_t type, const void *payload, uint16_t len);
void telem_log(const char *fmt, ...);
void telem_tick(uint32_t tick);
