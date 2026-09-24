#pragma once
#include <stdint.h>

/* 11-bit ID = (node << 5) | cmd, floats little endian, ID 0x000 = broadcast e-stop */
enum {
    CMD_ESTOP        = 0x00,
    CMD_SET_STATE    = 0x01,  /* u8: 0 idle, 1 torque, 2 speed, 3 calibrate */
    CMD_SET_IQ       = 0x02,  /* f32 A */
    CMD_SET_SPEED    = 0x03,  /* f32 mech rad/s */
    CMD_CLEAR_FAULTS = 0x04,
    CMD_SET_LIMITS   = 0x05,  /* f32 current A, f32 speed rad/s */
    CMD_SET_NODE_ID  = 0x06,  /* u8 */
    MSG_HEARTBEAT    = 0x10,  /* u8 state, u8 mode, u16 faults, u16 drv_stat1, u16 drv_stat2 */
    MSG_IQ_SPEED     = 0x11,  /* f32 iq, f32 speed */
    MSG_ID_POS       = 0x12,  /* f32 id, f32 mech angle */
    MSG_BUS_TEMP     = 0x13,  /* f32 vbus, i16 fet degC*10, i16 amb degC*10 */
};

void can_proto_poll(void);
void can_proto_telemetry(uint32_t n, uint16_t drv_s1, uint16_t drv_s2);
