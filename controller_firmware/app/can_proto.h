/*
 * can_proto.h - CAN command and telemetry protocol.
 *
 * 11-bit ID = (node << 5) | cmd. Floats are little endian.
 * ID 0x000 is a broadcast e-stop for every node.
 */
#pragma once
#include <stdint.h>

enum {
    /* host -> drive */
    CMD_ESTOP        = 0x00,
    CMD_SET_STATE    = 0x01,  /* u8: 0 idle, 1 torque, 2 speed, 3 calibrate */
    CMD_SET_IQ       = 0x02,  /* f32 A */
    CMD_SET_SPEED    = 0x03,  /* f32 mechanical rad/s */
    CMD_CLEAR_FAULTS = 0x04,
    CMD_SET_LIMITS   = 0x05,  /* f32 current A, f32 speed rad/s */
    CMD_SET_NODE_ID  = 0x06,  /* u8 */

    /* drive -> host, every CAN_TELEM_MS */
    MSG_HEARTBEAT    = 0x10,  /* u8 state, u8 mode, u16 faults, u16 drv status 1, u16 drv status 2 */
    MSG_IQ_SPEED     = 0x11,  /* f32 iq, f32 speed */
    MSG_ID_POS       = 0x12,  /* f32 id, f32 mechanical angle */
    MSG_BUS_TEMP     = 0x13,  /* f32 vbus, i16 FET degC*10, i16 ambient degC*10 */
};

void can_proto_poll(void);
void can_proto_telemetry(uint32_t tick, uint16_t drv_status1, uint16_t drv_status2);
