/*
 * can_proto.c - handles received CAN commands and sends telemetry.
 */
#include "can_proto.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "hal.h"
#include "config.h"
#include <string.h>

/* Read a little endian float from a CAN payload. */
static float get_f32(const uint8_t *d)
{
    float f;
    memcpy(&f, d, 4);
    return f;
}

/* Send one 8-byte telemetry frame from this node. */
static void send(uint32_t msg, const void *data)
{
    hal_can_send((cfg.node_id << 5) | msg, (const uint8_t *)data, 8);
}

/* Act on one command addressed to this node. */
static void handle(uint32_t cmd, const uint8_t *d, uint8_t len)
{
    switch (cmd) {
    case CMD_ESTOP:
        app_request(RUN_IDLE, SRC_CAN);
        break;
    case CMD_SET_STATE:
        /* not calibrated / faulted requests are refused; the heartbeat shows the state */
        if (len >= 1) app_request(d[0] <= RUN_CALIBRATE ? d[0] : RUN_IDLE, SRC_CAN);
        break;
    case CMD_SET_IQ:
        if (len >= 4) app_set_iq(get_f32(d), SRC_CAN);
        break;
    case CMD_SET_SPEED:
        if (len >= 4) app_set_speed(get_f32(d), SRC_CAN);
        break;
    case CMD_CLEAR_FAULTS:
        app_clear_faults();
        break;
    case CMD_SET_LIMITS:
        if (len >= 8) app_set_limits(get_f32(d), get_f32(d + 4));
        break;
    case CMD_SET_NODE_ID:
        if (len >= 1 && d[0] >= 1 && d[0] <= 63) {
            cfg.node_id = d[0];
            app_save_req = 1;
        }
        break;
    }
}

/* Drain the receive queue. */
void can_proto_poll(void)
{
    uint32_t id;
    uint8_t d[8], len;

    while (hal_can_recv(&id, d, &len)) {
        if (id == 0)
            foc.req = REQ_IDLE;     /* broadcast e-stop */
        else if ((id >> 5) == cfg.node_id)
            handle(id & 0x1Fu, d, len);
    }
}

/* Called every millisecond. Sends one frame per tick so the hardware's
   3-deep TX FIFO never overflows. */
void can_proto_telemetry(uint32_t tick, uint16_t s1, uint16_t s2)
{
    uint8_t b[8];
    float f[2];

    switch (tick % CAN_TELEM_MS) {
    case 0:
        b[0] = foc.state;
        b[1] = foc.mode;
        b[2] = (uint8_t)foc.faults;
        b[3] = (uint8_t)(foc.faults >> 8);
        b[4] = (uint8_t)s1;
        b[5] = (uint8_t)(s1 >> 8);
        b[6] = (uint8_t)s2;
        b[7] = (uint8_t)(s2 >> 8);
        send(MSG_HEARTBEAT, b);
        break;
    case 1:
        f[0] = foc.iq;
        f[1] = foc.omega_m;
        send(MSG_IQ_SPEED, f);
        break;
    case 2:
        f[0] = foc.id;
        f[1] = foc.theta_m;
        send(MSG_ID_POS, f);
        break;
    case 3: {
        float vbus = foc.vbus;
        int16_t t_fet = (int16_t)(foc.t_fet * 10), t_amb = (int16_t)(foc.t_amb * 10);
        memcpy(b, &vbus, 4);
        memcpy(b + 4, &t_fet, 2);
        memcpy(b + 6, &t_amb, 2);
        send(MSG_BUS_TEMP, b);
        break;
    }
    }
}
