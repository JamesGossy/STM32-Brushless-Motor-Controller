#include "can_proto.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "hal.h"
#include "config.h"
#include <string.h>

static float getf(const uint8_t *d) { float f; memcpy(&f, d, 4); return f; }

static void send(uint32_t cmd, const void *d)
{
    hal_can_send((cfg.node_id << 5) | cmd, (const uint8_t *)d, 8);
}

static void handle(uint32_t cmd, const uint8_t *d, uint8_t len)
{
    switch (cmd) {
    case CMD_ESTOP:        app_set_state(0, SRC_CAN); break;
    case CMD_SET_STATE:    if (len >= 1) app_set_state(d[0], SRC_CAN); break;
    case CMD_SET_IQ:       if (len >= 4) app_set_iq(getf(d), SRC_CAN); break;
    case CMD_SET_SPEED:    if (len >= 4) app_set_speed(getf(d), SRC_CAN); break;
    case CMD_CLEAR_FAULTS: app_clear_faults(); break;
    case CMD_SET_LIMITS:   if (len >= 8) app_set_limits(getf(d), getf(d + 4)); break;
    case CMD_SET_NODE_ID:
        if (len >= 1 && d[0] >= 1 && d[0] <= 63) { cfg.node_id = d[0]; app_save_req = 1; }
        break;
    }
}

void can_proto_poll(void)
{
    uint32_t id;
    uint8_t d[8], len;
    while (hal_can_recv(&id, d, &len)) {
        if (id == 0) foc.req = REQ_IDLE;
        else if ((id >> 5) == cfg.node_id) handle(id & 0x1Fu, d, len);
    }
}

/* one frame per ms so a 3-deep TX FIFO never overflows */
void can_proto_telemetry(uint32_t n, uint16_t s1, uint16_t s2)
{
    uint32_t slot = n % CAN_TELEM_MS;
    float a[2];
    if (slot == 0) {
        uint8_t hb[8] = {foc.state, foc.mode, (uint8_t)foc.faults, (uint8_t)(foc.faults >> 8),
                         (uint8_t)s1, (uint8_t)(s1 >> 8), (uint8_t)s2, (uint8_t)(s2 >> 8)};
        send(MSG_HEARTBEAT, hb);
    } else if (slot == 1) {
        a[0] = foc.iq; a[1] = foc.omega_m;
        send(MSG_IQ_SPEED, a);
    } else if (slot == 2) {
        a[0] = foc.id; a[1] = foc.theta_m;
        send(MSG_ID_POS, a);
    } else if (slot == 3) {
        uint8_t b[8];
        float v = foc.vbus;
        int16_t tf = (int16_t)(foc.t_fet * 10), ta = (int16_t)(foc.t_amb * 10);
        memcpy(b, &v, 4); memcpy(b + 4, &tf, 2); memcpy(b + 6, &ta, 2);
        send(MSG_BUS_TEMP, b);
    }
}
