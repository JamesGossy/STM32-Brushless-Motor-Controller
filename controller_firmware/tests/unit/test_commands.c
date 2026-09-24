#include "test.h"
#include "frames.h"
#include "sim.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "config.h"

static void setup(void)
{
    static int done;
    if (!done) {
        sim_config_t c;
        sim_default_config(&c);
        sim_init(&c);
        sim_run_ms(300);                    /* past the boot offset measurement */
        done = 1;
    }
    uint8_t junk[4096];
    while (sim_serial_take(junk, sizeof junk)) {}
}

static void serial(const char *s) { sim_serial_inject(s); app_poll(); }

static void can(uint32_t cmd, const void *d, uint8_t len)
{
    sim_can_inject((cfg.node_id << 5) | cmd, d, len);
    app_poll();
}

static void serial_setpoints(void)
{
    setup();
    serial("iq 3.5\n");
    CHECK_NEAR(foc.cmd_iq, 3.5, 1e-6);
    CHECK(app_setpoint_src == SRC_SERIAL);
    serial("rpm 1000\r\n");
    CHECK_NEAR(foc.cmd_speed, 104.71976, 1e-3);
    serial("iq 999\n");
    CHECK_NEAR(foc.cmd_iq, I_MAX, 1e-6);
    serial("rpm -99999\n");
    CHECK_NEAR(foc.cmd_speed, -SPEED_MAX, 1e-3);
    serial("iq nan\n");
    CHECK(foc.cmd_iq == 0.0f);
}

static void serial_states(void)
{
    setup();
    serial("motor speed\n");  CHECK(foc.req == REQ_SPEED);
    serial("motor torque\n"); CHECK(foc.req == REQ_TORQUE);
    serial("calibrate\n");    CHECK(foc.req == REQ_CAL);
    serial("motor off\n");    CHECK(foc.req == REQ_IDLE);
    foc.req = REQ_NONE;
}

static void serial_limits_and_errors(void)
{
    setup();
    serial("limits 5 3000\n");
    CHECK_NEAR(foc.i_limit, 5.0, 1e-6);
    CHECK_NEAR(foc.speed_limit, 314.159, 1e-2);
    serial("limits -1 -1\n");
    CHECK(foc.i_limit == 0.0f && foc.speed_limit == 0.0f);
    serial("bogus\n");
    uint8_t buf[512];
    size_t n = sim_serial_take(buf, sizeof buf);
    frame_t fr[4];
    int bad;
    int k = parse_frames(buf, n, fr, 4, &bad);
    CHECK(k >= 1 && fr[k - 1].type == TELEM_LOG && !memcmp(fr[k - 1].payload, "unknown command: bogus", 22));
    foc.i_limit = I_MAX;
    foc.speed_limit = SPEED_MAX;
}

static void serial_status_reply(void)
{
    setup();
    serial("status\n");
    uint8_t buf[512];
    size_t n = sim_serial_take(buf, sizeof buf);
    frame_t fr[4];
    int bad;
    int k = parse_frames(buf, n, fr, 4, &bad);
    CHECK(k == 1 && fr[0].type == TELEM_LOG && !memcmp(fr[0].payload, "state=", 6));
}

static void telem_toggle(void)
{
    setup();
    serial("telem off\n");
    sim_run_ms(20);
    uint8_t buf[512];
    CHECK(sim_serial_take(buf, sizeof buf) == 0);
    serial("telem on\n");
    sim_run_ms(20);
    CHECK(sim_serial_take(buf, sizeof buf) > 0);
}

static void can_setpoints(void)
{
    setup();
    float v = 2.5f;
    can(CMD_SET_IQ, &v, 4);
    CHECK_NEAR(foc.cmd_iq, 2.5, 1e-6);
    CHECK(app_setpoint_src == SRC_CAN);
    v = 50.0f;
    can(CMD_SET_SPEED, &v, 4);
    CHECK_NEAR(foc.cmd_speed, 50.0, 1e-6);
    uint32_t nan_bits = 0x7FC00000u;
    can(CMD_SET_IQ, &nan_bits, 4);
    CHECK(foc.cmd_iq == 0.0f);
    v = 1e9f;
    can(CMD_SET_SPEED, &v, 4);
    CHECK_NEAR(foc.cmd_speed, SPEED_MAX, 1e-3);
    can(CMD_SET_IQ, &v, 2);                  /* short frame ignored */
    CHECK_NEAR(foc.cmd_iq, 0.0, 1e-6);
}

static void can_node_filter_and_estop(void)
{
    setup();
    float v = 4.0f;
    sim_can_inject(((cfg.node_id + 1) << 5) | CMD_SET_IQ, &v, 4);
    app_poll();
    CHECK(foc.cmd_iq != 4.0f);
    uint8_t st = 2;
    can(CMD_SET_STATE, &st, 1);
    CHECK(foc.req == REQ_SPEED);
    sim_can_inject(0, "", 0);
    app_poll();
    CHECK(foc.req == REQ_IDLE);
    foc.req = REQ_NONE;
}

static void can_node_id_saved(void)
{
    setup();
    uint8_t id = 7;
    can(CMD_SET_NODE_ID, &id, 1);
    CHECK(cfg.node_id == 7);
    CHECK(app_save_req == 0);               /* saved immediately while idle */
    cfg_defaults();
    cfg_load();
    CHECK(cfg.node_id == 7);
    id = 0;
    can(CMD_SET_NODE_ID, &id, 1);           /* 0 is the broadcast id, rejected */
    CHECK(cfg.node_id == 7);
    cfg.node_id = CAN_NODE_ID;
}

int main(void)
{
    test_case t[] = {T(serial_setpoints), T(serial_states), T(serial_limits_and_errors), T(serial_status_reply),
                     T(telem_toggle), T(can_setpoints), T(can_node_filter_and_estop), T(can_node_id_saved)};
    return run_tests(t, sizeof t / sizeof t[0]);
}
