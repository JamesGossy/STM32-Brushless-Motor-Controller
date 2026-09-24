/*
 * test_commands.c - serial console and CAN command handling.
 *
 * Commands are injected through the simulated serial/CAN links and the
 * effect on the setpoints and state requests is checked directly.
 */
#include "test.h"
#include "frames.h"
#include "sim.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "config.h"

/* Boot the simulator once (past offset measurement) and empty the serial output. */
static void setup(void)
{
    static int booted;
    if (!booted) {
        sim_config_t c;
        sim_default_config(&c);
        sim_init(&c);
        sim_run_ms(300);
        booted = 1;
    }
    uint8_t junk[4096];
    while (sim_serial_take(junk, sizeof junk)) {}
    foc.req = REQ_NONE;
}

/* Send one serial line and let the main loop process it. */
static void serial(const char *line)
{
    sim_serial_inject(line);
    app_poll();
}

/* Send a CAN command to this node. */
static void can(uint32_t cmd, const void *data, uint8_t len)
{
    sim_can_inject((cfg.node_id << 5) | cmd, data, len);
    app_poll();
}

/* Replies (LOG frames) produced since the last setup(). */
static int replies(frame_t *out, int max)
{
    static uint8_t buf[8192];
    int bad;
    size_t n = sim_serial_take(buf, sizeof buf);
    return parse_frames(buf, n, out, max, &bad);
}

/* Setpoints are stored, clamped, and NaN is rejected. */
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

/* Before calibration, run requests are refused with a helpful message. */
static void refuses_until_calibrated(void)
{
    setup();
    cfg.cal_valid = 0;
    serial("motor 1000\n");
    CHECK(foc.req == REQ_NONE);
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
    frame_t f[8];
    int n = replies(f, 8);
    CHECK(has_log(f, n, "not calibrated"));

    serial("calibrate\n");
    CHECK(foc.req == REQ_CAL);
}

/* With calibration done, each command requests the right state. */
static void state_commands(void)
{
    setup();
    cfg.cal_valid = 1;
    serial("motor speed\n");  CHECK(foc.req == REQ_SPEED);
    serial("motor torque\n"); CHECK(foc.req == REQ_TORQUE);
    serial("motor off\n");    CHECK(foc.req == REQ_IDLE);

    serial("motor 1500\n");
    CHECK(foc.req == REQ_SPEED);
    CHECK_NEAR(foc.cmd_speed, 1500 * 0.10471976, 1e-3);
    cfg.cal_valid = 0;
}

/* A latched fault must be cleared before anything else starts. */
static void faults_block_requests(void)
{
    setup();
    cfg.cal_valid = 1;
    foc_fault(F_OVERTEMP);
    serial("motor speed\n");
    CHECK(foc.req == REQ_NONE);
    frame_t f[8];
    int n = replies(f, 8);
    CHECK(has_log(f, n, "clear the faults"));

    serial("clear\n");
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
    cfg.cal_valid = 0;
}

/* Limits are clamped to 0..config limits; bad input gets a usage reply. */
static void limits_and_errors(void)
{
    setup();
    serial("limits 5 3000\n");
    CHECK_NEAR(foc.i_limit, 5.0, 1e-6);
    CHECK_NEAR(foc.speed_limit, 314.159, 1e-2);
    serial("limits -1 -1\n");
    CHECK(foc.i_limit == 0.0f && foc.speed_limit == 0.0f);
    serial("bogus\n");
    serial("rpm abc\n");
    frame_t f[8];
    int n = replies(f, 8);
    CHECK(has_log(f, n, "unknown command 'bogus'"));
    CHECK(has_log(f, n, "usage: rpm"));
    foc.i_limit = I_MAX;
    foc.speed_limit = SPEED_MAX;
}

/* `status` replies with a LOG line. */
static void status_reply(void)
{
    setup();
    serial("status\n");
    frame_t f[4];
    int n = replies(f, 4);
    CHECK(n == 1 && has_log(f, n, "state=IDLE"));
}

/* `telem off` stops the stream, `telem on` restarts it. */
static void telem_toggle(void)
{
    setup();
    uint8_t buf[512];
    serial("telem off\n");
    sim_run_ms(20);
    CHECK(sim_serial_take(buf, sizeof buf) == 0);
    serial("telem on\n");
    sim_run_ms(20);
    CHECK(sim_serial_take(buf, sizeof buf) > 0);
}

/* CAN setpoints: stored, clamped, NaN and short frames rejected. */
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

    can(CMD_SET_IQ, &v, 2);
    CHECK_NEAR(foc.cmd_iq, 0.0, 1e-6);
}

/* Frames for other nodes are ignored; ID 0 is a broadcast e-stop. */
static void can_node_filter_and_estop(void)
{
    setup();
    cfg.cal_valid = 1;
    float v = 4.0f;
    sim_can_inject(((cfg.node_id + 1) << 5) | CMD_SET_IQ, &v, 4);
    app_poll();
    CHECK(foc.cmd_iq != 4.0f);

    uint8_t state = RUN_SPEED;
    can(CMD_SET_STATE, &state, 1);
    CHECK(foc.req == REQ_SPEED);

    sim_can_inject(0, "", 0);
    app_poll();
    CHECK(foc.req == REQ_IDLE);
    cfg.cal_valid = 0;
}

/* A new node id is saved straight away while idle; 0 is rejected. */
static void can_node_id_saved(void)
{
    setup();
    uint8_t id = 7;
    can(CMD_SET_NODE_ID, &id, 1);
    CHECK(cfg.node_id == 7);
    CHECK(app_save_req == 0);
    cfg_defaults();
    cfg_load();
    CHECK(cfg.node_id == 7);

    id = 0;
    can(CMD_SET_NODE_ID, &id, 1);
    CHECK(cfg.node_id == 7);
    cfg.node_id = CAN_NODE_ID;
}

int main(void)
{
    test_case t[] = {
        T(serial_setpoints), T(refuses_until_calibrated), T(state_commands), T(faults_block_requests),
        T(limits_and_errors), T(status_reply), T(telem_toggle), T(can_setpoints),
        T(can_node_filter_and_estop), T(can_node_id_saved),
    };
    return run_tests(t, sizeof t / sizeof t[0]);
}
