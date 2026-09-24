/*
 * test_sil.c - software-in-the-loop integration tests.
 *
 * The real firmware (app/) runs against the simulated motor, inverter and
 * sensors. Each case boots a fresh drive, so they run one per process:
 *
 *   test_sil <case>        (run without arguments to list the cases)
 */
#include "test.h"
#include "frames.h"
#include "sim.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "config.h"
#include <string.h>

#define RPM(x) ((x) * 0.10471976f)     /* rpm -> rad/s */
#define TWO_PI_D 6.283185307179586

static sim_config_t sim_config;

/* ---- helpers ---- */

/* Fresh simulator, booted past the offset measurement. */
static void boot(void)
{
    sim_default_config(&sim_config);
    sim_init(&sim_config);
    sim_run_ms(300);
}

/* Type a console command. */
static void cmd(const char *line)
{
    sim_serial_inject(line);
    sim_serial_inject("\n");
}

static void discard_serial(void)
{
    uint8_t junk[4096];
    while (sim_serial_take(junk, sizeof junk)) {}
}

/* Run the calibration and wait for it to finish (it takes about 5 s). */
static void calibrate(void)
{
    cmd("calibrate");
    sim_run_ms(10);
    for (int i = 0; i < 100 && foc.state == ST_CAL; i++) sim_run_ms(100);
    sim_run_ms(10);
}

/* Firmware electrical angle minus the true rotor angle, wrapped to +/- pi. */
static double angle_error(void)
{
    double truth = fmod(POLE_PAIRS * sim_motor()->th_m, TWO_PI_D);
    if (truth < 0) truth += TWO_PI_D;
    double e = foc.theta_e - truth;
    while (e > TWO_PI_D / 2) e -= TWO_PI_D;
    while (e < -TWO_PI_D / 2) e += TWO_PI_D;
    return e;
}

/* Boot, calibrate and start spinning towards `rpm`. */
static void spin_up(float rpm)
{
    char line[32];
    boot();
    calibrate();
    snprintf(line, sizeof line, "motor %.1f", rpm);
    cmd(line);
}

static void can_command(uint32_t cmd_id, const void *data, uint8_t len)
{
    sim_can_inject((cfg.node_id << 5) | cmd_id, data, len);
}

static int faulted_with(uint16_t bits)
{
    return foc.state == ST_FAULT && (foc.faults & bits) && !sim_pwm_enabled();
}

/* ---- cases ---- */

/* Boots to idle with no faults, outputs off, sensible readings. */
static void t_boot(void)
{
    boot();
    CHECK(foc.state == ST_IDLE);
    CHECK(foc.faults == 0);
    CHECK(!sim_pwm_enabled());
    CHECK_NEAR(foc.vbus, sim_config.vbus, 0.2);
    CHECK_NEAR(foc.ia, 0.0, 0.1);
}

/* Calibration finds the (deliberately flipped) encoder and current sense,
   gets the angle right to ~1 degree, and saves it. */
static void t_calibration(void)
{
    boot();
    calibrate();
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
    CHECK(cfg.cal_valid == 1);
    CHECK(cfg.enc_dir == sim_config.enc_dir);
    CHECK(cfg.cur_sign == sim_config.cur_sign);
    CHECK(fabs(angle_error()) < 0.02);

    cfg_t saved = cfg;
    cfg_defaults();
    cfg_load();
    CHECK(cfg.cal_valid == 1 && cfg.enc_offset == saved.enc_offset);
}

/* Running before calibration is refused and the motor stays off. */
static void t_not_calibrated(void)
{
    boot();
    discard_serial();
    cmd("motor torque");
    sim_run_ms(5);
    CHECK(foc.state == ST_IDLE && !sim_pwm_enabled());

    static uint8_t buf[8192];
    frame_t f[64];
    int bad, n = parse_frames(buf, sim_serial_take(buf, sizeof buf), f, 64, &bad);
    CHECK(has_log(f, n, "not calibrated"));
}

/* A 5 A torque step settles within 2 ms, and the plant agrees. */
static void t_torque_step(void)
{
    boot();
    calibrate();
    cmd("motor torque");
    sim_run_ms(5);
    cmd("iq 5");
    sim_run_ms(3);
    CHECK_NEAR(foc.iq, 5.0, 0.25);
    CHECK_NEAR(foc.id, 0.0, 0.25);
    CHECK_NEAR(sim_motor()->iq, 5.0, 0.3);
    CHECK(sim_motor()->w_m > 0);
}

/* Speed follows setpoint changes. */
static void t_speed_tracking(void)
{
    spin_up(3000);
    sim_run_ms(700);
    CHECK_NEAR(sim_motor()->w_m, RPM(3000), 3.0);
    CHECK_NEAR(foc.omega_m, sim_motor()->w_m, 2.0);
    cmd("rpm 1000");
    sim_run_ms(500);
    CHECK_NEAR(sim_motor()->w_m, RPM(1000), 3.0);
}

/* A load step barely moves the speed; the current rises to carry it. */
static void t_load_rejection(void)
{
    spin_up(3000);
    sim_run_ms(700);
    sim_set_load(0.1);
    sim_run_ms(300);
    CHECK_NEAR(sim_motor()->w_m, RPM(3000), 0.02 * RPM(3000));
    CHECK_NEAR(foc.iq, 0.1 / KT, 0.8);
}

/* 9500 rpm is above base speed at 24 V: needs negative id and stays inside the voltage limit. */
static void t_field_weakening(void)
{
    spin_up(9500);
    sim_run_ms(2000);
    CHECK_NEAR(sim_motor()->w_m, RPM(9500), 5.0);
    CHECK(foc.id_ref < -5.0f);
    float vmax = MOD_MAX * foc.vbus / 1.7320508f;
    CHECK(sqrtf(foc.vd * foc.vd + foc.vq * foc.vq) < vmax);
    CHECK(foc.faults == 0);
}

/* Reverses through zero cleanly. */
static void t_reverse(void)
{
    spin_up(5000);
    sim_run_ms(800);
    cmd("rpm -5000");
    sim_run_ms(1500);
    CHECK_NEAR(sim_motor()->w_m, RPM(-5000), 5.0);
    CHECK(foc.faults == 0);
}

/* Driven entirely over CAN, with telemetry coming back. */
static void t_can_control(void)
{
    boot();
    calibrate();

    uint32_t id;
    uint8_t d[8], len;
    uint8_t state = RUN_SPEED;
    can_command(CMD_SET_STATE, &state, 1);
    for (int i = 0; i < 60; i++) {
        float speed = RPM(2000);
        can_command(CMD_SET_SPEED, &speed, 4);
        sim_run_ms(10);
        if (i == 49) while (sim_can_pop(&id, d, &len)) {}    /* keep only the last 100 ms */
    }
    CHECK(foc.state == ST_RUN);
    CHECK_NEAR(sim_motor()->w_m, RPM(2000), 3.0);

    int heartbeats = 0, speed_ok = 0;
    while (sim_can_pop(&id, d, &len)) {
        if (id == ((cfg.node_id << 5) | MSG_HEARTBEAT)) {
            heartbeats++;
            CHECK(d[0] == ST_RUN && d[1] == MODE_SPEED);
        }
        if (id == ((cfg.node_id << 5) | MSG_IQ_SPEED)) {
            float w;
            memcpy(&w, d + 4, 4);
            speed_ok = fabsf(w - RPM(2000)) < 5.0f;
        }
    }
    CHECK(heartbeats > 0);
    CHECK(speed_ok);
}

/* A CAN master that stops sending setpoints gets the motor stopped. */
static void t_can_timeout(void)
{
    boot();
    calibrate();
    uint8_t state = RUN_TORQUE;
    float iq = 1.0f;
    can_command(CMD_SET_STATE, &state, 1);
    can_command(CMD_SET_IQ, &iq, 4);
    sim_run_ms(100);
    CHECK(foc.state == ST_RUN);
    sim_run_ms(CAN_TIMEOUT_MS + 50);
    CHECK(foc.state == ST_IDLE && !sim_pwm_enabled());
}

/* Console setpoints don't time out. */
static void t_serial_latched(void)
{
    spin_up(1500);
    sim_run_ms(1500);
    CHECK(foc.state == ST_RUN);
    CHECK_NEAR(sim_motor()->w_m, RPM(1500), 3.0);
}

/* Broadcast e-stop stops the drive within a couple of ms. */
static void t_estop(void)
{
    spin_up(3000);
    sim_run_ms(500);
    sim_can_inject(0, "", 0);
    sim_run_ms(2);
    CHECK(foc.state == ST_IDLE && !sim_pwm_enabled());
}

/* A real (balanced) current spike trips instantly and can be cleared. */
static void t_overcurrent(void)
{
    spin_up(1000);
    sim_run_ms(300);
    sim_set_adc_bias(0, 35.0);
    sim_set_adc_bias(1, -35.0);
    sim_run_ms(2);
    CHECK(faulted_with(F_OVERCURRENT));

    sim_set_adc_bias(0, 0.0);
    sim_set_adc_bias(1, 0.0);
    cmd("clear");
    sim_run_ms(2);
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
}

/* One drifting current sensor breaks ia + ib + ic = 0. */
static void t_current_sensor_fault(void)
{
    spin_up(1000);
    sim_run_ms(300);
    sim_set_adc_bias(2, 8.0);
    sim_run_ms(20);
    CHECK(faulted_with(F_CURRENT_SUM));
}

static void t_undervoltage(void)
{
    spin_up(1000);
    sim_run_ms(300);
    sim_set_vbus(6.0);
    sim_run_ms(2);
    CHECK(faulted_with(F_UNDERVOLT));
}

static void t_overtemp(void)
{
    spin_up(1000);
    sim_run_ms(300);
    sim_set_temp(TEMP_MAX + 5.0, 25.0);
    sim_run_ms(5);
    CHECK(faulted_with(F_OVERTEMP));
}

/* A frozen encoder locks the rotor against a fixed current vector: caught as a stall. */
static void t_encoder_fault(void)
{
    spin_up(3000);
    sim_run_ms(600);
    sim_freeze_encoder(1);
    sim_run_ms(STALL_MS + 400);
    CHECK(faulted_with(F_ENCODER | F_STALL));
}

/* A sudden angle jump (loose magnet, corrupt SPI) trips immediately. */
static void t_encoder_glitch(void)
{
    spin_up(3000);
    sim_run_ms(600);
    sim_set_encoder_offset(sim_config.enc_offset + 1.5);
    sim_run_ms(5);
    CHECK(faulted_with(F_ENCODER));
}

/* A jammed rotor trips the stall detector. */
static void t_stall(void)
{
    spin_up(1000);
    sim_run_ms(500);
    sim_lock_rotor(1);
    sim_run_ms(STALL_MS + 800);
    CHECK(faulted_with(F_STALL));
}

static void t_gate_fault(void)
{
    spin_up(1000);
    sim_run_ms(300);
    sim_set_gate_fault(1);
    sim_run_ms(20);
    CHECK(faulted_with(F_DRV));
}

/* Serial telemetry arrives at the right rate with valid CRCs and sane values. */
static void t_telemetry(void)
{
    spin_up(2000);
    sim_run_ms(700);
    discard_serial();
    sim_run_ms(100);

    static uint8_t buf[65536];
    static frame_t f[512];
    int bad;
    int n = parse_frames(buf, sim_serial_take(buf, sizeof buf), f, 512, &bad);

    int motor = 0, speed = 0, temp = 0;
    for (int i = 0; i < n; i++) {
        if (f[i].type == TELEM_MOTOR) { motor++; CHECK(f[i].payload[0] == ST_RUN); }
        if (f[i].type == TELEM_SPEED) { speed++; CHECK_NEAR(frame_f32(&f[i], 0), 2000.0, 30.0); }
        if (f[i].type == TELEM_TEMP) temp++;
    }
    CHECK(bad == 0);
    CHECK(motor >= 100 / TELEM_MS - 1 && speed == motor);
    CHECK(temp >= 1);
}

int main(int argc, char **argv)
{
    static const test_case cases[] = {
        {"boot", t_boot}, {"calibration", t_calibration}, {"not_calibrated", t_not_calibrated},
        {"torque_step", t_torque_step}, {"speed_tracking", t_speed_tracking},
        {"load_rejection", t_load_rejection}, {"field_weakening", t_field_weakening},
        {"reverse", t_reverse}, {"can_control", t_can_control}, {"can_timeout", t_can_timeout},
        {"serial_latched", t_serial_latched}, {"estop", t_estop}, {"overcurrent", t_overcurrent},
        {"current_sensor_fault", t_current_sensor_fault}, {"undervoltage", t_undervoltage},
        {"overtemp", t_overtemp}, {"encoder_fault", t_encoder_fault}, {"encoder_glitch", t_encoder_glitch},
        {"stall", t_stall}, {"gate_fault", t_gate_fault}, {"telemetry", t_telemetry},
    };
    const int count = sizeof cases / sizeof cases[0];

    if (argc < 2) {
        printf("usage: test_sil <case>\n");
        for (int i = 0; i < count; i++) printf("  %s\n", cases[i].name);
        return 2;
    }
    for (int i = 0; i < count; i++) {
        if (!strcmp(argv[1], cases[i].name)) {
            cases[i].fn();
            printf("%s %s\n", test_failures ? "FAIL" : "PASS", cases[i].name);
            return test_failures ? 1 : 0;
        }
    }
    printf("unknown case '%s'\n", argv[1]);
    return 2;
}
