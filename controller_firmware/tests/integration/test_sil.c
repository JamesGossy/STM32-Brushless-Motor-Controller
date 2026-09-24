/* Software-in-the-loop tests: the real firmware app code runs against the
   motor model. One case per process: `test_sil <case>`. */
#include "test.h"
#include "frames.h"
#include "sim.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "can_proto.h"
#include "config.h"
#include <string.h>

#define RPM(x) ((x) * 0.10471976f)

static sim_config_t sc;

static void boot(void)
{
    sim_default_config(&sc);
    sim_init(&sc);
    sim_run_ms(300);
}

static void cmd(const char *s)
{
    sim_serial_inject(s);
    sim_serial_inject("\n");
}

static void drain(void)
{
    uint8_t junk[4096];
    while (sim_serial_take(junk, sizeof junk)) {}
}

static void calibrate(void)
{
    cmd("calibrate");
    for (int i = 0; i < 100 && foc.state != ST_IDLE; i++) sim_run_ms(100);
    sim_run_ms(100);
    for (int i = 0; i < 100 && foc.state == ST_CAL; i++) sim_run_ms(100);
}

static double elec_angle_error(void)
{
    motor_t *m = sim_motor();
    double truth = fmod(POLE_PAIRS * m->th_m, 2 * 3.14159265358979);
    if (truth < 0) truth += 2 * 3.14159265358979;
    double e = foc.theta_e - truth;
    while (e > 3.14159265) e -= 2 * 3.14159265358979;
    while (e < -3.14159265) e += 2 * 3.14159265358979;
    return e;
}

static void calibrated_speed_mode(float rpm)
{
    boot();
    calibrate();
    cmd("motor speed");
    sim_run_ms(5);
    char s[32];
    snprintf(s, sizeof s, "rpm %.1f", rpm);
    cmd(s);
}

/* ---- cases ---- */

static void t_boot(void)
{
    boot();
    CHECK(foc.state == ST_IDLE);
    CHECK(foc.faults == 0);
    CHECK(!sim_pwm_enabled());
    CHECK_NEAR(foc.vbus, sc.vbus, 0.2);
    CHECK_NEAR(foc.ia, 0.0, 0.1);
}

static void t_calibration(void)
{
    boot();
    calibrate();
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
    CHECK(cfg.cal_valid == 1);
    CHECK(cfg.enc_dir == sc.enc_dir);
    CHECK(cfg.cur_sign == sc.cur_sign);
    CHECK(fabs(elec_angle_error()) < 0.02);           /* ~1 electrical degree */
    cfg_t saved = cfg;
    cfg_defaults();
    cfg_load();
    CHECK(cfg.cal_valid == 1 && cfg.enc_offset == saved.enc_offset);
}

static void t_not_calibrated(void)
{
    boot();
    cmd("motor torque");
    sim_run_ms(5);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_NOT_CAL));
    CHECK(!sim_pwm_enabled());
}

static void t_torque_step(void)
{
    boot();
    calibrate();
    cmd("motor torque");
    sim_run_ms(5);
    cmd("iq 5");
    sim_run_ms(1);                                  /* command takes effect at the next 2 kHz update */
    sim_run_ms(2);
    CHECK_NEAR(foc.iq, 5.0, 0.25);
    CHECK_NEAR(foc.id, 0.0, 0.25);
    CHECK_NEAR(sim_motor()->iq, 5.0, 0.3);          /* plant agrees with the estimate */
    CHECK(sim_motor()->w_m > 0);
}

static void t_speed_tracking(void)
{
    calibrated_speed_mode(3000);
    sim_run_ms(700);
    CHECK_NEAR(sim_motor()->w_m, RPM(3000), 3.0);
    CHECK_NEAR(foc.omega_m, sim_motor()->w_m, 2.0);
    cmd("rpm 1000");
    sim_run_ms(500);
    CHECK_NEAR(sim_motor()->w_m, RPM(1000), 3.0);
}

static void t_load_rejection(void)
{
    calibrated_speed_mode(3000);
    sim_run_ms(700);
    sim_set_load(0.1);
    sim_run_ms(300);
    CHECK_NEAR(sim_motor()->w_m, RPM(3000), 0.02 * RPM(3000));
    CHECK_NEAR(foc.iq, 0.1 / KT, 0.8);
}

static void t_field_weakening(void)
{
    calibrated_speed_mode(9500);
    sim_run_ms(2000);
    CHECK_NEAR(sim_motor()->w_m, RPM(9500), 5.0);
    CHECK(foc.id_ref < -5.0f);                        /* above base speed only with negative id */
    float vmax = MOD_MAX * foc.vbus / 1.7320508f;
    CHECK(sqrtf(foc.vd * foc.vd + foc.vq * foc.vq) < vmax);
    CHECK(foc.faults == 0);
}

static void t_reverse(void)
{
    calibrated_speed_mode(5000);
    sim_run_ms(800);
    cmd("rpm -5000");
    sim_run_ms(1500);
    CHECK_NEAR(sim_motor()->w_m, RPM(-5000), 5.0);
    CHECK(foc.faults == 0);
}

static void can_send_cmd(uint32_t c, const void *d, uint8_t len) { sim_can_inject((cfg.node_id << 5) | c, d, len); }

static void t_can_control(void)
{
    boot();
    calibrate();
    uint8_t st = 2;
    can_send_cmd(CMD_SET_STATE, &st, 1);
    uint32_t id;
    uint8_t d[8], len;
    for (int i = 0; i < 60; i++) {
        float w = RPM(2000);
        can_send_cmd(CMD_SET_SPEED, &w, 4);
        sim_run_ms(10);
        if (i == 49) while (sim_can_pop(&id, d, &len)) {}
    }
    CHECK(foc.state == ST_RUN);
    CHECK_NEAR(sim_motor()->w_m, RPM(2000), 3.0);

    int hb = 0, speed = 0;
    while (sim_can_pop(&id, d, &len)) {
        if (id == ((cfg.node_id << 5) | MSG_HEARTBEAT)) { hb++; CHECK(d[0] == ST_RUN && d[1] == MODE_SPEED); }
        if (id == ((cfg.node_id << 5) | MSG_IQ_SPEED)) { float w; memcpy(&w, d + 4, 4); speed = fabsf(w - RPM(2000)) < 5.0f; }
    }
    CHECK(hb > 0);
    CHECK(speed);
}

static void t_can_timeout(void)
{
    boot();
    calibrate();
    uint8_t st = 1;
    float iq = 1.0f;
    can_send_cmd(CMD_SET_STATE, &st, 1);
    can_send_cmd(CMD_SET_IQ, &iq, 4);
    sim_run_ms(100);
    CHECK(foc.state == ST_RUN);
    sim_run_ms(CAN_TIMEOUT_MS + 50);
    CHECK(foc.state == ST_IDLE);
    CHECK(!sim_pwm_enabled());
}

static void t_serial_latched(void)
{
    calibrated_speed_mode(1500);
    sim_run_ms(1500);
    CHECK(foc.state == ST_RUN);
    CHECK_NEAR(sim_motor()->w_m, RPM(1500), 3.0);
}

static void t_estop(void)
{
    calibrated_speed_mode(3000);
    sim_run_ms(500);
    sim_can_inject(0, "", 0);
    sim_run_ms(2);
    CHECK(foc.state == ST_IDLE && !sim_pwm_enabled());
}

static void t_overcurrent(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(300);
    sim_set_adc_bias(0, 35.0);              /* phase A to B current spike, sums to zero */
    sim_set_adc_bias(1, -35.0);
    sim_run_ms(2);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_OVERCURRENT));
    CHECK(!sim_pwm_enabled());
    sim_set_adc_bias(0, 0.0);
    sim_set_adc_bias(1, 0.0);
    cmd("clear");
    sim_run_ms(2);
    CHECK(foc.state == ST_IDLE && foc.faults == 0);
}

static void t_current_sensor_fault(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(300);
    sim_set_adc_bias(2, 8.0);               /* one channel drifts: currents no longer sum to zero */
    sim_run_ms(20);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_CURRENT_SUM) && !sim_pwm_enabled());
}

static void t_undervoltage(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(300);
    sim_set_vbus(6.0);
    sim_run_ms(2);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_UNDERVOLT) && !sim_pwm_enabled());
}

static void t_overtemp(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(300);
    sim_set_temp(TEMP_MAX + 5.0, 25.0);
    sim_run_ms(5);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_OVERTEMP) && !sim_pwm_enabled());
}

static void t_encoder_fault(void)
{
    calibrated_speed_mode(3000);
    sim_run_ms(600);
    sim_freeze_encoder(1);                  /* rotor locks against a frozen current vector */
    sim_run_ms(STALL_MS + 400);
    CHECK(foc.state == ST_FAULT && (foc.faults & (F_ENCODER | F_STALL)) && !sim_pwm_enabled());
}

static void t_encoder_glitch(void)
{
    calibrated_speed_mode(3000);
    sim_run_ms(600);
    sim_set_encoder_offset(sc.enc_offset + 1.5);            /* angle jumps: loose magnet, SPI corruption */
    sim_run_ms(5);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_ENCODER) && !sim_pwm_enabled());
}

static void t_stall(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(500);
    sim_lock_rotor(1);                      /* jammed mechanism */
    sim_run_ms(STALL_MS + 800);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_STALL) && !sim_pwm_enabled());
}

static void t_gate_fault(void)
{
    calibrated_speed_mode(1000);
    sim_run_ms(300);
    sim_set_gate_fault(1);
    sim_run_ms(20);
    CHECK(foc.state == ST_FAULT && (foc.faults & F_DRV) && !sim_pwm_enabled());
}

static void t_telemetry(void)
{
    calibrated_speed_mode(2000);
    sim_run_ms(700);
    drain();
    sim_run_ms(100);
    static uint8_t buf[65536];
    size_t n = sim_serial_take(buf, sizeof buf);
    static frame_t fr[512];
    int bad;
    int k = parse_frames(buf, n, fr, 512, &bad);
    int motor = 0, speed = 0, temp = 0;
    for (int i = 0; i < k; i++) {
        if (fr[i].type == TELEM_MOTOR) { motor++; CHECK(fr[i].payload[0] == ST_RUN); }
        if (fr[i].type == TELEM_SPEED) { speed++; CHECK_NEAR(frame_f32(&fr[i], 0), 2000.0, 30.0); }
        if (fr[i].type == TELEM_TEMP) temp++;
    }
    CHECK(bad == 0);
    CHECK(motor >= 100 / TELEM_MS - 1 && speed == motor);
    CHECK(temp >= 1);
}

typedef struct { const char *name; void (*fn)(void); } sil_case;

int main(int argc, char **argv)
{
    static const sil_case cases[] = {
        {"boot", t_boot}, {"calibration", t_calibration}, {"not_calibrated", t_not_calibrated},
        {"torque_step", t_torque_step}, {"speed_tracking", t_speed_tracking},
        {"load_rejection", t_load_rejection}, {"field_weakening", t_field_weakening},
        {"reverse", t_reverse}, {"can_control", t_can_control}, {"can_timeout", t_can_timeout},
        {"serial_latched", t_serial_latched}, {"estop", t_estop}, {"overcurrent", t_overcurrent},
        {"current_sensor_fault", t_current_sensor_fault},
        {"undervoltage", t_undervoltage}, {"overtemp", t_overtemp}, {"encoder_fault", t_encoder_fault},
        {"encoder_glitch", t_encoder_glitch}, {"stall", t_stall},
        {"gate_fault", t_gate_fault}, {"telemetry", t_telemetry},
    };
    if (argc < 2) {
        printf("usage: test_sil <case>\n");
        for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) printf("  %s\n", cases[i].name);
        return 2;
    }
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        if (!strcmp(argv[1], cases[i].name)) {
            cases[i].fn();
            printf("%s %s\n", test_failures ? "FAIL" : "PASS", cases[i].name);
            return test_failures ? 1 : 0;
        }
    }
    printf("unknown case %s\n", argv[1]);
    return 2;
}
