/*
 * cmd.c - parses serial console commands.
 *
 *   motor <rpm>          speed mode at <rpm> (e.g. "motor 3000")
 *   motor speed|torque   switch mode
 *   motor off            stop
 *   rpm <rpm>            speed setpoint
 *   iq <A>               torque (q-axis current) setpoint
 *   calibrate            run the sensor calibration
 *   clear                clear faults
 *   limits <A> <rpm>     runtime current and speed limits
 *   node <id>            CAN node id (saved)
 *   telem on|off         serial telemetry stream
 *   status, help
 */
#include "cmd.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "telem.h"
#include "hal.h"
#include <stdlib.h>
#include <string.h>

#define RAD_S_PER_RPM 0.10471976f
#define RPM_PER_RAD_S 9.5492966f

static const char *state_names[] = {"BOOT", "IDLE", "CALIBRATING", "RUN", "FAULT"};

/* Split off the next space-separated word, or NULL at the end of the line. */
static char *next_word(char **s)
{
    while (**s == ' ' || **s == '\t') (*s)++;
    if (!**s) return NULL;
    char *word = *s;
    while (**s && **s != ' ' && **s != '\t') (*s)++;
    if (**s) *(*s)++ = 0;
    return word;
}

/* Parse the next word as a number. Returns 0 if missing or not a number. */
static int next_number(char **s, float *v)
{
    char *word = next_word(s), *end;
    if (!word) return 0;
    *v = strtof(word, &end);
    return end != word && *end == 0;
}

/* Request a state and report the result. Returns 1 if accepted. */
static int request(uint8_t run, const char *ok_msg)
{
    const char *err = app_request(run, SRC_SERIAL);
    telem_log("%s", err ? err : ok_msg);
    return err == NULL;
}

static void cmd_motor(char *args)
{
    float rpm;
    char *save = args;
    if (next_number(&args, &rpm)) {
        if (request(RUN_SPEED, "speed mode")) {
            app_set_speed(rpm * RAD_S_PER_RPM, SRC_SERIAL);
            telem_log("speed %.0f rpm", (double)rpm);
        }
        return;
    }

    args = save;
    char *what = next_word(&args);
    if (!what || !strcmp(what, "off") || !strcmp(what, "stop"))
        request(RUN_IDLE, "motor off");
    else if (!strcmp(what, "speed") || !strcmp(what, "on"))
        request(RUN_SPEED, "speed mode: set speed with 'rpm <rpm>'");
    else if (!strcmp(what, "torque"))
        request(RUN_TORQUE, "torque mode: set current with 'iq <A>'");
    else
        telem_log("usage: motor <rpm> | motor speed | motor torque | motor off");
}

static void cmd_status(void)
{
    telem_log("state=%s mode=%s cal=%s vbus=%.1fV speed=%.0frpm iq=%.2fA id=%.2fA faults=%s",
              state_names[foc.state],
              foc.mode == MODE_SPEED ? "speed" : foc.mode == MODE_TORQUE ? "torque" : "-",
              cfg.cal_valid ? "yes" : "no", (double)foc.vbus,
              (double)(foc.omega_m * RPM_PER_RAD_S), (double)foc.iq, (double)foc.id,
              app_fault_text(foc.faults));
}

/* Run one command line. */
void cmd_exec(char *line)
{
    char *args = line;
    char *cmd = next_word(&args);
    float a, b;
    if (!cmd) return;

    if (!strcmp(cmd, "motor")) {
        cmd_motor(args);
    } else if (!strcmp(cmd, "rpm")) {
        if (!next_number(&args, &a)) { telem_log("usage: rpm <rpm>"); return; }
        app_set_speed(a * RAD_S_PER_RPM, SRC_SERIAL);
        if (foc.state != ST_RUN || foc.mode != MODE_SPEED) telem_log("setpoint stored; start with 'motor speed'");
    } else if (!strcmp(cmd, "iq")) {
        if (!next_number(&args, &a)) { telem_log("usage: iq <A>"); return; }
        app_set_iq(a, SRC_SERIAL);
        if (foc.state != ST_RUN || foc.mode != MODE_TORQUE) telem_log("setpoint stored; start with 'motor torque'");
    } else if (!strcmp(cmd, "calibrate")) {
        request(RUN_CALIBRATE, "calibrating: keep the motor free to spin (about 5 s)");
    } else if (!strcmp(cmd, "clear")) {
        app_clear_faults();
        telem_log("faults: %s", app_fault_text(foc.faults));
    } else if (!strcmp(cmd, "limits")) {
        if (!next_number(&args, &a) || !next_number(&args, &b)) { telem_log("usage: limits <A> <rpm>"); return; }
        app_set_limits(a, b * RAD_S_PER_RPM);
    } else if (!strcmp(cmd, "node")) {
        if (!next_number(&args, &a) || a < 1 || a > 63) { telem_log("usage: node <1..63>"); return; }
        cfg.node_id = (uint32_t)a;
        app_save_req = 1;
    } else if (!strcmp(cmd, "telem")) {
        char *onoff = next_word(&args);
        telem_enabled = !(onoff && !strcmp(onoff, "off"));
    } else if (!strcmp(cmd, "status")) {
        cmd_status();
    } else if (!strcmp(cmd, "help")) {
        telem_log("motor <rpm> | motor speed|torque|off | rpm <rpm> | iq <A> | calibrate | clear | "
                  "limits <A> <rpm> | node <id> | telem on|off | status");
    } else {
        telem_log("unknown command '%s' (try 'help')", cmd);
    }
}

/* Collect serial bytes into lines and run each complete line. */
void cmd_poll(void)
{
    static char line[96];
    static unsigned len;
    uint8_t buf[32];
    size_t n;

    while ((n = hal_serial_read(buf, sizeof buf)) > 0) {
        for (size_t i = 0; i < n; i++) {
            char ch = (char)buf[i];
            if (ch == '\n' || ch == '\r') {
                line[len] = 0;
                if (len) cmd_exec(line);
                len = 0;
            } else if (len < sizeof line - 1) {
                line[len++] = ch;
            }
        }
    }
}
