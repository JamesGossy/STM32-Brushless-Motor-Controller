#include "cmd.h"
#include "app.h"
#include "foc.h"
#include "cfg.h"
#include "telem.h"
#include "hal.h"
#include <stdlib.h>
#include <string.h>

#define RADS_PER_RPM 0.10471976f

static const char *state_names[] = {"BOOT", "IDLE", "CAL", "RUN", "FAULT"};

static char *next_tok(char **s)
{
    while (**s == ' ' || **s == '\t') (*s)++;
    if (!**s) return 0;
    char *t = *s;
    while (**s && **s != ' ' && **s != '\t') (*s)++;
    if (**s) *(*s)++ = 0;
    return t;
}

static int arg_f(char **s, float *v)
{
    char *t = next_tok(s), *end;
    if (!t) return 0;
    *v = strtof(t, &end);
    return end != t;
}

void cmd_exec(char *line)
{
    char *s = line, *c = next_tok(&s);
    float a, b;
    if (!c) return;

    if (!strcmp(c, "motor")) {
        char *m = next_tok(&s);
        if (!m || !strcmp(m, "off") || !strcmp(m, "stop") || !strcmp(m, "disable")) app_set_state(0, SRC_SERIAL);
        else if (!strcmp(m, "torque")) app_set_state(1, SRC_SERIAL);
        else if (!strcmp(m, "speed") || !strcmp(m, "enable")) app_set_state(2, SRC_SERIAL);
        else telem_log("? motor off|torque|speed");
    } else if (!strcmp(c, "calibrate")) {
        app_set_state(3, SRC_SERIAL);
    } else if (!strcmp(c, "clear")) {
        app_clear_faults();
    } else if (!strcmp(c, "iq")) {
        if (arg_f(&s, &a)) app_set_iq(a, SRC_SERIAL); else telem_log("? iq <A>");
    } else if (!strcmp(c, "rpm")) {
        if (arg_f(&s, &a)) app_set_speed(a * RADS_PER_RPM, SRC_SERIAL); else telem_log("? rpm <rpm>");
    } else if (!strcmp(c, "limits")) {
        if (arg_f(&s, &a) && arg_f(&s, &b)) app_set_limits(a, b * RADS_PER_RPM);
        else telem_log("? limits <A> <rpm>");
    } else if (!strcmp(c, "node")) {
        if (arg_f(&s, &a) && a >= 1 && a <= 63) { cfg.node_id = (uint32_t)a; app_save_req = 1; }
        else telem_log("? node <1..63>");
    } else if (!strcmp(c, "telem")) {
        char *m = next_tok(&s);
        telem_enabled = !(m && !strcmp(m, "off"));
    } else if (!strcmp(c, "status")) {
        telem_log("state=%s mode=%d faults=0x%04x cal=%d vbus=%.1fV rpm=%.0f iq=%.2fA id=%.2fA",
                  state_names[foc.state], foc.mode, foc.faults, (int)cfg.cal_valid,
                  (double)foc.vbus, (double)(foc.omega_m * 9.5492966f), (double)foc.iq, (double)foc.id);
    } else if (!strcmp(c, "help")) {
        telem_log("motor off|torque|speed, calibrate, clear, iq <A>, rpm <rpm>, limits <A> <rpm>, node <id>, telem on|off, status");
    } else {
        telem_log("unknown command: %s", c);
    }
}

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
