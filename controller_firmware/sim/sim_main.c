/* foc_sim: runs the firmware against the motor model in real time and serves
   the serial link (framed telemetry out, text commands in) on TCP 127.0.0.1:5599,
   the same stream the board sends over USB. Lines starting with "sim" drive the
   plant instead of the firmware. */
#include "sim.h"
#include "net.h"
#include "telem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CLIENTS 4

static int clients[MAX_CLIENTS];
static char lines[MAX_CLIENTS][128];
static int line_len[MAX_CLIENTS];

static void reply(int c, const char *msg)
{
    uint8_t f[TELEM_MAX_PAYLOAD + 7];
    size_t n = strlen(msg);
    if (n > TELEM_MAX_PAYLOAD) n = TELEM_MAX_PAYLOAD;
    net_send(c, f, (int)telem_frame(TELEM_LOG, msg, (uint16_t)n, f));
}

static void sim_command(int c, char *l)
{
    char buf[120];
    double a, b;
    if (sscanf(l, "sim load %lf", &a) == 1) { sim_set_load(a); snprintf(buf, sizeof buf, "sim: load %.3f Nm", a); }
    else if (sscanf(l, "sim vbus %lf", &a) == 1) { sim_set_vbus(a); snprintf(buf, sizeof buf, "sim: vbus %.1f V", a); }
    else if (sscanf(l, "sim bias %lf %lf", &a, &b) == 2) { sim_set_adc_bias((int)a, b); snprintf(buf, sizeof buf, "sim: phase %d bias %.2f A", (int)a, b); }
    else if (sscanf(l, "sim temp %lf", &a) == 1) { sim_set_temp(a, 25.0); snprintf(buf, sizeof buf, "sim: fet temp %.1f C", a); }
    else if (!strcmp(l, "sim freeze on") || !strcmp(l, "sim freeze off")) { sim_freeze_encoder(l[11] == 'n'); snprintf(buf, sizeof buf, "sim: encoder %s", l[11] == 'n' ? "frozen" : "live"); }
    else if (!strcmp(l, "sim lock on") || !strcmp(l, "sim lock off")) { sim_lock_rotor(l[9] == 'n'); snprintf(buf, sizeof buf, "sim: rotor %s", l[9] == 'n' ? "locked" : "free"); }
    else if (!strcmp(l, "sim gate on") || !strcmp(l, "sim gate off")) { sim_set_gate_fault(l[9] == 'n'); snprintf(buf, sizeof buf, "sim: gate fault %s", l[9] == 'n' ? "on" : "off"); }
    else if (!strcmp(l, "sim status")) {
        motor_t *m = sim_motor();
        snprintf(buf, sizeof buf, "sim: %.0f rpm, id %.2f A, iq %.2f A, load %.3f Nm, vbus %.1f V",
                 m->w_m * 9.5492966, m->id, m->iq, m->t_load, m->vbus);
    } else snprintf(buf, sizeof buf, "sim: load <Nm> | vbus <V> | bias <ph> <A> | temp <C> | freeze on|off | lock on|off | gate on|off | status");
    reply(c, buf);
}

static void handle_line(int c, char *l)
{
    if (!strncmp(l, "sim", 3) && (l[3] == ' ' || l[3] == 0)) { sim_command(c, l); return; }
    sim_serial_inject(l);
    sim_serial_inject("\n");
}

int main(int argc, char **argv)
{
    int port = 5599, fast = 0;
    sim_config_t cfg;
    sim_default_config(&cfg);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--vbus") && i + 1 < argc) cfg.vbus = atof(argv[++i]);
        else if (!strcmp(argv[i], "--fast")) fast = 1;
        else { printf("usage: foc_sim [--port 5599] [--vbus 24] [--fast]\n"); return 1; }
    }

    int ls = net_listen(port);
    if (ls < 0) { fprintf(stderr, "foc_sim: cannot listen on port %d\n", port); return 1; }
    for (int i = 0; i < MAX_CLIENTS; i++) clients[i] = -1;
    printf("foc_sim: serial link on tcp://127.0.0.1:%d\n", port);
    fflush(stdout);

    sim_init(&cfg);
    uint64_t t0 = net_now_ms();
    for (;;) {
        sim_run_ms(1);

        int c = net_accept(ls);
        if (c >= 0) {
            int slot = -1;
            for (int i = 0; i < MAX_CLIENTS; i++) if (clients[i] < 0) { slot = i; break; }
            if (slot < 0) net_close(c);
            else { clients[slot] = c; line_len[slot] = 0; printf("foc_sim: client connected\n"); fflush(stdout); }
        }

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i] < 0) continue;
            char buf[256];
            int n = net_recv(clients[i], buf, sizeof buf);
            if (n < 0) { net_close(clients[i]); clients[i] = -1; continue; }
            for (int k = 0; k < n; k++) {
                if (buf[k] == '\n' || buf[k] == '\r') {
                    lines[i][line_len[i]] = 0;
                    if (line_len[i]) handle_line(clients[i], lines[i]);
                    line_len[i] = 0;
                } else if (line_len[i] < (int)sizeof lines[i] - 1) {
                    lines[i][line_len[i]++] = buf[k];
                }
            }
        }

        uint8_t out[4096];
        size_t n;
        while ((n = sim_serial_take(out, sizeof out)) > 0)
            for (int i = 0; i < MAX_CLIENTS; i++)
                if (clients[i] >= 0 && net_send(clients[i], out, (int)n) < 0) { net_close(clients[i]); clients[i] = -1; }

        if (!fast) {
            while ((int64_t)(sim_millis() - (net_now_ms() - t0)) > 0) net_sleep_ms(1);
        }
    }
}
