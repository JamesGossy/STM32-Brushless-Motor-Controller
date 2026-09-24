/*
 * sim_main.c - foc_sim, the simulated motor controller.
 *
 * Runs the firmware and motor model in real time and serves the serial link
 * on tcp://127.0.0.1:5599: telemetry frames out, text commands in, exactly
 * like the board's USB port. The dashboard connects here. Lines starting
 * with "sim" change the simulated world instead of going to the firmware.
 *
 *   foc_sim [--port 5599] [--vbus 24] [--fast]
 */
#include "sim.h"
#include "net.h"
#include "telem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CLIENTS 4
#define LINE_MAX_LEN 128

typedef struct {
    int sock;               /* -1 when the slot is free */
    char line[LINE_MAX_LEN];
    int len;
} client_t;

static client_t clients[MAX_CLIENTS];

/* Send a text reply to one client as a LOG frame. */
static void reply(int sock, const char *text)
{
    uint8_t frame[TELEM_MAX_PAYLOAD + 7];
    size_t n = strlen(text);
    if (n > TELEM_MAX_PAYLOAD) n = TELEM_MAX_PAYLOAD;
    net_send(sock, frame, (int)telem_frame(TELEM_LOG, text, (uint16_t)n, frame));
}

/* Is `line` exactly "sim <what> on" or "sim <what> off"? Sets *on. */
static int on_off(const char *line, const char *what, int *on)
{
    char on_cmd[40], off_cmd[40];
    snprintf(on_cmd, sizeof on_cmd, "sim %s on", what);
    snprintf(off_cmd, sizeof off_cmd, "sim %s off", what);
    if (!strcmp(line, on_cmd)) { *on = 1; return 1; }
    if (!strcmp(line, off_cmd)) { *on = 0; return 1; }
    return 0;
}

/* Handle a "sim ..." command. */
static void sim_command(int sock, const char *line)
{
    char msg[120];
    double a, b;
    int on;

    if (sscanf(line, "sim load %lf", &a) == 1) {
        sim_set_load(a);
        snprintf(msg, sizeof msg, "sim: load %.3f Nm", a);
    } else if (sscanf(line, "sim vbus %lf", &a) == 1) {
        sim_set_vbus(a);
        snprintf(msg, sizeof msg, "sim: vbus %.1f V", a);
    } else if (sscanf(line, "sim bias %lf %lf", &a, &b) == 2) {
        sim_set_adc_bias((int)a, b);
        snprintf(msg, sizeof msg, "sim: phase %d sensor bias %.2f A", (int)a, b);
    } else if (sscanf(line, "sim temp %lf", &a) == 1) {
        sim_set_temp(a, 25.0);
        snprintf(msg, sizeof msg, "sim: FET temperature %.1f C", a);
    } else if (on_off(line, "lock", &on)) {
        sim_lock_rotor(on);
        snprintf(msg, sizeof msg, "sim: rotor %s", on ? "locked" : "free");
    } else if (on_off(line, "freeze", &on)) {
        sim_freeze_encoder(on);
        snprintf(msg, sizeof msg, "sim: encoder %s", on ? "frozen" : "live");
    } else if (on_off(line, "gate", &on)) {
        sim_set_gate_fault(on);
        snprintf(msg, sizeof msg, "sim: gate driver fault %s", on ? "on" : "off");
    } else if (!strcmp(line, "sim status")) {
        motor_t *m = sim_motor();
        snprintf(msg, sizeof msg, "sim: %.0f rpm, id %.2f A, iq %.2f A, load %.3f Nm, vbus %.1f V",
                 m->w_m * 9.5492966, m->id, m->iq, m->t_load, m->vbus);
    } else {
        snprintf(msg, sizeof msg,
                 "sim: load <Nm> | vbus <V> | bias <phase> <A> | temp <C> | lock on|off | "
                 "freeze on|off | gate on|off | status");
    }
    reply(sock, msg);
}

/* A complete line from a client: simulator command or firmware command. */
static void handle_line(int sock, const char *line)
{
    if (!strncmp(line, "sim", 3) && (line[3] == ' ' || line[3] == 0)) {
        sim_command(sock, line);
    } else {
        sim_serial_inject(line);
        sim_serial_inject("\n");
    }
}

/* Accept new clients and read input from the connected ones. */
static void service_clients(int listener)
{
    int s = net_accept(listener);
    if (s >= 0) {
        int slot;
        for (slot = 0; slot < MAX_CLIENTS && clients[slot].sock >= 0; slot++) {}
        if (slot == MAX_CLIENTS) {
            net_close(s);
        } else {
            clients[slot].sock = s;
            clients[slot].len = 0;
            printf("foc_sim: client connected\n");
            fflush(stdout);
        }
    }

    for (int i = 0; i < MAX_CLIENTS; i++) {
        client_t *c = &clients[i];
        if (c->sock < 0) continue;

        char buf[256];
        int n = net_recv(c->sock, buf, sizeof buf);
        if (n < 0) {
            net_close(c->sock);
            c->sock = -1;
            continue;
        }
        for (int k = 0; k < n; k++) {
            if (buf[k] == '\n' || buf[k] == '\r') {
                c->line[c->len] = 0;
                if (c->len) handle_line(c->sock, c->line);
                c->len = 0;
            } else if (c->len < LINE_MAX_LEN - 1) {
                c->line[c->len++] = buf[k];
            }
        }
    }
}

/* Forward the firmware's serial output to every client. */
static void forward_output(void)
{
    uint8_t out[4096];
    size_t n;
    while ((n = sim_serial_take(out, sizeof out)) > 0) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].sock >= 0 && net_send(clients[i].sock, out, (int)n) < 0) {
                net_close(clients[i].sock);
                clients[i].sock = -1;
            }
        }
    }
}

int main(int argc, char **argv)
{
    int port = 5599, fast = 0;
    sim_config_t config;
    sim_default_config(&config);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--vbus") && i + 1 < argc) config.vbus = atof(argv[++i]);
        else if (!strcmp(argv[i], "--fast")) fast = 1;     /* don't pace to real time (tests) */
        else {
            printf("usage: foc_sim [--port 5599] [--vbus 24] [--fast]\n");
            return 1;
        }
    }

    int listener = net_listen(port);
    if (listener < 0) {
        fprintf(stderr, "foc_sim: cannot listen on port %d\n", port);
        return 1;
    }
    for (int i = 0; i < MAX_CLIENTS; i++) clients[i].sock = -1;
    printf("foc_sim: serial link on tcp://127.0.0.1:%d\n", port);
    fflush(stdout);

    sim_init(&config);
    uint64_t start = net_now_ms();

    for (;;) {
        sim_run_ms(1);
        service_clients(listener);
        forward_output();

        /* stay in step with the wall clock */
        while (!fast && (int64_t)(sim_millis() - (net_now_ms() - start)) > 0)
            net_sleep_ms(1);
    }
}
