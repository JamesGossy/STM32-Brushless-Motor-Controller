/*
 * cmd.h - text command console on the serial link (USB CDC on the board,
 * TCP in the simulator). One command per line, replies come back as
 * telemetry LOG frames. Type `help` for the list.
 */
#pragma once

void cmd_poll(void);
void cmd_exec(char *line);
