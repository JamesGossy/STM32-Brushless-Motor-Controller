#pragma once

/* Text commands over the serial link (USB CDC / simulator TCP), one per line:
     motor off | motor torque | motor speed | calibrate | clear
     iq <A> | rpm <rpm> | limits <A> <rpm> | node <id>
     telem on | telem off | status | help                                   */
void cmd_poll(void);
void cmd_exec(char *line);
