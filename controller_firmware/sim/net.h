#pragma once
#include <stdint.h>

/* minimal non-blocking TCP for Windows and POSIX */
int net_listen(int port);
int net_accept(int ls);                         /* -1 if none */
int net_recv(int s, char *buf, int max);        /* >0 bytes, 0 nothing, -1 closed */
int net_send(int s, const void *buf, int n);    /* -1 on error */
void net_close(int s);
void net_sleep_ms(int ms);
uint64_t net_now_ms(void);
