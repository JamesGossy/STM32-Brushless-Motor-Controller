/*
 * net.h - tiny non-blocking TCP wrapper that works on Windows and POSIX.
 */
#pragma once
#include <stdint.h>

int net_listen(int port);                       /* listening socket on 127.0.0.1, or -1 */
int net_accept(int listener);                   /* new client, or -1 if none waiting */
int net_recv(int sock, char *buf, int max);     /* >0 bytes, 0 nothing yet, -1 closed */
int net_send(int sock, const void *buf, int len);   /* 0 ok, -1 error */
void net_close(int sock);
void net_sleep_ms(int ms);
uint64_t net_now_ms(void);
