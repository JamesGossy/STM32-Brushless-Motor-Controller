/*
 * net.c - sockets, sleep and a millisecond clock for foc_sim.
 */
#include "net.h"

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>

static void net_start(void)
{
    static int started;
    if (!started) {
        WSADATA w;
        WSAStartup(MAKEWORD(2, 2), &w);
        started = 1;
    }
}
static void set_nonblocking(SOCKET s) { u_long on = 1; ioctlsocket(s, FIONBIO, &on); }
static int would_block(void) { return WSAGetLastError() == WSAEWOULDBLOCK; }
void net_close(int s) { closesocket((SOCKET)s); }
void net_sleep_ms(int ms) { Sleep((DWORD)ms); }
uint64_t net_now_ms(void) { return GetTickCount64(); }

#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static void net_start(void) { signal(SIGPIPE, SIG_IGN); }   /* a closed client must not kill us */
static void set_nonblocking(int s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
static int would_block(void) { return errno == EAGAIN || errno == EWOULDBLOCK; }
void net_close(int s) { close(s); }

void net_sleep_ms(int ms)
{
    struct timespec t = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&t, 0);
}

uint64_t net_now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)t.tv_nsec / 1000000u;
}
#endif

int net_listen(int port)
{
    net_start();
    int s = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    int on = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof on);

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);     /* local connections only */
    addr.sin_port = htons((unsigned short)port);
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(s, 4) < 0) {
        net_close(s);
        return -1;
    }
    set_nonblocking(s);
    return s;
}

int net_accept(int listener)
{
    int s = (int)accept(listener, 0, 0);
    if (s < 0) return -1;
    set_nonblocking(s);
    int on = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof on);
    return s;
}

int net_recv(int s, char *buf, int max)
{
    int n = (int)recv(s, buf, max, 0);
    if (n > 0) return n;
    if (n < 0 && would_block()) return 0;
    return -1;
}

/* Send everything, waiting up to 50 ms for a slow client before giving up. */
int net_send(int s, const void *buf, int len)
{
    const char *p = buf;
    uint64_t start = net_now_ms();
    while (len > 0) {
        int n = (int)send(s, p, len, 0);
        if (n > 0) {
            p += n;
            len -= n;
        } else if (n < 0 && would_block() && net_now_ms() - start < 50) {
            net_sleep_ms(1);
        } else {
            return -1;
        }
    }
    return 0;
}
