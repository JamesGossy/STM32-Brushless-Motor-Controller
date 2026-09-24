#include "net.h"

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
typedef int socklen_t;
static int started;
static void start(void) { if (!started) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); started = 1; } }
static void nonblock(SOCKET s) { u_long on = 1; ioctlsocket(s, FIONBIO, &on); }
static int would_block(void) { return WSAGetLastError() == WSAEWOULDBLOCK; }
void net_close(int s) { closesocket((SOCKET)s); }
void net_sleep_ms(int ms) { Sleep((DWORD)ms); }
uint64_t net_now_ms(void) { return GetTickCount64(); }
#define SEND_FLAGS 0
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
static void start(void) { signal(SIGPIPE, SIG_IGN); }
static void nonblock(int s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
static int would_block(void) { return errno == EAGAIN || errno == EWOULDBLOCK; }
void net_close(int s) { close(s); }
void net_sleep_ms(int ms) { struct timespec t = {ms / 1000, (ms % 1000) * 1000000L}; nanosleep(&t, 0); }
uint64_t net_now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)t.tv_nsec / 1000000u;
}
#define SEND_FLAGS 0
#endif

int net_listen(int port)
{
    start();
    int s = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    int on = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof on);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons((unsigned short)port);
    if (bind(s, (struct sockaddr *)&a, sizeof a) < 0 || listen(s, 4) < 0) { net_close(s); return -1; }
    nonblock(s);
    return s;
}

int net_accept(int ls)
{
    int s = (int)accept(ls, 0, 0);
    if (s < 0) return -1;
    nonblock(s);
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

int net_send(int s, const void *buf, int n)
{
    const char *p = buf;
    uint64_t t0 = net_now_ms();
    while (n > 0) {
        int k = (int)send(s, p, n, SEND_FLAGS);
        if (k > 0) { p += k; n -= k; continue; }
        if (k < 0 && would_block() && net_now_ms() - t0 < 50) { net_sleep_ms(1); continue; }
        return -1;
    }
    return 0;
}
