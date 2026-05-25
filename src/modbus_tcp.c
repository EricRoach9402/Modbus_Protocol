/**
 * @file modbus_tcp.c
 * @brief Standalone Modbus TCP (CM_MODBUS-independent).
 */

#define _GNU_SOURCE

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdarg.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "modbus_tcp.h"

static void mb_tcp_logv(mb_tcp_ctx_t *ctx, mb_tcp_log_level_t level, const char *fmt, ...)
{
    if (!ctx || !ctx->cfg.logv) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    ctx->cfg.logv(ctx->cfg.log_userdata, level, fmt, ap);
    va_end(ap);
}

static void mb_tcp_link_call(mb_tcp_ctx_t *ctx, int fd, int connected)
{
    if (ctx && ctx->cfg.on_link) {
        ctx->cfg.on_link(ctx->cfg.link_userdata, fd, connected);
    }
}

static void *mb_tcp_server_thread(void *arg);
static void *mb_tcp_client_thread(void *arg);

static int mb_tcp_listen_init(mb_tcp_ctx_t *ctx)
{
    struct sockaddr_in server_addr = {0};
    int opt = 1;

    ctx->listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->listen_sock < 0) {
        mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "socket: %s", strerror(errno));
        return -1;
    }

    if (setsockopt(ctx->listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "SO_REUSEADDR: %s", strerror(errno));
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(ctx->cfg.port);

    if (bind(ctx->listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "bind: %s", strerror(errno));
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
        return -1;
    }

    if (listen(ctx->listen_sock, MB_TCP_MAX_CLIENTS) < 0) {
        mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "listen: %s", strerror(errno));
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
        return -1;
    }

    mb_tcp_logv(ctx, MB_TCP_LOG_DEBUG, "listening on port %u", (unsigned)ctx->cfg.port);
    return 0;
}

int mb_tcp_start(mb_tcp_ctx_t *ctx, const mb_tcp_config_t *cfg)
{
    if (!ctx || !cfg) {
        return -1;
    }
    if (cfg->mode == MB_TCP_MODE_CONNECT_CLIENT &&
        (cfg->remote_host == NULL || cfg->remote_host[0] == '\0')) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->listen_sock = -1;
    ctx->stream_sock = -1;
    ctx->cfg = *cfg;
    if (ctx->cfg.connect_timeout_sec == 0) {
        ctx->cfg.connect_timeout_sec = 5;
    }
    if (ctx->cfg.connect_retry_sec == 0) {
        ctx->cfg.connect_retry_sec = 3;
    }
    ctx->keep_running = 1;
    ctx->tid = 0;

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        return -1;
    }

    if (ctx->cfg.on_init) {
        ctx->cfg.on_init(ctx->cfg.userdata);
    }

    if (ctx->cfg.mode == MB_TCP_MODE_LISTEN_SERVER) {
        if (mb_tcp_listen_init(ctx) < 0) {
            pthread_mutex_destroy(&ctx->lock);
            return -1;
        }
        if (pthread_create(&ctx->thread, NULL, mb_tcp_server_thread, ctx) != 0) {
            close(ctx->listen_sock);
            ctx->listen_sock = -1;
            pthread_mutex_destroy(&ctx->lock);
            return -1;
        }
    } else {
        if (pthread_create(&ctx->thread, NULL, mb_tcp_client_thread, ctx) != 0) {
            pthread_mutex_destroy(&ctx->lock);
            return -1;
        }
    }

    return 0;
}

void mb_tcp_stop(mb_tcp_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->keep_running = 0;

    if (ctx->listen_sock >= 0) {
        shutdown(ctx->listen_sock, SHUT_RDWR);
    }
    for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
        if (ctx->client_fds[i] > 0) {
            shutdown(ctx->client_fds[i], SHUT_RDWR);
        }
    }
    if (ctx->stream_sock >= 0) {
        shutdown(ctx->stream_sock, SHUT_RDWR);
    }

    pthread_join(ctx->thread, NULL);

    if (ctx->listen_sock >= 0) {
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
    }
    for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
        if (ctx->client_fds[i] > 0) {
            close(ctx->client_fds[i]);
            ctx->client_fds[i] = 0;
        }
    }
    if (ctx->stream_sock >= 0) {
        close(ctx->stream_sock);
        ctx->stream_sock = -1;
    }

    pthread_mutex_destroy(&ctx->lock);
}

static void *mb_tcp_server_thread(void *arg)
{
    mb_tcp_ctx_t *ctx = (mb_tcp_ctx_t *)arg;
    struct sockaddr_in client_addr = {0};
    fd_set readfds;
    int max_sd, sd, new_socket, activity;
    int connection_state = 0;
    socklen_t addrlen = sizeof(client_addr);
    struct timeval timeout;

    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    while (ctx->keep_running) {
        FD_ZERO(&readfds);
        FD_SET(ctx->listen_sock, &readfds);
        max_sd = ctx->listen_sock;

        for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
            sd = ctx->client_fds[i];
            if (sd > 0) {
                FD_SET(sd, &readfds);
                if (sd > max_sd) {
                    max_sd = sd;
                }
            }
        }

        activity = select(max_sd + 1, &readfds, NULL, NULL, &timeout);

        if ((activity < 0) && (errno != EINTR)) {
            continue;
        }

        if (activity == 0) {
            for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
                sd = ctx->client_fds[i];
                if (sd > 0) {
                    char buffer[1];
                    int check = recv(sd, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT);
                    if (check == 0) {
                        mb_tcp_logv(ctx, MB_TCP_LOG_WARN, "client disconnected fd %d", sd);
                        close(sd);
                        ctx->client_fds[i] = 0;
                        connection_state = 0;
                        mb_tcp_link_call(ctx, sd, 0);
                        if (ctx->cfg.on_error) {
                            ctx->cfg.on_error(ctx->cfg.userdata, connection_state);
                        }
                    }
                }
            }
            continue;
        }

        if (ctx->listen_sock >= 0 && FD_ISSET(ctx->listen_sock, &readfds)) {
            new_socket = accept(ctx->listen_sock, (struct sockaddr *)&client_addr, &addrlen);
            if (new_socket < 0) {
                if (ctx->keep_running) {
                    mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "accept: %s", strerror(errno));
                }
                continue;
            }

            mb_tcp_logv(ctx, MB_TCP_LOG_DEBUG, "new client fd %d from %s:%u", new_socket,
                        inet_ntoa(client_addr.sin_addr), (unsigned)ntohs(client_addr.sin_port));

            int keep_alive = 1;
            int keep_idle = 5;
            int keep_intvl = 5;
            int keep_cnt = 1;
            setsockopt(new_socket, SOL_SOCKET, SO_KEEPALIVE, &keep_alive, sizeof(keep_alive));
            setsockopt(new_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(keep_idle));
            setsockopt(new_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keep_intvl, sizeof(keep_intvl));
            setsockopt(new_socket, IPPROTO_TCP, TCP_KEEPCNT, &keep_cnt, sizeof(keep_cnt));

            int placed = 0;
            for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
                if (ctx->client_fds[i] == 0) {
                    ctx->client_fds[i] = new_socket;
                    placed = 1;
                    break;
                }
            }
            if (!placed) {
                mb_tcp_logv(ctx, MB_TCP_LOG_WARN, "max clients reached, rejecting fd %d", new_socket);
                close(new_socket);
                continue;
            }

            connection_state = 1;
            mb_tcp_link_call(ctx, new_socket, 1);
            if (ctx->cfg.on_error) {
                ctx->cfg.on_error(ctx->cfg.userdata, connection_state);
            }
        }

        for (int i = 0; i < MB_TCP_MAX_CLIENTS; i++) {
            sd = ctx->client_fds[i];
            if (sd > 0 && FD_ISSET(sd, &readfds)) {
                if (ctx->cfg.on_process) {
                    if (ctx->cfg.on_process(ctx->cfg.userdata, sd) < 0) {
                        close(sd);
                        ctx->client_fds[i] = 0;
                        connection_state = 0;
                        mb_tcp_link_call(ctx, sd, 0);
                        if (ctx->cfg.on_error) {
                            ctx->cfg.on_error(ctx->cfg.userdata, connection_state);
                        }
                    }
                }
            }
        }
    }

    mb_tcp_logv(ctx, MB_TCP_LOG_DEBUG, "server thread exit");
    return NULL;
}

static void *mb_tcp_client_thread(void *arg)
{
    mb_tcp_ctx_t *ctx = (mb_tcp_ctx_t *)arg;
    int sock = -1;
    int connected = 0;

    while (ctx->keep_running) {
        if (!connected) {
            if (mb_tcp_connect(&sock, ctx->cfg.bind_iface, ctx->cfg.remote_host, ctx->cfg.port,
                               ctx->cfg.connect_timeout_sec) == 0) {
                mb_tcp_logv(ctx, MB_TCP_LOG_DEBUG, "connected to %s:%u", ctx->cfg.remote_host,
                            (unsigned)ctx->cfg.port);
                ctx->stream_sock = sock;
                connected = 1;
                mb_tcp_link_call(ctx, sock, 1);
                if (ctx->cfg.on_error) {
                    ctx->cfg.on_error(ctx->cfg.userdata, 1);
                }
            } else {
                mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "connect %s:%u failed, retry in %us",
                            ctx->cfg.remote_host, (unsigned)ctx->cfg.port,
                            ctx->cfg.connect_retry_sec);
                if (ctx->cfg.on_error) {
                    ctx->cfg.on_error(ctx->cfg.userdata, 0);
                }
                for (unsigned sec = 0; sec < ctx->cfg.connect_retry_sec && ctx->keep_running;
                     sec++) {
                    sleep(1);
                }
                continue;
            }
        }

        if (ctx->cfg.on_process) {
            if (ctx->cfg.on_process(ctx->cfg.userdata, sock) < 0) {
                mb_tcp_logv(ctx, MB_TCP_LOG_ERROR, "on_process failed");
                mb_tcp_link_call(ctx, sock, 0);
                close(sock);
                sock = -1;
                ctx->stream_sock = -1;
                connected = 0;
                if (ctx->cfg.on_error) {
                    ctx->cfg.on_error(ctx->cfg.userdata, 0);
                }
                continue;
            }
        }

        sleep(1);
    }

    if (sock >= 0) {
        mb_tcp_link_call(ctx, sock, 0);
    }
    mb_tcp_logv(ctx, MB_TCP_LOG_DEBUG, "client thread exit");
    return NULL;
}

int mb_tcp_connect(int *sock_out, const char *bind_iface, const char *remote_host, uint16_t port,
                   unsigned int timeout_sec)
{
    int yes = 1;
    struct sockaddr_in server_addr;

    if (!sock_out || !remote_host || remote_host[0] == '\0' || port == 0) {
        return -EINVAL;
    }

    if (timeout_sec == 0) {
        timeout_sec = 5;
    }

    *sock_out = socket(AF_INET, SOCK_STREAM, 0);
    if (*sock_out < 0) {
        return -1;
    }

    struct timeval tmo;
    tmo.tv_sec = (time_t)timeout_sec;
    tmo.tv_usec = 0;
    if (setsockopt(*sock_out, SOL_SOCKET, SO_SNDTIMEO, &tmo, sizeof(tmo)) < 0) {
        close(*sock_out);
        *sock_out = -1;
        return -1;
    }
    if (setsockopt(*sock_out, SOL_SOCKET, SO_RCVTIMEO, &tmo, sizeof(tmo)) < 0) {
        close(*sock_out);
        *sock_out = -1;
        return -1;
    }

    if (bind_iface && bind_iface[0] != '\0') {
        if (setsockopt(*sock_out, SOL_SOCKET, SO_BINDTODEVICE, bind_iface, strlen(bind_iface)) <
            0) {
            close(*sock_out);
            *sock_out = -1;
            return -1;
        }
    }

    if (setsockopt(*sock_out, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
        close(*sock_out);
        *sock_out = -1;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, remote_host, &server_addr.sin_addr) <= 0) {
        close(*sock_out);
        *sock_out = -1;
        return -1;
    }

    int flags = fcntl(*sock_out, F_GETFL, 0);
    fcntl(*sock_out, F_SETFL, flags | O_NONBLOCK);

    if (connect(*sock_out, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        if (errno == EINPROGRESS) {
            fd_set writefds;
            struct timeval tv;

            FD_ZERO(&writefds);
            FD_SET(*sock_out, &writefds);
            tv.tv_sec = (time_t)timeout_sec;
            tv.tv_usec = 0;

            if (select(*sock_out + 1, NULL, &writefds, NULL, &tv) > 0) {
                int so_error = 0;
                socklen_t len = sizeof(so_error);
                getsockopt(*sock_out, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0) {
                    fcntl(*sock_out, F_SETFL, flags);
                    return 0;
                }
            }
        }
        close(*sock_out);
        *sock_out = -1;
        return -1;
    }

    fcntl(*sock_out, F_SETFL, flags);
    return 0;
}

void mb_tcp_close(int *fd)
{
    if (fd && *fd >= 0) {
        shutdown(*fd, SHUT_RDWR);
        close(*fd);
        *fd = -1;
    }
}

int mb_tcp_build_request_basis(mb_tcp_ctx_t *ctx, uint8_t modbus_uid, int function, int addr,
                               uint16_t nb, uint8_t *req, size_t req_cap)
{
    if (!ctx || !req || req_cap < MODBUS_TCP_MIN_READ_REQ_LENGTH) {
        return -1;
    }
    if (function < 0 || function > 255 || addr < 0 || addr > 65535) {
        return -1;
    }

    pthread_mutex_lock(&ctx->lock);
    if (ctx->tid < UINT16_MAX) {
        ctx->tid++;
    } else {
        ctx->tid = 1;
    }
    uint16_t tid = ctx->tid;
    pthread_mutex_unlock(&ctx->lock);

    /** MBAP length = unit (1) + PDU (5) for read-style requests. */
    const uint16_t mbap_len = 1u + 5u;

    req[0] = (uint8_t)(tid >> 8);
    req[1] = (uint8_t)(tid & 0xFFu);
    req[2] = 0;
    req[3] = 0;
    req[4] = (uint8_t)(mbap_len >> 8);
    req[5] = (uint8_t)(mbap_len & 0xFFu);
    req[6] = modbus_uid;
    req[7] = (uint8_t)function;
    req[8] = (uint8_t)((unsigned)addr >> 8);
    req[9] = (uint8_t)((unsigned)addr & 0xFFu);
    req[10] = (uint8_t)(nb >> 8);
    req[11] = (uint8_t)(nb & 0xFFu);

    return 0;
}

int mb_tcp_build_fc06_request(mb_tcp_ctx_t *ctx, uint8_t modbus_uid,
                               uint16_t addr, uint16_t value,
                               uint8_t *req, size_t req_cap)
{
    if (!ctx || !req || req_cap < MODBUS_TCP_MIN_READ_REQ_LENGTH) {
        return -1;
    }

    pthread_mutex_lock(&ctx->lock);
    if (ctx->tid < UINT16_MAX) {
        ctx->tid++;
    } else {
        ctx->tid = 1;
    }
    uint16_t tid = ctx->tid;
    pthread_mutex_unlock(&ctx->lock);

    req[0]  = (uint8_t)(tid >> 8);
    req[1]  = (uint8_t)(tid & 0xFFu);
    req[2]  = 0x00u;
    req[3]  = 0x00u;
    req[4]  = 0x00u;
    req[5]  = MODBUS_TCP_MBAP_LEN_STANDARD_REQ;
    req[6]  = modbus_uid;
    req[7]  = MODBUS_FUNC_WRITE_SINGLE_REGISTER;
    req[8]  = (uint8_t)(addr >> 8);
    req[9]  = (uint8_t)(addr & 0xFFu);
    req[10] = (uint8_t)(value >> 8);
    req[11] = (uint8_t)(value & 0xFFu);

    return 0;
}

int mb_tcp_build_fc16_request(mb_tcp_ctx_t *ctx, uint8_t modbus_uid,
                               uint16_t addr, uint16_t num_registers,
                               const uint16_t *values,
                               uint8_t *req, size_t req_cap)
{
    if (!ctx || !req || !values || num_registers == 0) {
        return -1;
    }

    uint8_t  byte_count = (uint8_t)(num_registers * 2u);
    uint16_t mbap_len   = (uint16_t)(7u + (uint16_t)byte_count);
    size_t   total_len  = MODBUS_TCP_MBAP_HEADER_LEN + (size_t)mbap_len;

    if (req_cap < total_len) {
        return -1;
    }

    pthread_mutex_lock(&ctx->lock);
    if (ctx->tid < UINT16_MAX) {
        ctx->tid++;
    } else {
        ctx->tid = 1;
    }
    uint16_t tid = ctx->tid;
    pthread_mutex_unlock(&ctx->lock);

    req[0]  = (uint8_t)(tid >> 8);
    req[1]  = (uint8_t)(tid & 0xFFu);
    req[2]  = 0x00u;
    req[3]  = 0x00u;
    req[4]  = (uint8_t)(mbap_len >> 8);
    req[5]  = (uint8_t)(mbap_len & 0xFFu);
    req[6]  = modbus_uid;
    req[7]  = MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    req[8]  = (uint8_t)(addr >> 8);
    req[9]  = (uint8_t)(addr & 0xFFu);
    req[10] = (uint8_t)(num_registers >> 8);
    req[11] = (uint8_t)(num_registers & 0xFFu);
    req[12] = byte_count;

    for (uint16_t i = 0u; i < num_registers; i++) {
        size_t offset = 13u + (size_t)(i * 2u);
        req[offset]      = (uint8_t)(values[i] >> 8);
        req[offset + 1u] = (uint8_t)(values[i] & 0xFFu);
    }

    return (int)total_len;
}

int mb_tcp_parse_request(const uint8_t *request, size_t length, uint8_t *function_code,
                         uint16_t *address, uint16_t *quantity)
{
    if (!request || !function_code || !address || !quantity) {
        return -1;
    }
    if (length < MODBUS_TCP_MBAP_PREFIX_LEN) {
        return -1;
    }

    uint16_t mbap_len = ((uint16_t)request[4] << 8) | (uint16_t)request[5];
    if (mbap_len < 6u) {
        return -1;
    }
    if (mbap_len > (uint16_t)(MODBUS_TCP_MAX_ADU_LENGTH - MODBUS_TCP_MBAP_PREFIX_LEN)) {
        return -1;
    }
    if (length < MODBUS_TCP_MBAP_PREFIX_LEN + (size_t)mbap_len) {
        return -1;
    }

    *function_code = request[7];
    *address = (uint16_t)(((uint16_t)request[8] << 8) | (uint16_t)request[9]);

    if (*function_code == MODBUS_FUNC_WRITE_SINGLE_REGISTER) {
        *quantity = 1;
    } else {
        *quantity = (uint16_t)(((uint16_t)request[10] << 8) | (uint16_t)request[11]);
    }

    return 0;
}

int mb_tcp_build_read_registers_response(uint16_t tid, uint8_t function_code, uint8_t unit_id,
                                         const uint16_t *data, uint16_t quantity, uint8_t *response,
                                         size_t response_cap)
{
    if (!response || !data) {
        return -1;
    }
    if (quantity > 125) {
        return -1;
    }

    size_t pdu_len = 1u + 1u + (size_t)quantity * 2u;
    size_t mbap_len = 1u + pdu_len;
    size_t total = MODBUS_TCP_MBAP_PREFIX_LEN + mbap_len;

    if (response_cap < total) {
        return -1;
    }

    response[0] = (uint8_t)((tid >> 8) & 0xFFu);
    response[1] = (uint8_t)(tid & 0xFFu);
    response[2] = 0;
    response[3] = 0;
    response[4] = (uint8_t)(mbap_len >> 8);
    response[5] = (uint8_t)(mbap_len & 0xFFu);
    response[6] = unit_id;
    response[7] = function_code;
    response[8] = (uint8_t)(quantity * 2u);

    for (uint16_t i = 0; i < quantity; i++) {
        response[9 + i * 2] = (uint8_t)((data[i] >> 8) & 0xFFu);
        response[10 + i * 2] = (uint8_t)(data[i] & 0xFFu);
    }

    return (int)total;
}
