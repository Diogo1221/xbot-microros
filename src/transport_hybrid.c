
#include "transport_hybrid.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "connection.h"

#include "wizchip_conf.h"
#include "socket.h"

#define UROS_SOCK_NUM 0

// Global Flag: Determines which transport is active
bool g_is_eth_transport = true;

uint8_t  g_agent_ip_bytes[4] = {0, 0, 0, 0};
uint16_t g_agent_port        = 8888;

static bool     s_found;
static uint8_t  s_found_ip[4];
static uint16_t s_found_port;

void set_discovered_agent(const uint8_t ip[4], uint16_t port) {
    memcpy(s_found_ip, ip, 4);
    s_found_port = port;
    s_found = true;
}

void set_microros_agent_ip(const char* ip, int port) {
    if (ip) {
        // Simple manual parse to avoid heavy inet_aton overhead
        unsigned int a, b, c, d;
        if (sscanf(ip, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            g_agent_ip_bytes[0] = (uint8_t)a;
            g_agent_ip_bytes[1] = (uint8_t)b;
            g_agent_ip_bytes[2] = (uint8_t)c;
            g_agent_ip_bytes[3] = (uint8_t)d;
        }
    }
    g_agent_port = (uint16_t)port;
}

__attribute__((weak)) int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool w5500_transport_open(struct uxrCustomTransport * transport) {
    (void)transport;

    const conn_config_t *cfg = conn_get_config();
    memcpy(g_agent_ip_bytes, s_found ? s_found_ip : cfg->agent_ip, 4);
    g_agent_port = s_found ? s_found_port : cfg->agent_port;

    // Note: wizchip_init() is assumed to be called in vUROSTask already.

    // Safety: Ensure Socket 0 is closed before attempting to open it
    close(UROS_SOCK_NUM);

    if(socket(UROS_SOCK_NUM, Sn_MR_UDP, 4444, 0) != UROS_SOCK_NUM) {
        return false; // Failed to open socket
    }
    return true;
}

bool w5500_transport_close(struct uxrCustomTransport * transport) {
    (void)transport;
    close(UROS_SOCK_NUM);
    return true;
}

size_t w5500_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    (void)transport;

    int32_t ret = sendto(UROS_SOCK_NUM, (uint8_t*)buf, len, g_agent_ip_bytes, g_agent_port);

    if (ret < 0) {
        *err = 1; // Error occurred
        return 0;
    }
    return (size_t)ret;
}

size_t w5500_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    (void)transport;

    // 1. Calculate the deadline
    uint64_t start_us = time_us_64();
    uint64_t timeout_us = (uint64_t)timeout * 1000;

    while (getSn_RX_RSR(UROS_SOCK_NUM) == 0) {

        // Check if timeout has expired
        if ((time_us_64() - start_us) >= timeout_us) {
            *err = 0; // Success code (0) but 0 bytes means "Timeout" in uROS
            return 0;
        }

    }

    // 3. Data Available: Read packet
    uint8_t remote_ip[4];
    uint16_t remote_port;

    int32_t ret = recvfrom(UROS_SOCK_NUM, buf, len, remote_ip, &remote_port);

    if (ret > 0) {
        return (size_t)ret;
    }

    // If we get here, something strange happened (e.g. read error)
    *err = 1;
    return 0;
}

bool serial_transport_open(struct uxrCustomTransport * transport)
{
    (void)transport;
    static bool require_init = true;
    if(require_init)
    {
        stdio_init_all();
        require_init = false;
    }
    return true;
}

bool serial_transport_close(struct uxrCustomTransport * transport)
{
    (void)transport;
    return true;
}

size_t serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)transport;
    size_t bytes_written = 0;
    for (size_t i = 0; i < len; i++)
    {
        // PICO_ERROR_GENERIC indicates buffer full or disconnected
        if (putchar(buf[i]) == PICO_ERROR_GENERIC)
        {
            *errcode = 1;
            return bytes_written;
        }
        bytes_written++;
    }
    return bytes_written;
}

size_t serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    (void)transport;
    uint64_t start_time_us = time_us_64();
    size_t bytes_read = 0;

    while (bytes_read < len)
    {
        int64_t elapsed_time_us = (int64_t)timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0) {
            return bytes_read;
        }

        int character = getchar_timeout_us(elapsed_time_us);
        if (character == PICO_ERROR_TIMEOUT) {
            return bytes_read;
        }

        buf[bytes_read++] = (uint8_t)character;
    }
    return bytes_read;
}

bool hybrid_transport_open(struct uxrCustomTransport * transport) {
    if (g_is_eth_transport) {
        return w5500_transport_open(transport);
    }
    return serial_transport_open(transport);
}

bool hybrid_transport_close(struct uxrCustomTransport * transport) {
    if (g_is_eth_transport) {
        return w5500_transport_close(transport);
    }
    return serial_transport_close(transport);
}

size_t hybrid_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    if (g_is_eth_transport) {
        return w5500_transport_write(transport, buf, len, err);
    }
    return serial_transport_write(transport, buf, len, err);
}

size_t hybrid_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    if (g_is_eth_transport) {
        return w5500_transport_read(transport, buf, len, timeout, err);
    }
    return serial_transport_read(transport, buf, len, timeout, err);
}