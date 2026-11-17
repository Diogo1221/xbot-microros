
#include "connection.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"   /* flash_range_erase / flash_range_program      */
#include "hardware/sync.h"    /* save_and_disable_interrupts / restore         */

#ifndef PICO_FLASH_SIZE_BYTES
#  define PICO_FLASH_SIZE_BYTES (4u * 1024u * 1024u)
#endif

#define CONN_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CONN_FLASH_PTR      ((const conn_config_t *)(XIP_BASE + CONN_FLASH_OFFSET))

/* sizeof(conn_config_t) must fit in one flash page (256 bytes). */
_Static_assert(sizeof(conn_config_t) <= FLASH_PAGE_SIZE,
               "conn_config_t exceeds one flash page — increase CONN_PASS_MAX");

#ifndef WIFI_SSID_DEFAULT
#  define WIFI_SSID_DEFAULT       "your_network_ssid"
#endif
#ifndef WIFI_PASSWORD_DEFAULT
#  define WIFI_PASSWORD_DEFAULT   "your_network_password"
#endif
#ifndef WIFI_AGENT_IP_DEFAULT
#  define WIFI_AGENT_IP_DEFAULT   {192, 168, 1, 100}
#endif
#ifndef WIFI_AGENT_PORT_DEFAULT
#  define WIFI_AGENT_PORT_DEFAULT 8888
#endif

static conn_config_t s_cfg;
static bool          s_loaded = false;

const conn_config_t *conn_get_config(void) { return &s_cfg; }

static void __not_in_flash_func(do_flash_write)(const uint8_t *page256)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase  (CONN_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONN_FLASH_OFFSET, page256, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void conn_save_config(const conn_config_t *cfg)
{
    /* Zero-pad to a full 256-byte page so unused bytes are predictable. */
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page, cfg, sizeof(conn_config_t));
    do_flash_write(page);
    printf("[CONN] Config saved to flash.\n");
}

void conn_early_init(void)
{
    if (s_loaded) return;

    if (CONN_FLASH_PTR->magic == CONN_MAGIC) {
        memcpy(&s_cfg, CONN_FLASH_PTR, sizeof(conn_config_t));
        printf("[CONN] Config from flash — mode: %s  agent: %u.%u.%u.%u:%u\n",
               s_cfg.use_wifi ? "WiFi" : "Ethernet",
               s_cfg.agent_ip[0], s_cfg.agent_ip[1],
               s_cfg.agent_ip[2], s_cfg.agent_ip[3],
               s_cfg.agent_port);
    } else {
        /* First boot or erased flash — build and persist defaults. */
        printf("[CONN] Flash blank — writing defaults.\n");
        memset(&s_cfg, 0, sizeof(s_cfg));
        s_cfg.magic      = CONN_MAGIC;
#ifdef USE_WIFI
        s_cfg.use_wifi   = 1;
#else
        s_cfg.use_wifi   = 0;
#endif
        const uint8_t default_ip[] = WIFI_AGENT_IP_DEFAULT;
        memcpy(s_cfg.agent_ip, default_ip, 4);
        s_cfg.agent_port = WIFI_AGENT_PORT_DEFAULT;
        strncpy(s_cfg.ssid,     WIFI_SSID_DEFAULT,     CONN_SSID_MAX - 1);
        strncpy(s_cfg.password, WIFI_PASSWORD_DEFAULT, CONN_PASS_MAX - 1);

        conn_save_config(&s_cfg);
    }

    s_loaded = true;
}

#ifdef USE_WIFI

#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netif.h"

static int s_sock = -1;   /* UDP socket fd, open while transport is active   */

bool connection_wifi_init(void)
{
    const conn_config_t *cfg = conn_get_config();
    printf("[CONN] WiFi init  SSID: \"%s\" ...\n", cfg->ssid);

    if (cyw43_arch_init() != 0) {
        printf("[CONN] cyw43_arch_init() failed.\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();

    int rc = cyw43_arch_wifi_connect_timeout_ms(
                 cfg->ssid, cfg->password,
                 CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc != 0) {
        printf("[CONN] WiFi connect failed (err %d).\n", rc);
        return false;
    }

    printf("[CONN] WiFi connected.  IP: %s\n",
           ip4addr_ntoa(netif_ip4_addr(netif_default)));
    return true;
}

bool wifi_transport_open(struct uxrCustomTransport *t)
{
    (void)t;

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        printf("[CONN] socket() failed.\n");
        return false;
    }

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family      = AF_INET;
    local.sin_port        = htons(4444);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        printf("[CONN] bind() failed.\n");
        close(s_sock);
        s_sock = -1;
        return false;
    }

    printf("[CONN] WiFi UDP transport open (local :4444).\n");
    return true;
}

bool wifi_transport_close(struct uxrCustomTransport *t)
{
    (void)t;
    if (s_sock >= 0) { close(s_sock); s_sock = -1; }
    return true;
}

size_t wifi_transport_write(struct uxrCustomTransport *t,
                             const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t;
    const conn_config_t *cfg = conn_get_config();

    /* Build destination address from stored agent IP bytes. */
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(cfg->agent_port);

    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
             cfg->agent_ip[0], cfg->agent_ip[1],
             cfg->agent_ip[2], cfg->agent_ip[3]);
    dest.sin_addr.s_addr = inet_addr(ip_str);

    int sent = sendto(s_sock, buf, len, 0,
                      (struct sockaddr *)&dest, sizeof(dest));
    if (sent < 0) { *err = 1; return 0; }
    return (size_t)sent;
}

size_t wifi_transport_read(struct uxrCustomTransport *t,
                            uint8_t *buf, size_t len,
                            int timeout_ms, uint8_t *err)
{
    (void)t;

    /* Use select() for portable timeout handling with LwIP sockets. */
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s_sock, &rfds);

    struct timeval tv;
    tv.tv_sec  =  timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int sel = select(s_sock + 1, &rfds, NULL, NULL, &tv);
    if (sel <= 0) { *err = 0; return 0; }   /* timeout → 0 bytes, no error    */

    struct sockaddr_in src;
    socklen_t slen = sizeof(src);
    int rcvd = recvfrom(s_sock, buf, len, 0,
                        (struct sockaddr *)&src, &slen);
    if (rcvd < 0) { *err = 1; return 0; }
    return (size_t)rcvd;
}

#endif /* USE_WIFI */
