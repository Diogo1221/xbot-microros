
#ifndef CONNECTION_H
#define CONNECTION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONN_MAGIC    (0xC07F1643u ^ (uint32_t)(BOARD_ID))  /* per-board: reflash with new ID resets config */
#define CONN_SSID_MAX 33            /* max SSID length + NUL                   */
#define CONN_PASS_MAX 65            /* max passphrase length + NUL             */

typedef struct __attribute__((packed)) {
    uint32_t magic;                     /* CONN_MAGIC when valid               */
    uint8_t  use_wifi;                  /* 1 = WiFi, 0 = W5500 Ethernet       */
    char     ssid    [CONN_SSID_MAX];   /* WiFi SSID (NUL-terminated)          */
    char     password[CONN_PASS_MAX];   /* WiFi passphrase (NUL-terminated)    */
    uint8_t  agent_ip[4];               /* micro-ROS agent IPv4 address        */
    uint16_t agent_port;                /* micro-ROS agent UDP port            */
} conn_config_t;                        /* written zero-padded to 256 bytes    */

void                  conn_early_init   (void);
const conn_config_t  *conn_get_config   (void);
void                  conn_save_config  (const conn_config_t *cfg);

#ifdef USE_WIFI

#include <uxr/client/profile/transport/custom/custom_transport.h>

bool   connection_wifi_init (void);

bool   wifi_transport_open  (struct uxrCustomTransport *t);
bool   wifi_transport_close (struct uxrCustomTransport *t);
size_t wifi_transport_write (struct uxrCustomTransport *t,
                              const uint8_t *buf, size_t len, uint8_t *err);
size_t wifi_transport_read  (struct uxrCustomTransport *t,
                              uint8_t *buf, size_t len,
                              int timeout_ms, uint8_t *err);

#endif /* USE_WIFI */

#ifdef __cplusplus
}
#endif

#endif /* CONNECTION_H */
