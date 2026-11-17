
#include "eth_w5500.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" // Required if using LwIP locking macros provided by Pico SDK

#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/ip4_addr.h"

#include "wizchip_conf.h" // [CRITICAL] Needed for getVERSIONR()
#include "wiznet_port/lwip/w5x00_lwip.h"
#include "wiznet_port/lwip/w5x00_init.h"
#include "wiznet_port/timer/timer.h"

extern void wizchip_spi_initialize(void);
extern void wizchip_cris_initialize(void);
extern void wizchip_reset(void);
extern void wizchip_initialize(void);

#define W5500_SPI_PORT  spi0
#define W5500_PIN_CS    17

static struct netif s_eth_netif;   // The LwIP Network Interface Structure
static bool s_hw_inited = false;   // Flag: Hardware initialized?
static bool s_netif_added = false; // Flag: Interface added to LwIP? (Prevents Panic)

static void w5500_pre_init_hardware(void) {
    if (s_hw_inited) return;

    printf("[W5500] Pre-initializing Hardware...\n");

    // Initialize Critical Section (Mutex/Spinlock for SPI)
    wizchip_cris_initialize();

    // Initialize SPI0 at defined baud rate
    printf("[W5500] SPI initialize...\n");
    wizchip_spi_initialize();

    // Hardware Reset Pulse
    printf("[W5500] Reset initialize...\n");
    wizchip_reset();

    // WIZnet Library Init (Set Buffer Sizes)
    printf("[W5500] Wizchip_initialize...\n");
    wizchip_initialize();

    // Timer Init for DHCP timeouts
    printf("[W5500] Wizchip_1ms_timer_initialize...\n");
    wizchip_1ms_timer_initialize(NULL);

    s_hw_inited = true;
}

bool eth_w5500_dhcp_up(void) {
    printf("Inside eth_w5500_dhcp_up \n");

    // 1. Hardware Pre-Init (Safe to call repeatedly, internal guard)
    w5500_pre_init_hardware();

    LOCK_TCPIP_CORE();
    {
        if (!s_netif_added) {
            printf("[W5500] Registering Interface with LwIP...\n");
            uint8_t mac[6] = {0x02, 0x08, 0xDC, 0x12, 0x34, 0x56};

            // Link the W5500 driver functions to the LwIP struct
            bool success = wiznet_init(&s_eth_netif, W5500_SPI_PORT, W5500_PIN_CS, mac);

            if (success) {
                s_netif_added = true;
            } else {
                printf("[ERROR] wiznet_init failed. Retrying later.\n");
            }
        }

        // 3. Bring Link Up & Start DHCP (Safe to call on retry)
        if (s_netif_added) {
            // Ensure interface is administratively up
            netif_set_up(&s_eth_netif);

            // Restart DHCP state machine
            dhcp_start(&s_eth_netif);
        }
    }
    UNLOCK_TCPIP_CORE();

    // 4. IP Verification Loop (Wait for DHCP ACK)
    printf("[W5500] Waiting for Link & IP...\n");
    for (int i = 0; i < 10; i++) {
        // Check if Link is Up AND we have a valid non-zero IP
        if (netif_is_up(&s_eth_netif) && !ip4_addr_isany_val(*netif_ip4_addr(&s_eth_netif))) {
            char ip_str[16];
            ip4addr_ntoa_r(netif_ip4_addr(&s_eth_netif), ip_str, 16);
            printf("[W5500] IP OBTAINED: %s\n", ip_str);
            return true;
        }

        // Diagnostic: Check if hardware is responding via SPI
        if (i % 5 == 0) {
            uint8_t ver = getVERSIONR();
            printf("[DIAG] W5500 Version: 0x%02X\n", ver);
            if (ver == 0x04) printf("[DIAG] Hardware is HEALTHY.\n");
        }

        sleep_ms(500);
    }

    return false;
}

bool eth_w5500_static_up(const char* ip, const char* mask, const char* gw) {
    // 1. Hardware Init
    w5500_pre_init_hardware();

    uint8_t mac[6] = {0x02, 0x08, 0xDC, 0x12, 0x34, 0x56};

    // 2. Parse String IPs to LwIP Structs
    ip4_addr_t ipaddr, netmask, gateway;
    ip4addr_aton(ip, &ipaddr);
    ip4addr_aton(mask, &netmask);
    ip4addr_aton(gw, &gateway);

    // 3. Register Static Interface
    LOCK_TCPIP_CORE();
    {
        if (!wiznet_init_static(&s_eth_netif, W5500_SPI_PORT, W5500_PIN_CS, mac,
                                &ipaddr, &netmask, &gateway)) {
            UNLOCK_TCPIP_CORE();
            return false;
        }

        // Mark as default interface and bring up
        netif_set_default(&s_eth_netif);
        netif_set_up(&s_eth_netif);
    }
    UNLOCK_TCPIP_CORE();

    printf("[W5500] Static IP Set: %s\n", ip);
    return true;
}