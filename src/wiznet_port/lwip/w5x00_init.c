// src/wiznet_port/lwip/w5x00_init.c

#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "netif/ethernet.h"     // ethernet_input()

#include "lwip/tcpip.h"

#include "wizchip_conf.h"
#include "socket.h"

#include "w5x00_lwip.h"         // netif_initialize(), callbacks, global mac[]
#include "wiznet_port/timer/timer.h"

// These are implemented by your ioLibrary SPI port file (w5x00_spi.c)
#include "w5x00_spi.h"


// Implemented in w5x00_spi.c
void wizchip_spi_initialize(void);
void wizchip_cris_initialize(void);
void wizchip_reset(void);
void wizchip_initialize(void);
void network_initialize(wiz_NetInfo net_info);

static bool w5x00_port_and_chip_up(const uint8_t mac_in[6])
{

        printf("[W5500-INIT] w5x00_port_and_chip_up() ENTRY\n");
    
    printf("[W5500-INIT] Calling wizchip_1ms_timer_initialize...\n");
    // 1ms tick needed by WIZnet stack pieces (DHCP/timeouts/etc.)
    wizchip_1ms_timer_initialize(NULL);
    printf("[W5500-INIT] Calling wizchip_spi_initialize...\n");

    // SPI + critical section + reset + callback registration + W5x00 init
    wizchip_spi_initialize();

        printf("[W5500-INIT] Calling wizchip_cris_initialize...\n");

    wizchip_cris_initialize();
        printf("[W5500-INIT] Calling wizchip_reset...\n");

    wizchip_reset();

        printf("[W5500-INIT] Calling wizchip_initialize...\n");

    wizchip_initialize();


        printf("[W5500-INIT] Programming network info...\n");

    // Program network info into the W5x00 (MAC + DHCP flag)
    wiz_NetInfo ni;
    memset(&ni, 0, sizeof(ni));
    memcpy(ni.mac, mac_in, 6);
    ni.dhcp = NETINFO_DHCP;

    network_initialize(ni);
    printf("[W5500-INIT] w5x00_port_and_chip_up() EXIT\n");

    return true;
}

bool wiznet_init(struct netif *netif, spi_inst_t *spi, uint cs_pin, const uint8_t mac_in[6])
{
    printf("[W5500] wiznet_init() ENTRY - netif=%p, mac=%p\n", (void*)netif, (void*)mac_in);
    
    (void)spi; 
    (void)cs_pin;
    
    if (!netif || !mac_in) {
        printf("[W5500] wiznet_init() NULL PARAMETER! netif=%p mac=%p\n", (void*)netif, (void*)mac_in);
        return false;
    }
    
    printf("[W5500] wiznet_init() calling w5x00_port_and_chip_up...\n");
    
    memcpy(mac, mac_in, 6);
    if (!w5x00_port_and_chip_up(mac_in)) {
        printf("[W5500] w5x00_port_and_chip_up FAILED!\n");
        return false;
    }
    if (!w5x00_port_and_chip_up(mac_in)) return false;

    // DHCP case: 0.0.0.0
    ip4_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&netmask, 0, 0, 0, 0);
    IP4_ADDR(&gw, 0, 0, 0, 0);

// [CHANGE] 'ethernet_input' -> 'tcpip_input'
    if (!netif_add(netif, &ipaddr, &netmask, &gw, NULL, netif_initialize, tcpip_input)) {
        return false;
    }

    netif_set_link_callback(netif, netif_link_callback);
    netif_set_status_callback(netif, netif_status_callback);

    return true;
}

bool wiznet_init_static(struct netif *netif,
                        spi_inst_t *spi,
                        uint cs_pin,
                        const uint8_t mac_in[6],
                        const ip4_addr_t *ipaddr,
                        const ip4_addr_t *netmask,
                        const ip4_addr_t *gw)
{
    (void)spi;
    (void)cs_pin;

    if (!netif || !mac_in || !ipaddr || !netmask || !gw) return false;

    // Copy MAC into the global 'mac' that your netif_initialize() uses
    memcpy(mac, mac_in, 6);

    // Bring up chip first
    wizchip_1ms_timer_initialize(NULL);
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();

    // Program static network info into W5x00
    wiz_NetInfo ni;
    memset(&ni, 0, sizeof(ni));
    memcpy(ni.mac, mac_in, 6);
    ni.dhcp = NETINFO_STATIC;

    // ip4_addr_t stores addr in network byte order; copy bytes out safely
    // by using the lwIP macro to access as u32 then split.
    uint32_t ip_u32  = ip4_addr_get_u32(ipaddr);
    uint32_t sn_u32  = ip4_addr_get_u32(netmask);
    uint32_t gw_u32  = ip4_addr_get_u32(gw);

    ni.ip[0] = (uint8_t)(ip_u32);
    ni.ip[1] = (uint8_t)(ip_u32 >> 8);
    ni.ip[2] = (uint8_t)(ip_u32 >> 16);
    ni.ip[3] = (uint8_t)(ip_u32 >> 24);

    ni.sn[0] = (uint8_t)(sn_u32);
    ni.sn[1] = (uint8_t)(sn_u32 >> 8);
    ni.sn[2] = (uint8_t)(sn_u32 >> 16);
    ni.sn[3] = (uint8_t)(sn_u32 >> 24);

    ni.gw[0] = (uint8_t)(gw_u32);
    ni.gw[1] = (uint8_t)(gw_u32 >> 8);
    ni.gw[2] = (uint8_t)(gw_u32 >> 16);
    ni.gw[3] = (uint8_t)(gw_u32 >> 24);

    network_initialize(ni);

    // Add lwIP netif using static IP
    if (!netif_add(netif, ipaddr, netmask, gw, NULL, netif_initialize, tcpip_input)) {
            return false;
    }
    netif_set_link_callback(netif, netif_link_callback);
    netif_set_status_callback(netif, netif_status_callback);

    return true;
}
