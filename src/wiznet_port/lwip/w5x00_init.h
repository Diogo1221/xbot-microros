#ifndef W5X00_INIT_H
#define W5X00_INIT_H

#include <stdbool.h>
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

bool wiznet_init(struct netif *netif, spi_inst_t *spi, uint cs_pin, const uint8_t mac_in[6]);

bool wiznet_init_static(struct netif *netif, spi_inst_t *spi, uint cs_pin, 
                        const uint8_t mac_in[6],
                        const ip4_addr_t *ipaddr, 
                        const ip4_addr_t *netmask, 
                        const ip4_addr_t *gw);

#ifdef __cplusplus
}
#endif

#endif // W5X00_INIT_H
