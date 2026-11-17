
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool eth_w5500_dhcp_up(void);

bool eth_w5500_static_up(const char* ip, const char* mask, const char* gw);

// Initializes the RP2040 SPI0 interface (Baud rate, Pins)
void wizchip_spi_initialize(void);

// Configures W5500 Buffer sizes (2KB TX/RX per socket)
void wizchip_initialize(void);

// Performs a Hardware Reset (toggles RST pin)
void wizchip_reset(void);

void wizchip_cris_initialize(void);

#ifdef __cplusplus
}
#endif