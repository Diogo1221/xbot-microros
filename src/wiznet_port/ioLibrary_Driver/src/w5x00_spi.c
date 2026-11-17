/**
 * w5x00_spi.c - Auto-Negotiating Version
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/critical_section.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"

// --- Pin Definitions ---
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RST  20

static critical_section_t g_wizchip_cri_sec;

// --- Helper: Manual Register Read (Bypasses Library) ---
// We use this to test the connection before the library is even fully set up.
static uint8_t manual_read_version(void) {
    uint8_t tx[4] = {0x00, 0x39, 0x00, 0x00};
    uint8_t rx[4] = {0};
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx, rx, 4);
    gpio_put(PIN_CS, 1);
    return rx[3];
}

void wizchip_spi_initialize(void)
{
    printf("\n[DIAG] Starting SPI0...\n");

    spi_init(SPI_PORT, 20*1000 * 1000); 
    
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    // -----------------------------------------------------------
    // DIAGNOSTIC: SPI LOOPBACK TEST
    // -----------------------------------------------------------
    printf("[DIAG] Running Loopback Test (GP0 <-> GP3)...\n");
    
    // We send 0xAA. If we read 0xAA, the Pico and code are perfect.
    uint8_t tx_test = 0xAA;
    uint8_t rx_test = 0x00;
    
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // Send one byte
    spi_write_read_blocking(SPI_PORT, &tx_test, &rx_test, 1);
    
    if (rx_test == 0xAA) {
        printf("[DIAG] SUCCESS: Pico Loopback Verified! (Pico TX -> RX OK)\n");
        printf("[DIAG] If W5500 fails now, the issue is WIRING to the chip.\n");
    } else {
        printf("[DIAG] FAILURE: Loopback Read 0x%02X (Expected 0xAA)\n", rx_test);
        printf("[DIAG] CHECK: Is GP0 connected to GP3? Are pins defined correctly?\n");
        // We do not stop, so you can see the negotiation result too
    }
    // -----------------------------------------------------------

    // 2. Chip Reset
    printf("[W5500] Resetting Chip...\n");
    gpio_put(PIN_RST, 0);
    sleep_ms(200); 
    gpio_put(PIN_RST, 1);
    sleep_ms(800); 

    // 3. Negotiate Mode
    printf("[W5500-AUTO] Negotiating with W5500...\n");
    uint8_t modes[4][2] = { {0,0}, {1,1}, {0,1}, {1,0} };
    
    for (int i = 0; i < 4; i++) {
        int cpol = modes[i][0];
        int cpha = modes[i][1];

        spi_set_format(SPI_PORT, 8, (spi_cpol_t)cpol, (spi_cpha_t)cpha, SPI_MSB_FIRST);
        sleep_ms(10); 

        uint8_t ver = manual_read_version();
        printf("    Mode (%d, %d) -> Read: 0x%02X\n", cpol, cpha, ver);

        if (ver == 0x04) {
            printf("[W5500-AUTO] LOCKED: Mode (%d, %d)\n", cpol, cpha);
            return; 
        }
    }

    printf("[W5500-AUTO] FAIL. Defaulting to Mode 0.\n");
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}
// --- Standard Driver Functions (Unchanged) ---
static inline void wizchip_select(void) { gpio_put(PIN_CS, 0); }
static inline void wizchip_deselect(void) { gpio_put(PIN_CS, 1); }

static uint8_t wizchip_read(void) {
    uint8_t rx_data = 0, tx_data = 0xFF;
    spi_read_blocking(SPI_PORT, tx_data, &rx_data, 1);
    return rx_data;
}

static void wizchip_write(uint8_t tx_data) {
    spi_write_blocking(SPI_PORT, &tx_data, 1);
}

static void wizchip_critical_section_lock(void) {
    critical_section_enter_blocking(&g_wizchip_cri_sec);
}

static void wizchip_critical_section_unlock(void) {
    critical_section_exit(&g_wizchip_cri_sec);
}

void wizchip_cris_initialize(void) {
    critical_section_init(&g_wizchip_cri_sec);
    reg_wizchip_cris_cbfunc(wizchip_critical_section_lock, wizchip_critical_section_unlock);
}

void wizchip_initialize(void) {
    wizchip_deselect();
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

    // Standard Buffer Config
    uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1) {
        printf("[W5500] WIZCHIP Initialized Fail\n");
    }
}

void wizchip_reset() {
    // This is now handled in spi_initialize, but we keep it for API compatibility
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);
}

void network_initialize(wiz_NetInfo net_info) {
    ctlnetwork(CN_SET_NETINFO, (void *)&net_info);
}