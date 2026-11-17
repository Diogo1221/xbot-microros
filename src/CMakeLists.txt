# =============================================================================
# CONNECTION MODE — auto-detected from board_config.h
# =============================================================================
# Set USE_WIFI in board_config.h to switch to WiFi transport.
# This block reads that header and configures the build accordingly so you
# only ever need to edit one file.
# =============================================================================
file(READ "${CMAKE_CURRENT_LIST_DIR}/board_config.h" _BOARD_CFG)
if(_BOARD_CFG MATCHES "(^|\n)[ \t]*#[ \t]*define[ \t]+USE_WIFI")
    set(CONN_USE_WIFI TRUE)
    message(STATUS "[CONN] Transport mode: WiFi (CYW43) — W5500 sources excluded")
else()
    set(CONN_USE_WIFI FALSE)
    message(STATUS "[CONN] Transport mode: Ethernet W5500")
endif()


# =============================================================================
# EXECUTABLE SOURCES
# =============================================================================
add_executable(${NAME}
    # --- Application logic ---
    main.cpp
    connection.c          # NEW: persistent config + WiFi transport
    HW.cpp
    uROS.cpp
    memory.cpp
    FreeRTOS_hooks.c

    # --- W5500 Ethernet sources (excluded when WiFi is selected) ---
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:transport_hybrid.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:agent_discovery.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:wiznet_port/ioLibrary_Driver/src/w5x00_spi.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:wiznet_port/ioLibrary_Driver/src/w5x00_gpio_irq.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:wiznet_port/timer/timer.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Ethernet/wizchip_conf.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Ethernet/socket.c>
    $<$<NOT:$<BOOL:${CONN_USE_WIFI}>>:../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Ethernet/W5500/w5500.c>
)


# =============================================================================
# LIBRARIES
# =============================================================================
target_link_libraries(${NAME}
    pico_stdlib
    microros

    # Hardware drivers (common to both modes)
    hardware_pwm
    hardware_uart
    hardware_dma
    hardware_watchdog
    hardware_timer
    hardware_irq
    hardware_adc
    hardware_vreg
    hardware_clocks
    hardware_flash          # flash_range_erase / flash_range_program

    # Multicore & sync
    pico_sync
    pico_multicore

    # FreeRTOS SMP kernel
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    freertos_config
)

if(CONN_USE_WIFI)
    # WiFi mode: CYW43 driver + LwIP integrated with FreeRTOS
    target_link_libraries(${NAME}
        pico_cyw43_arch_lwip_sys_freertos
    )
    target_compile_definitions(${NAME} PRIVATE USE_WIFI=1)
else()
    # Ethernet mode: SPI for W5500 (LwIP deliberately excluded)
    target_link_libraries(${NAME}
        hardware_spi
    )
endif()


# =============================================================================
# INCLUDE DIRECTORIES
# =============================================================================
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}                               # /src
    ${CMAKE_CURRENT_LIST_DIR}/../libmicroros/include        # micro-ROS headers
)

if(NOT CONN_USE_WIFI)
    target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/wiznet_port
        ${CMAKE_CURRENT_LIST_DIR}/wiznet_port/ioLibrary_Driver/inc
        ${CMAKE_CURRENT_LIST_DIR}/wiznet_port/timer
        ${CMAKE_CURRENT_LIST_DIR}/../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/inc
        ${CMAKE_CURRENT_LIST_DIR}/../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Ethernet
        ${CMAKE_CURRENT_LIST_DIR}/../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Ethernet/W5500
        ${CMAKE_CURRENT_LIST_DIR}/../lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/Internet
    )
    # WIZnet chip ID required by ioLibrary
    target_compile_definitions(${NAME} PRIVATE _WIZCHIP_=5500)
endif()


# =============================================================================
# COMPILER FLAGS
# =============================================================================
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -std=c11   -ffunction-sections -fdata-sections -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ffunction-sections -fdata-sections -O3")

target_compile_options(${NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)


# =============================================================================
# PICO CONFIG
# =============================================================================
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

pico_add_extra_outputs(${NAME})
