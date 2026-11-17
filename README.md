# XBot board firmware

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Firmware for the boards of the XBot leg-wheel quadruped: micro-ROS on the RP2350 (Raspberry Pi Pico 2) over W5500 Ethernet (UDP) to a micro-ROS Agent.
Based on [micro-ROS for the Raspberry Pi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) (Apache-2.0).

The robot has 8 boards (4 legs, TOP and BOT per leg). The same source is built once per board; the board is selected by `BOARD_ID` in `src/board_config.h`.

| `BOARD_ID` | Board | `BOARD_ID` | Board |
|---|---|---|---|
| 11 | L1 TOP | 12 | L1 BOT |
| 21 | L2 TOP | 22 | L2 BOT |
| 31 | L3 TOP | 32 | L3 BOT |
| 41 | L4 TOP | 42 | L4 BOT |

## Dependencies

Versions used to build this tree (Ubuntu 24.04):

| Component | Version | How it is found |
|---|---|---|
| CMake | 3.28.3 (3.12 or newer) | |
| Arm GNU toolchain | `arm-none-eabi-gcc` 13.2.1 (Ubuntu package `gcc-arm-none-eabi`) | `PATH` |
| [Pico SDK](https://github.com/raspberrypi/pico-sdk) | 2.1.1, with submodules | `PICO_SDK_PATH` |
| [pico-extras](https://github.com/raspberrypi/pico-extras) | sdk-2.1.0 | `PICO_EXTRAS_PATH` (required) |
| [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) | V11.1.x (`V11.1.0+`), RP2350 ARM NTZ port | `FREERTOS_KERNEL_PATH` |
| [ioLibrary_Driver](https://github.com/Wiznet/ioLibrary_Driver) (WIZnet W5500 driver) | from WIZnet-PICO-LWIP-C 2.0.0 | included: `lib/` |
| micro-ROS static library | ROS 2 Jazzy: `libmicroros.a` and `include/` | included: `libmicroros/` |

`lib/` has only the `Ethernet` and `Internet` parts of the WIZnet ioLibrary (its license is in `lib/WIZnet-PICO-LWIP-C/libraries/ioLibrary_Driver/license.txt`).
`libmicroros/` is the prebuilt micro-ROS library used for the firmware; the micro-ROS sources it was built from are Apache-2.0 and eProsima/ROS 2 licenses apply to its headers.
The Pico SDK, pico-extras and FreeRTOS-Kernel are not included; install them at the versions above.

On the host that runs the Agent: ROS 2 Jazzy with `micro_ros_agent`.

## Build

```bash
export PICO_SDK_PATH=$HOME/pico-sdk
export PICO_EXTRAS_PATH=$HOME/pico-extras
export FREERTOS_KERNEL_PATH=$HOME/FreeRTOS-Kernel

# set BOARD_ID in src/board_config.h, then:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2_w -DPICO_PLATFORM=rp2350-arm-s \
      -DCMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc \
      -DCMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ \
      -DCMAKE_ASM_COMPILER=/usr/bin/arm-none-eabi-gcc
cmake --build build -j
```

The image is `build/src/main.uf2`. To flash a board, hold BOOTSEL, plug the USB cable and copy the file to the RP2350 drive.
Repeat with the next `BOARD_ID` for each board.

## Network

The transport is Ethernet (W5500).

| | Address |
|---|---|
| Board IP | `192.168.1.<BOARD_ID>` (through the switch) |
| Board 41 | `192.168.5.41` (direct link to the host) |
| Agent IP | `192.168.1.100`, or `192.168.5.1` for board 41 |
| Agent port | UDP 8888 |

The Agent address is set by `WIFI_AGENT_IP_DEFAULT` and `WIFI_AGENT_PORT_DEFAULT` in `src/board_config.h` (the names come from the WiFi path).
Start the Agent on the host:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Agent discovery

On the first Ethernet initialization the board sends an XRCE `GET_INFO` to the Multicast group `239.255.0.2:7400` and uses the address announced by the Agent; if no Agent in the board's sub-net answers, the configured Agent address is used.
It is controlled by `XBOT_MULTICAST_DISCOVERY` in `src/board_config.h` (1 = on, 0 = off). Start the Agent with discovery enabled:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -d 7400
```

## License

Apache-2.0. See [LICENSE](LICENSE).
