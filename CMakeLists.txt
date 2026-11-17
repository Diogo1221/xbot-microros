cmake_minimum_required(VERSION 3.12)

set(NAME main) # <-- Name your project/executable here!

set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350)

# Include Pico SDK
include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

# Add pico-extras BEFORE pico_sdk_init()
if(DEFINED ENV{PICO_EXTRAS_PATH})
    message("PICO_EXTRAS_PATH found: $ENV{PICO_EXTRAS_PATH}")
    include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)
else()
    message(FATAL_ERROR "PICO_EXTRAS_PATH not set! Run: export PICO_EXTRAS_PATH=~/pico-extras")
endif()

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
# Pull in FreeRTOS
#include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include("${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake") 
include(FreeRTOS_Kernel_import.cmake)


# Initialize Pico SDK
pico_sdk_init()


# Link directories for micro-ROS library
link_directories(libmicroros)

add_subdirectory(src)

# target_compile_options(main PRIVATE -Wno-volatile)
