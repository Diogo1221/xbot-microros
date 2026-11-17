
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"

#include "hardware/uart.h"
#include "hardware/pwm.h"
// [FIX] Headers required for overclocking
#include "hardware/vreg.h"
#include "hardware/clocks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <control_msgs/action/follow_joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>

#include "rcl/logging.h"
#include "rcutils/logging_macros.h"

#include "uROS.h"
#include "HW.h"
#include "memory.h"
#include "connection.h"     /* conn_early_init() — flash config before tasks  */

#ifndef USE_WIFI
#include "eth_w5500.h"
#include "wizchip_conf.h"   /* getVERSIONR()                                  */
#endif

#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3)
#define UROS_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)

// --- Task Stack Sizes (in Bytes) ---
#define MOTOR_STACK_SIZE    (24 * 1024)
#define UROS_STACK_SIZE     (32 * 1024)

// --- Core Affinity Masks ---
#define CORE_0_AFFINITY     (1 << 0) // Core 0: Network
#define CORE_1_AFFINITY     (1 << 1) // Core 1: Physics

void vMotorControlTask(void *pvParameters);
void vUROSTask(void *pvParameters);
void emergency_stop(void);

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    set_sys_clock_khz(250000, true);

    setup_outputs(); // Basic GPIO setup (LEDs, Debug pins)
    stdio_init_all();

    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }

    printf("\n\n=== BOOTING PICO 2 W (ROBOT) ===\n");
    printf("Clock: 250 MHz | Core 0: Network | Core 1: Physics\n");

    setup_outputs();

    conn_early_init();

    BaseType_t result;

    TaskHandle_t uros_task_handle;
    result = xTaskCreate(
        vUROSTask,            // Task Function
        "UROSTask",           // Task Name
        UROS_STACK_SIZE,      // Stack Size
        NULL,                 // Parameters
        UROS_TASK_PRIORITY,   // Priority
        &uros_task_handle     // Handle
    );

    if (result != pdPASS) {
        printf("FATAL: Failed to create Micro-ROS Task\n");
        while (1); // Halt system
    }

    // Pin Network Task to Core 0
    vTaskCoreAffinitySet(uros_task_handle, CORE_0_AFFINITY);
    printf("uROS task created on Core 0\n");

    size_t free_heap = xPortGetFreeHeapSize();
    printf("Free heap: %u bytes\n", free_heap);
    printf("Starting FreeRTOS scheduler...\n");

    // Pass control to FreeRTOS. This function should never return.
    vTaskStartScheduler();

    printf("ERROR: Scheduler returned! (Insufficient Heap?)\n");
    while (1) {
        emergency_stop();
        sleep_ms(1000);
    }
}