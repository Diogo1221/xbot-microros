#ifndef HW_H
#define HW_H

#include "board_config.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h" // Required for System Reset

#include "pico/platform.h"
#include "hardware/structs/sio.h" // [CRITICAL] For Atomic GPIO Access

#include "hardware/timer.h" // [ADDED] For hardware timer
#include "hardware/irq.h"   // [ADDED] For ISR handling

#include "uROS.h"

#pragma once
#include <stdint.h>
extern volatile float    g_motor_control_freq_hz;   // instantaneous
extern volatile uint32_t g_motor_control_period_us; // last period (us)

extern volatile float g_angle_read_freq_hz;
extern volatile uint32_t g_angle_read_period_us;

#ifndef USE_WIFI
#define SPI_PORT_1     spi1
#define SPI_MISO_PIN_1 12
#define SPI_CLK_PIN_1  14
#define SPI_MOSI_PIN_1 15
#endif /* USE_WIFI */

#define MOTOR_DEBUG_PIN 21
#define COMMS_DEBUG_PIN 22
#ifndef LATENCY_DEBUG_PIN
#define LATENCY_DEBUG_PIN 27
#endif
#ifndef DEBUG_PWM_APPLY_PIN
#define DEBUG_PWM_APPLY_PIN 28
#endif
#ifndef LATENCY_PULSE_US
#define LATENCY_PULSE_US 50000
#endif
#ifndef LATENCY_CB_DEBUG
#define LATENCY_CB_DEBUG 1
#endif
#ifndef LATENCY_RESYNC_MS
#define LATENCY_RESYNC_MS 5000
#endif
#ifndef LATENCY_RESYNC_TIMEOUT_MS
#define LATENCY_RESYNC_TIMEOUT_MS 20
#endif

#if IS_TOP_BOARD
    #define MOTOR_0_PWM_PIN 0
    #define DIR_PIN_0       1

    #define PIN_M1_RPWM     4  // Right PWM

    #define PIN_M1_LPWM     5  // Left PWM — standard wiring, uniform across the fleet
    #define PIN_M1_REN      6  // hard-wired 3.3V on the board, never driven by firmware
    #define PIN_M1_LEN      7  // hard-wired 3.3V on the board, never driven by firmware

    // --- Encoders (Moved from 6/7 to 2/3) ---
    #define MOTOR0_PIN      2
    #define MOTOR1_PIN      3

    #define SPI_CS_PIN_0 13
    #define SPI_CS_PIN_1 11

    #define PULSES_PER_REV_M0 11
    #define PULSES_PER_REV_M1 (11 * 71)

#elif IS_BOT_BOARD
    // --- BOTTOM BOARD (3 Joints, MAX14870 PWM/DIR) ---
    #define MOTOR_2_PWM_PIN 0
    #define DIR_PIN_2       1
    #define MOTOR2_PIN      2

    #define MOTOR3_PIN      10   // j1 FG (tacho). Moved off GP7 — GP7 is shorted to GND on the PCB.
    #define MOTOR_3_PWM_PIN 8
    #define DIR_PIN_3       9

    #define MOTOR_4_PWM_PIN 4
    #define DIR_PIN_4       5
    #define MOTOR4_PIN      3

    #define SPI_CS_PIN_0    13
    #define SPI_CS_PIN_1    26

    #define PULSES_PER_REV_M2 (11 * 337 * 4)
    #define PULSES_PER_REV_M3 (6 * 45 * 3)
    #define PULSES_PER_REV_M4 (11 * 56 * 1)

    #define SERVO_PIN      6
    #define SERVO_WRAP     19999    // 250 MHz / 250 clkdiv / 20000 = 50 Hz (1 count = 1 µs)
    #define SERVO_CLK_DIV  250.0f

    // Wheel geometry — 80 mm diameter drive wheel
    #define WHEEL_RADIUS_M  0.04f     // 40 mm radius
#endif

// Encoder state for each motor (output shaft reference)
typedef struct {
    uint pin;                          // encoder pin or identifier
    volatile uint32_t pulse_count;     // pulses accumulated by ISR since last drain
    volatile uint32_t total_pulses;    // cumulative ISR edge count, NEVER drained (debug: proves edges arrive)
    absolute_time_t  last_time;        // last service time (can be used for housekeeping)

    // Fixed-window velocity estimation (spans multiple calls)
    absolute_time_t  win_start;        // timestamp when current velocity window started [1]
    int32_t          win_pulses;       // pulses accumulated within the current window [1]
    float            last_vel;         // last COMPUTED velocity — held while the window fills
} MotorEncoder;

// --- PID State Variables for Position and Velocity ---
extern float integral_pos[num_joints];
extern float prev_error_pos[num_joints];
extern float integral_vel[num_joints];
extern float prev_error_vel[num_joints];

// REPLACE WITH THIS:
#if IS_TOP_BOARD
    static float MIN_PWM_MOVE[num_joints]  = {0.0, 35.0};
    static float MIN_PWM_HOLD[num_joints]  = {0.0, 0.0};
    static bool  GRAVITY_HOLD[num_joints]  = {false, true};
#elif IS_BOT_BOARD
    static float MIN_PWM_MOVE[num_joints]  = {15.0, 20.0, 10.0};
    static float MIN_PWM_HOLD[num_joints]  = {0.0, 0.0, 10.0};
    static bool  GRAVITY_HOLD[num_joints]  = {false, false, false};
#endif

// #define PWM_WRAP_VALUE 65535 // 16-bit PWM counter range

#define MOTOR_SYSCLK_HZ     250000000   // FreeRTOS core clock
#define PWM_TARGET_HZ       33333       // ~33 kHz — brushed motors on H-bridges
#define PWM_WRAP_VALUE      ((MOTOR_SYSCLK_HZ / PWM_TARGET_HZ) - 1)

#define FIT0441_TARGET_HZ   25000
#define FIT0441_CLKDIV      ((float)PWM_TARGET_HZ / (float)FIT0441_TARGET_HZ)

// #define PWM_WRAP_VALUE 7499

// PID
extern volatile float Kp_pos[num_joints];
extern volatile float Ki_pos[num_joints];
extern volatile float Kd_pos[num_joints];

extern volatile float Kp_vel[num_joints];
extern volatile float Ki_vel[num_joints];
extern volatile float Kd_vel[num_joints];

extern volatile float g_pwm_max[num_joints];

void init_spi();

void setup_outputs();

void init_pwm();

void emergency_stop(void);

float pid_compute(int joint_index, float target_position, float current_position);

void motor_control(int motor_id, bool enable, bool direction, float pwm_value);

float read_encoder_angle(int id);

#define GLITCH_MAX_REJECT 5

extern volatile uint16_t g_enc_raw[2];     // last raw 14-bit SPI word (0 = dead bus)
extern volatile uint16_t g_enc_reject[2];  // consecutive glitch rejections

#define ENC_RAW_STUCK_LOW    0x0000
#define ENC_RAW_STUCK_HIGH   0x3FFF

#define ENC_BAD_MAX          10

#define ENC_RECOVER_EVERY    500

extern volatile uint16_t g_enc_bad[2];       // consecutive invalid raw words
extern volatile uint8_t  g_enc_fault[2];     // 1 = encoder declared dead
extern volatile uint32_t g_enc_recoveries;   // count of SPI re-inits attempted
extern volatile uint8_t  g_enc_recover_req;  // 1 = a re-init is pending

void spi_encoders_service_recovery(void);

bool encoder_healthy(int id);

void spi_encoders_recover(void);

extern volatile uint8_t  g_last_dir[num_joints];    // DIR pin state, 0/1
extern volatile uint8_t  g_last_en[num_joints];     // enable flag, 0/1
extern volatile float    g_last_pwm[num_joints];    // PWM the loop asked for, 0-100
extern volatile uint16_t g_last_level[num_joints];  // level on the pin (j1: INVERTED duty)

static void encoder_callback(uint gpio, uint32_t events) ;

static inline int motor_index_from_gpio(uint gpio);

// Initialize GPIOs and interrupts for all encoders
void encoders_init() ;

// Returns angular velocity in rad/s for the specified motor
float get_motor_velocity(uint motor_id) ;

uint32_t get_total_pulses(uint motor_id) ;

float angular_error(float target, float current) ;

float position_pid(int joint_index, float target, float current, float dt);

float velocity_pid(int joint_index, float target, float current, float dt);

float wrap_pi(float x);           // if not static for cross-TU use [1]

#if IS_BOT_BOARD
void init_servo(uint pin);
void servo_write_rad(uint pin, float angle_rad);   // 0…π → 500…2500 µs
#endif

#endif // HW_H