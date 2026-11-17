#include "HW.h"
#include <cstdio>

#if IS_TOP_BOARD
    const float MAX_ANGLE[num_joints] = {300.0, 244.8};
    const float MIN_ANGLE[num_joints] = {132.0, 77.6};
    const float MID_ANGLE[num_joints] = {0.0, 0.0};
#elif IS_BOT_BOARD
    // (Add the 3rd joint's min/max/mid values here)
    const float MAX_ANGLE[num_joints] = {300.0, 244.8, 300.0};
    const float MIN_ANGLE[num_joints] = {132.0, 77.6, 132.0};
    const float MID_ANGLE[num_joints] = {0.0, 0.0, 0.0};
#endif

const float CONVERSION_SCALE = 3.0;

float integral[num_joints] = {0};                  // Integral term for each joint
float previous_error[num_joints] = {0.0};          // Previous error for each joint

// Limits for integral term to avoid windup
const float integral_max = 100.0;
const float integral_min = -100.0;

#define DEBUG_PID 0

#if IS_TOP_BOARD
MotorEncoder motors[num_joints] = {
    {.pin = MOTOR0_PIN},
    {.pin = MOTOR1_PIN},
};
#elif IS_BOT_BOARD
MotorEncoder motors[num_joints] = {
    {.pin = MOTOR2_PIN},
    {.pin = MOTOR3_PIN},
    {.pin = MOTOR4_PIN},
};
#endif

// --- PID State Variables for Position and Velocity ---
float integral_pos[num_joints] = {0};
float prev_error_pos[num_joints] = {0};
float integral_vel[num_joints] = {0};
float prev_error_vel[num_joints] = {0};

#if IS_TOP_BOARD
    volatile float Kp_pos[num_joints] = {0.0f, 0.0f};
    volatile float Ki_pos[num_joints] = {0.0f, 0.0f};
    volatile float Kd_pos[num_joints] = {0.0f, 0.0f};
    volatile float Kp_vel[num_joints] = {0.0f, 0.0f};
    volatile float Ki_vel[num_joints] = {0.0f, 0.0f};
    volatile float Kd_vel[num_joints] = {0.0f, 0.0f};
#elif IS_BOT_BOARD
    volatile float Kp_pos[num_joints] = {0.0f, 0.0f, 0.0f};
    volatile float Ki_pos[num_joints] = {0.0f, 0.0f, 0.0f};
    volatile float Kd_pos[num_joints] = {0.0f, 0.0f, 0.0f};
    volatile float Kp_vel[num_joints] = {0.0f, 0.0f, 0.0f};
    volatile float Ki_vel[num_joints] = {0.0f, 0.0f, 0.0f};
    volatile float Kd_vel[num_joints] = {0.0f, 0.0f, 0.0f};
#endif
uint16_t start_count = 0;

// Initialize SPI
void init_spi() {

    spi_init(SPI_PORT_1, 10 * 1000 * 1000);

    spi_set_format(SPI_PORT_1, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(SPI_CLK_PIN_1,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN_1, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN_1, GPIO_FUNC_SPI);

    // Initialize Sensor 0 CS (Pin 13)
    gpio_init(SPI_CS_PIN_0);
    gpio_set_dir(SPI_CS_PIN_0, GPIO_OUT);
    gpio_put(SPI_CS_PIN_0, 1); // Start Inactive (High)

    // Initialize Sensor 1 CS (Pin 11)
    gpio_init(SPI_CS_PIN_1);
    gpio_set_dir(SPI_CS_PIN_1, GPIO_OUT);
    gpio_put(SPI_CS_PIN_1, 1); // Start Inactive (High)
}

// REPLACE YOUR setup_outputs() WITH THIS:
void setup_outputs(){
    gpio_init(MOTOR_DEBUG_PIN);
    gpio_set_dir(MOTOR_DEBUG_PIN, GPIO_OUT);

    gpio_init(COMMS_DEBUG_PIN);
    gpio_set_dir(COMMS_DEBUG_PIN, GPIO_OUT);
}

void emergency_stop() {
    // Dynamically stop however many motors this board has
    for (int i = 0; i < num_joints; i++) {
        motor_control(i, false, false, 0.0f);
    }
}

void init_pwm() {
    #if IS_TOP_BOARD
        // --- Motor 0: Standard Logic (GP0, GP1) ---
        gpio_set_function(MOTOR_0_PWM_PIN, GPIO_FUNC_PWM);
        uint slice0 = pwm_gpio_to_slice_num(MOTOR_0_PWM_PIN);
        pwm_set_wrap(slice0, PWM_WRAP_VALUE);
        pwm_set_clkdiv(slice0, 1.0f);
        pwm_set_gpio_level(MOTOR_0_PWM_PIN, 0);
        pwm_set_enabled(slice0, true);

        gpio_init(DIR_PIN_0);
        gpio_set_dir(DIR_PIN_0, GPIO_OUT);

        // --- Motor 1: BTS7960 PWM Logic (GP4, GP5) ---
        uint m1_pins[] = {PIN_M1_RPWM, PIN_M1_LPWM};
        for (int i = 0; i < 2; i++) {
            gpio_set_function(m1_pins[i], GPIO_FUNC_PWM);
            uint slice = pwm_gpio_to_slice_num(m1_pins[i]);
            pwm_set_wrap(slice, PWM_WRAP_VALUE);
            pwm_set_clkdiv(slice, 1.0f);
            pwm_set_gpio_level(m1_pins[i], 0);
            pwm_set_enabled(slice, true);
        }

        // REN and LEN hard-wired to 3.3V on the board

    #elif IS_BOT_BOARD
        // --- Motors 2, 3, 4: Standard Logic (PWM + DIR) ---
        uint bot_pwm_pins[] = {MOTOR_2_PWM_PIN, MOTOR_3_PWM_PIN, MOTOR_4_PWM_PIN};
        uint bot_dir_pins[] = {DIR_PIN_2, DIR_PIN_3, DIR_PIN_4};

        for (int i = 0; i < 3; i++) {
            gpio_set_function(bot_pwm_pins[i], GPIO_FUNC_PWM);
            uint slice = pwm_gpio_to_slice_num(bot_pwm_pins[i]);
            pwm_set_wrap(slice, PWM_WRAP_VALUE);

            pwm_set_clkdiv(slice, (i == 1) ? FIT0441_CLKDIV : 1.0f);

            pwm_set_gpio_level(bot_pwm_pins[i], 0);
            pwm_set_enabled(slice, true);

            gpio_init(bot_dir_pins[i]);
            gpio_set_dir(bot_dir_pins[i], GPIO_OUT);
        }
    #endif
}

volatile uint16_t g_enc_raw[2]    = {0, 0};
volatile uint16_t g_enc_reject[2] = {0, 0};

volatile uint16_t g_enc_bad[2]    = {0, 0};
volatile uint8_t  g_enc_fault[2]  = {0, 0};
volatile uint32_t g_enc_recoveries = 0;
volatile uint8_t  g_enc_recover_req = 0;

void spi_encoders_service_recovery(void) {
    if (!g_enc_recover_req) return;
    g_enc_recover_req = 0;
    spi_encoders_recover();
}

void spi_encoders_recover(void) {
    g_enc_recoveries++;

    gpio_put(SPI_CS_PIN_0, 1);
    gpio_put(SPI_CS_PIN_1, 1);

    spi_deinit(SPI_PORT_1);
    spi_init(SPI_PORT_1, 10 * 1000 * 1000);
    spi_set_format(SPI_PORT_1, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(SPI_CLK_PIN_1,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN_1, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN_1, GPIO_FUNC_SPI);

    gpio_init(SPI_CS_PIN_0);
    gpio_set_dir(SPI_CS_PIN_0, GPIO_OUT);
    gpio_put(SPI_CS_PIN_0, 1);

    gpio_init(SPI_CS_PIN_1);
    gpio_set_dir(SPI_CS_PIN_1, GPIO_OUT);
    gpio_put(SPI_CS_PIN_1, 1);
}

bool encoder_healthy(int id) {
    if (id < 0 || id >= 2) return true;
    return g_enc_fault[id] == 0;
}

float __not_in_flash_func(read_encoder_angle)(int id) {

    // Buffers must be uint16_t for 16-bit transfers
    uint16_t tx_buf[1] = {0xFFFF};
    uint16_t rx_buf[1] = {0x00};

    // Determine which pin to toggle based on ID
    uint cs_pin;

    if (id == 0) {
        cs_pin = SPI_CS_PIN_0;
    } else if (id == 1) {
        cs_pin = SPI_CS_PIN_1;
    } else {
        return 0.0f; // Invalid ID
    }

    // 1. Pull CS Low manually
    gpio_put(cs_pin, 0);

    spi_write16_read16_blocking(SPI_PORT_1, tx_buf, rx_buf, 1);

    // 3. Pull CS High manually
    gpio_put(cs_pin, 1);

    uint16_t raw_angle = rx_buf[0] & 0x3FFF;
    g_enc_raw[id] = raw_angle;

    static float last_angle[2] = {-1.0f, -1.0f};
    if (raw_angle == ENC_RAW_STUCK_LOW || raw_angle == ENC_RAW_STUCK_HIGH) {
        static uint16_t recover_tick[2] = {0, 0};

        if (g_enc_bad[id] < ENC_BAD_MAX) {
            g_enc_bad[id]++;
            return (last_angle[id] >= 0.0f) ? last_angle[id] : -1.0f;
        }

        // Confirmed fault — a sustained run of dead frames.
        if (!g_enc_fault[id]) {
            g_enc_fault[id]  = 1;
            recover_tick[id] = ENC_RECOVER_EVERY;   // attempt recovery immediately
        }
        if (++recover_tick[id] >= ENC_RECOVER_EVERY) {
            recover_tick[id] = 0;
            g_enc_recover_req = 1;
        }
        // Hard sentinel. Callers MUST NOT close a position loop on this.
        return -1.0f;
    }

    g_enc_bad[id]   = 0;
    g_enc_fault[id] = 0;

    float angle = (float)raw_angle * (2.0f * M_PI) / 16384.0f;

    if (last_angle[id] >= 0.0f) {
        float diff = fabsf(angle - last_angle[id]);
        if (diff > (float)M_PI) diff = 2.0f * (float)M_PI - diff;
        if (diff > 1.0f && g_enc_reject[id] < GLITCH_MAX_REJECT) {
            g_enc_reject[id]++;
            return last_angle[id];  // transient glitch — hold last good reading
        }
    }
    g_enc_reject[id] = 0;
    last_angle[id] = angle;
    return angle;
}

// --- RESTORED WORKING PWM LOGIC ---
void set_motor_pwm_pin(uint pin, float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);

    // Your exact working math mapping 0-100 logic to the hardware counter
    uint16_t level = (uint16_t)((duty_cycle / 100.0f) * PWM_WRAP_VALUE);

    // Safety Clamp to ensure we never exceed the wrap value
    if (level > PWM_WRAP_VALUE) level = PWM_WRAP_VALUE;

    pwm_set_chan_level(slice_num, channel, level);
}

volatile uint8_t  g_last_dir[num_joints]    = {0};
volatile uint8_t  g_last_en[num_joints]     = {0};
volatile float    g_last_pwm[num_joints]    = {0};   // what the loop asked for, 0-100
volatile uint16_t g_last_level[num_joints]  = {0};   // what landed on the pin, 0..PWM_WRAP_VALUE

void __not_in_flash_func(motor_control)(int motor_id, bool enable, bool direction, float pwm_value) {
    // 1. Clamp PWM value between 0 and 100
    if (pwm_value > 100.0f) pwm_value = 100.0f;
    else if (pwm_value < 0.0f) pwm_value = 0.0f;

    // 2. Scale 0.0-100.0% to your 0-7499 hardware wrap value
    uint16_t duty = (uint16_t)((pwm_value / 100.0f) * PWM_WRAP_VALUE);

    if (motor_id >= 0 && motor_id < (int)num_joints) {
        g_last_dir[motor_id] = direction ? 1 : 0;
        g_last_en[motor_id]  = enable ? 1 : 0;
        g_last_pwm[motor_id] = pwm_value;
    }

#if IS_TOP_BOARD
    // --- TOP BOARD LOGIC ---
    if (!enable) {
        if (motor_id == 0) {
            pwm_set_gpio_level(MOTOR_0_PWM_PIN, 0);
        } else if (motor_id == 1) {
            pwm_set_gpio_level(PIN_M1_RPWM, PWM_WRAP_VALUE);
            pwm_set_gpio_level(PIN_M1_LPWM, PWM_WRAP_VALUE);
        }
        return;
    }

    if (motor_id == 0) {
        // Motor 0: standard sign-magnitude
        gpio_put(DIR_PIN_0, direction);
        pwm_set_gpio_level(MOTOR_0_PWM_PIN, duty);
    }
    else if (motor_id == 1) {
        if (direction) {
            pwm_set_gpio_level(PIN_M1_RPWM, PWM_WRAP_VALUE);           // right always GND
            pwm_set_gpio_level(PIN_M1_LPWM, PWM_WRAP_VALUE - duty);    // LOW=drive, HIGH=brake
        } else {
            pwm_set_gpio_level(PIN_M1_RPWM, PWM_WRAP_VALUE - duty);    // LOW=drive, HIGH=brake
            pwm_set_gpio_level(PIN_M1_LPWM, PWM_WRAP_VALUE);           // left always GND
        }
    }

#elif IS_BOT_BOARD
    // --- BOTTOM BOARD LOGIC ---
    uint pwm_pins[] = {MOTOR_2_PWM_PIN, MOTOR_3_PWM_PIN, MOTOR_4_PWM_PIN};
    uint dir_pins[] = {DIR_PIN_2, DIR_PIN_3, DIR_PIN_4};

    if (motor_id >= 0 && motor_id < 3) {
        if (motor_id == 1) {
            uint16_t inv_duty = PWM_WRAP_VALUE - duty;  // invert: 0 speed → full duty (stop)
            if (!enable) {
                pwm_set_gpio_level(MOTOR_3_PWM_PIN, PWM_WRAP_VALUE);  // 100% = stopped
                g_last_level[1] = PWM_WRAP_VALUE;
            } else {
                gpio_put(DIR_PIN_3, direction);
                pwm_set_gpio_level(MOTOR_3_PWM_PIN, inv_duty);
                g_last_level[1] = inv_duty;
            }
        } else {
            if (!enable) {
                pwm_set_gpio_level(pwm_pins[motor_id], 0);
            } else {
                gpio_put(dir_pins[motor_id], direction);
                pwm_set_gpio_level(pwm_pins[motor_id], duty);
            }
        }
    }
#endif
}
// Helper: map GPIO pin -> motor index
static inline int motor_index_from_gpio(uint gpio) {
    for (int i = 0; i < num_joints; ++i) {
        if (motors[i].pin == gpio) return i;
    }
    return -1; // not found
}

// ISR: use gpio to find motor, then increment the right counter
static void encoder_callback(uint gpio, uint32_t events) {
    int id = motor_index_from_gpio(gpio);
    if (id < 0) return;
    if (events & (GPIO_IRQ_EDGE_RISE /*| GPIO_IRQ_EDGE_FALL*/)) {
        motors[id].pulse_count++;
        motors[id].total_pulses++;   // cumulative, never drained — debug proof that edges arrive
    }
}

// Initialize GPIOs and interrupts for all encoders
void encoders_init() {
    for (int i = 0; i < num_joints; i++) {
        gpio_init(motors[i].pin);
        gpio_set_dir(motors[i].pin, GPIO_IN);
        gpio_pull_up(motors[i].pin);
        // Only the first call sets the callback, others just enable the IRQ
        if (i == 0) {
            gpio_set_irq_enabled_with_callback(motors[i].pin, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
        } else {
            gpio_set_irq_enabled(motors[i].pin, GPIO_IRQ_EDGE_RISE, true);
        }
        motors[i].last_time = get_absolute_time();
    }
}

volatile int32_t ENCODER_WIN_US = 20000; // measurement window in microseconds (20 ms)
float __not_in_flash_func(get_motor_velocity)(uint motor_id) {
    // Safety check: Prevent accessing out-of-bounds motors
    if (motor_id >= num_joints) return 0.0f;

    absolute_time_t now = get_absolute_time();

    // Lazy init of window
    if (to_us_since_boot(motors[motor_id].win_start) == 0) {
        motors[motor_id].win_start  = now;
        motors[motor_id].win_pulses = 0;
    }

    // Atomically drain new pulses since last call
    uint32_t new_pulses;
    uint32_t save = save_and_disable_interrupts();
    new_pulses = motors[motor_id].pulse_count;
    motors[motor_id].pulse_count = 0;
    restore_interrupts(save);

    // Accumulate into the window
    motors[motor_id].win_pulses += (int32_t)new_pulses;

    // Elapsed time of the measurement window (spans multiple calls)
    int64_t dt_window_us = absolute_time_diff_us(motors[motor_id].win_start, now);

    if (dt_window_us < ENCODER_WIN_US) {
        return motors[motor_id].last_vel;
    }

    if (dt_window_us > 0) {
        float dt = (float)dt_window_us / 1e6f;
        int32_t total = motors[motor_id].win_pulses;

        // Reset window start and accumulator for the next interval
        motors[motor_id].win_start  = now;
        motors[motor_id].win_pulses = 0;

        float ppr = 0.0f;
#if IS_TOP_BOARD
        if      (motor_id == 0) ppr = (float)PULSES_PER_REV_M0;
        else if (motor_id == 1) ppr = (float)PULSES_PER_REV_M1;
#elif IS_BOT_BOARD
        if      (motor_id == 0) ppr = (float)PULSES_PER_REV_M2;
        else if (motor_id == 1) ppr = (float)PULSES_PER_REV_M3;
        else if (motor_id == 2) ppr = (float)PULSES_PER_REV_M4;
#endif
        if (ppr > 0.0f) {
            float v = (2.0f * (float)M_PI * (float)total) / (ppr * dt);
            motors[motor_id].last_vel = v;   // latch: held by the guard above until the next pulse
            return v;
        }
    }

    motors[motor_id].last_vel = 0.0f;
    return 0.0f;
}

uint32_t get_total_pulses(uint motor_id) {
    if (motor_id >= num_joints) return 0;
    return motors[motor_id].total_pulses;
}
#if IS_BOT_BOARD
void init_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_wrap(&c, SERVO_WRAP);
    pwm_config_set_clkdiv_int_frac(&c, (uint8_t)SERVO_CLK_DIV, 0);
    pwm_init(slice, &c, true);
}

void __not_in_flash_func(servo_write_rad)(uint pin, float angle_rad) {
    if (angle_rad < 0.0f) angle_rad = 0.0f;
    if (angle_rad > (float)M_PI) angle_rad = (float)M_PI;
    uint16_t level = (uint16_t)(500.0f + (2000.0f * angle_rad / (float)M_PI));
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, level);
}
#endif

float wrap_pi(float x) {
    x = fmod(x + M_PI, 2.0*M_PI);
    if (x < 0) x += 2.0*M_PI;
    return x - M_PI; // (-pi, pi] [1]
}

float angular_error(float target, float current) {
    return wrap_pi(target - current); // shortest signed error [1]
}

// --- Position PID Controller ---
float __not_in_flash_func(position_pid)(int joint_index, float target, float current, float dt) {
    // Compute signed angular error (shortest path)
    float error = angular_error(target, current);

    // --- Proportional term ---
    float Pout = Kp_pos[joint_index] * error;

    // --- Integral term with anti-windup ---
    integral_pos[joint_index] += error * dt;
    // Clamp integral to prevent windup
    if (integral_pos[joint_index] > 50) integral_pos[joint_index] = 50;
    else if (integral_pos[joint_index] < -50) integral_pos[joint_index] = -50;
    float Iout = Ki_pos[joint_index] * integral_pos[joint_index];

    // --- Derivative term ---
    float derivative = (error - prev_error_pos[joint_index]) / dt;
    float Dout = Kd_pos[joint_index] * derivative;

    // --- Update previous error for next cycle ---
    prev_error_pos[joint_index] = error;

    // --- Debug print (optional) ---
    #if DEBUG_PID
    char debug_buffer[120];
    snprintf(debug_buffer, sizeof(debug_buffer),
        "Joint %d | tgt: %.2f, cur: %.2f, err: %.2f, P: %.2f, I: %.2f, D: %.2f, vset: %.2f",
        joint_index, target, current, error, Pout, Iout, Dout, Pout + Iout + Dout);
    publish_debug(debug_buffer);
    #endif

    // --- Return velocity setpoint (can be positive or negative) ---
    return Pout + Iout + Dout;
}

// --- Velocity PID Controller ---
float __not_in_flash_func(velocity_pid)(int j, float target, float current, float dt)
{
    float error = target - current;

    // --- P, I, D ---
    float Pout = Kp_vel[j] * error;

    // Integrator with guard
    integral_vel[j] += error * dt;
    // Keep a sane raw integral bound (before applying Ki)
    if (integral_vel[j] > 200.0f) integral_vel[j] = 200.0f;
    else if (integral_vel[j] < -200.0f) integral_vel[j] = -200.0f;

    float Iout = Ki_vel[j] * integral_vel[j];

    float derivative = (error - prev_error_vel[j]) / (dt > 1e-6f ? dt : 1e-6f);
    prev_error_vel[j] = error;
    float Dout = Kd_vel[j] * derivative;

    // Unsaturated control
    float u = Pout + Iout + Dout;

    float lim = g_pwm_max[j];
    if (!(lim > 1.0f && lim <= 100.0f)) lim = 100.0f;   // fall back if unset/garbage

    float u_sat = u;
    if (u_sat >  lim) u_sat =  lim;
    else if (u_sat < -lim) u_sat = -lim;

    // --- Anti-windup back-calculation ---
    if (Ki_vel[j] > 1e-6f) {
        const float Kaw = 0.5f;
        float du = u_sat - u;
        integral_vel[j] += Kaw * du * dt / Ki_vel[j];
    }

    Iout = Ki_vel[j] * integral_vel[j];

    if (Pout * Iout < 0.0f && fabsf(Iout) > fabsf(Pout)) {
        integral_vel[j] *= 0.98f;          // ~200 ms to bleed off at 1 kHz
        Iout = Ki_vel[j] * integral_vel[j];
    }

    float u_aw = Pout + Iout + Dout;

    // Final saturate to the same real limit
    if (u_aw >  lim) u_aw =  lim;
    else if (u_aw < -lim) u_aw = -lim;

    return u_aw; // signed command, magnitude <= g_pwm_max[j]
}
