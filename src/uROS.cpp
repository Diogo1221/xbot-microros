
#include "uROS.h"
#include "HW.h"
#include "memory.h"
#include "eth_w5500.h"
#include "board_config.h"

// Connection layer (WiFi OR W5500 Ethernet — selected in board_config.h)
#include "connection.h"

#ifndef USE_WIFI
// Transport Layer (Custom implementation for W5500/Serial)
#include "transport_hybrid.h"
#include "agent_discovery.h"
#include "wizchip_conf.h"
#include "socket.h"
#endif /* USE_WIFI */

// --- LEG 1 ---
#if BOARD_ID == 11
    const char *node_name               = "L1_TOP";
    const char *subscription_topic_name = "/L1_TOP_state";
    const char *offset_topic_name       = "/L1_TOP_offset";
    const char *feedback_topic_name     = "/L1_TOP_feedback";
    const char *pid_topic_name          = "/L1_TOP_PID";
    const char *pid_feedback_name       = "/L1_TOP_PID_feedback";
    const char *config_topic_name       = "/L1_TOP_config";
    const char *diag_topic_name         = "/L1_TOP_diag";
#elif BOARD_ID == 12
    const char *node_name               = "L1_BOT";
    const char *subscription_topic_name = "/L1_BOT_state";
    const char *offset_topic_name       = "/L1_BOT_offset";
    const char *feedback_topic_name     = "/L1_BOT_feedback";
    const char *pid_topic_name          = "/L1_BOT_PID";
    const char *pid_feedback_name       = "/L1_BOT_PID_feedback";
    const char *config_topic_name       = "/L1_BOT_config";
    const char *diag_topic_name         = "/L1_BOT_diag";

// --- LEG 2 ---
#elif BOARD_ID == 21
    const char *node_name               = "L2_TOP";
    const char *subscription_topic_name = "/L2_TOP_state";
    const char *offset_topic_name       = "/L2_TOP_offset";
    const char *feedback_topic_name     = "/L2_TOP_feedback";
    const char *pid_topic_name          = "/L2_TOP_PID";
    const char *pid_feedback_name       = "/L2_TOP_PID_feedback";
    const char *config_topic_name       = "/L2_TOP_config";
    const char *diag_topic_name         = "/L2_TOP_diag";
#elif BOARD_ID == 22
    const char *node_name               = "L2_BOT";
    const char *subscription_topic_name = "/L2_BOT_state";
    const char *offset_topic_name       = "/L2_BOT_offset";
    const char *feedback_topic_name     = "/L2_BOT_feedback";
    const char *pid_topic_name          = "/L2_BOT_PID";
    const char *pid_feedback_name       = "/L2_BOT_PID_feedback";
    const char *config_topic_name       = "/L2_BOT_config";
    const char *diag_topic_name         = "/L2_BOT_diag";

// --- LEG 3 ---
#elif BOARD_ID == 31
    const char *node_name               = "L3_TOP";
    const char *subscription_topic_name = "/L3_TOP_state";
    const char *offset_topic_name       = "/L3_TOP_offset";
    const char *feedback_topic_name     = "/L3_TOP_feedback";
    const char *pid_topic_name          = "/L3_TOP_PID";
    const char *pid_feedback_name       = "/L3_TOP_PID_feedback";
    const char *config_topic_name       = "/L3_TOP_config";
    const char *diag_topic_name         = "/L3_TOP_diag";
#elif BOARD_ID == 32
    const char *node_name               = "L3_BOT";
    const char *subscription_topic_name = "/L3_BOT_state";
    const char *offset_topic_name       = "/L3_BOT_offset";
    const char *feedback_topic_name     = "/L3_BOT_feedback";
    const char *pid_topic_name          = "/L3_BOT_PID";
    const char *pid_feedback_name       = "/L3_BOT_PID_feedback";
    const char *config_topic_name       = "/L3_BOT_config";
    const char *diag_topic_name         = "/L3_BOT_diag";

// --- LEG 4 ---
#elif BOARD_ID == 41
    const char *node_name               = "L4_TOP";
    const char *subscription_topic_name = "/L4_TOP_state";
    const char *offset_topic_name       = "/L4_TOP_offset";
    const char *feedback_topic_name     = "/L4_TOP_feedback";
    const char *pid_topic_name          = "/L4_TOP_PID";
    const char *pid_feedback_name       = "/L4_TOP_PID_feedback";
    const char *config_topic_name       = "/L4_TOP_config";
    const char *diag_topic_name         = "/L4_TOP_diag";
#elif BOARD_ID == 42
    const char *node_name               = "L4_BOT";
    const char *subscription_topic_name = "/L4_BOT_state";
    const char *offset_topic_name       = "/L4_BOT_offset";
    const char *feedback_topic_name     = "/L4_BOT_feedback";
    const char *pid_topic_name          = "/L4_BOT_PID";
    const char *pid_feedback_name       = "/L4_BOT_PID_feedback";
    const char *config_topic_name       = "/L4_BOT_config";
    const char *diag_topic_name         = "/L4_BOT_diag";

#else
    #error "BOARD_ID is not valid! Please set it to 11, 12, 21, 22, etc."
#endif

// Global Debug Topic (Shared by all legs)
const char *debug_topic_name        = "/LEG_DEBUG";

#define USE_METHODOLOGY_B 0

enum TransportMode { MODE_SERIAL, MODE_WIFI, MODE_ETH };

#ifdef USE_WIFI
TransportMode current_mode = MODE_WIFI;
#else
TransportMode current_mode = MODE_ETH;
#endif

// --- W5500 Hardware Pins ---
#define PIN_RST 20  // GP20 (Physical Pin 26)

// --- Micro-ROS UDP Configuration ---
#define UROS_SOCK_NUM   0
#define MY_PORT         4444
#define AGENT_PORT      8888

// Agent IP: 192.168.1.100
#define AGENT_IP_1      192
#define AGENT_IP_2      168
#define AGENT_IP_3      1
#define AGENT_IP_4      100

// Flag for transport initialization status
static bool transport_initialized = false;
volatile bool g_network_up = false; // Flag needed by init_uROS

#define CONTROL_LOOP_FREQ_HZ    1000  // Core 1 Physics Loop — 1 kHz.
#define UROS_SPIN_FREQ_HZ       1000  // Core 0 Comms Loop
#define FEEDBACK_PUBLISH_HZ     200  // Telemetry Rate

#define CTRL_RATE_SCALE (1000.0f / (float)CONTROL_LOOP_FREQ_HZ)   // 0.5 at 2 kHz

// Calculate dividers (for downsampling if needed)
const uint32_t FEEDBACK_DIVIDER = UROS_SPIN_FREQ_HZ / FEEDBACK_PUBLISH_HZ;

unsigned int num_handles = 4 /*Subs*/ + 0 /*Timer*/ + 0 /*Service*/;

#if IS_TOP_BOARD
volatile float g_pwm_max[2] = {80.0f, 70.0f};
#elif IS_BOT_BOARD
volatile float g_pwm_max[3] = {95.0f, 90.0f, 80.0f};
#endif
volatile float g_k_brake       = 12.0f;   // (unused by the EM brake; kept for config compatibility)
volatile float g_brake_dist    = 0.12f;   // rad: engage the brake only within this of target
volatile float g_brake_vel_min = 0.05f;   // rad/s over vel_set: overspeed margin before braking
volatile float g_hold_thresh   = 0.06f;   // (still unused)
float *config_values = NULL;

#if IS_TOP_BOARD
    const float MARGIN[num_joints] = {0.005f, 0.005f};
    const float range[num_joints]  = {M_PI, M_PI};
    const float VEL_MAX[num_joints] = {3.0f, 3.0f};
    const float SLEW_STEP_UP[num_joints] = {50.0f * CTRL_RATE_SCALE, 50.0f * CTRL_RATE_SCALE};
#elif IS_BOT_BOARD
    const float MARGIN[num_joints] = {0.005f, 0.040f, 0.005f};   // j1: 0.10 -> 0.04 (was parking 5.7deg short on the deadband edge)
    const float range[num_joints]  = {M_PI, M_PI, M_PI};
    const float VEL_MAX[num_joints] = {0.55f, 0.8f, 100.0f};

    const float SLEW_STEP_UP[num_joints] = {50.0f * CTRL_RATE_SCALE, 0.5f * CTRL_RATE_SCALE, 50.0f * CTRL_RATE_SCALE};

    #define SERVO_POS_OFF_DEG  0.0f
    #define SERVO_POS_ON_DEG   100.0f
    #define SERVO_POS_OFF_RAD  (SERVO_POS_OFF_DEG * (float)M_PI / 180.0f)
    #define SERVO_POS_ON_RAD   (SERVO_POS_ON_DEG  * (float)M_PI / 180.0f)
#endif

// static SemaphoreHandle_t timer_sem; // (Legacy/Unused)

#define UROS_TASK_TIME_DEBUG                    0
#define TARGET_CALLBACK_TIME_DEBUG              0
#define MOTOR_TASK_TIME_DEBUG                   0
#define ANGLE_READ_TIME_DEBUG                   0
#define GET_TARGET_FROM_VARIABLES_TIME_DEBUG    0
#define MOTOR_CONTROL_TIME_DEBUG                0
#define UPDATE_FEEDBACK_VARIABLES_TIME_DEBUG    0

// Profiling Variables
volatile uint32_t exec_time_us = 0;
volatile float    measured_freq = 0.0f;
volatile uint32_t target_callback_time_us = 0;
volatile float    target_callback_measured_freq = 0.0f;

rcl_node_t       node;
rcl_allocator_t  allocator;
rclc_support_t   support;
rclc_executor_t  executor;

// --- Quality of Service Profiles ---
rmw_qos_profile_t qos_profile  = rmw_qos_profile_default; // Best Effort
rmw_qos_profile_t qos_profile1 = rmw_qos_profile_default; // Reliable + Transient Local (PID)
rmw_qos_profile_t qos_profile2 = rmw_qos_profile_default; // Reliable + VOLATILE   (offset/calibrate)

// --- Publishers ---
rcl_publisher_t debug_uROS;
rcl_publisher_t pid_feedback_publisher;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t diag_publisher;  // per-board diagnostics (4F.I)

// --- Subscribers ---
rcl_subscription_t leg_controller_subscriber;
rcl_subscription_t offset_subscriber;
rcl_subscription_t pid_subscriber;
rcl_subscription_t config_subscriber;

// --- Messages ---
std_msgs__msg__String            debug_msg;
std_msgs__msg__Float32           msg;
std_msgs__msg__Float32           pid_msg;
std_msgs__msg__Float32MultiArray feedback_msg;
std_msgs__msg__Float32MultiArray leg_controller_state;
std_msgs__msg__Float32MultiArray offset_message;
std_msgs__msg__Float32MultiArray pid_message;
std_msgs__msg__Float32MultiArray pid_message_sub;
std_msgs__msg__Float32MultiArray config_message_sub;
std_msgs__msg__Float32MultiArray diag_msg;
static float diag_data[4 + 2 * num_joints] = { 0.0f };

volatile uint32_t g_sat_count[num_joints] = { 0 };   // rising-edge count of PWM-cap saturation
volatile float    g_pwm_mag[num_joints]   = { 0.0f };// latest applied |PWM| per motor

// Dynamic Arrays (Allocated in init)
float *feedback_positions = NULL;
float *feedback_positions_temp = NULL;
float *target_positions = NULL;
float *offset_positions = NULL;
float *pid_values = NULL;
float *pid_feedback_values = NULL;

// Synchronization & Control State
volatile float target_top_joint = 0.0;
volatile float target_top_leg_joint = 0.0;
volatile float target_joints_positions[CMD_MSG_SIZE] = {0.0};
volatile float offset[num_joints] = {0.0};
float joint_limits[num_joints][2] = {0.0};

#define MOTOR_PWM_MAX g_pwm_max

// Motor Physics State
static float min_pwm_move_runtime[num_joints] = {0.0};
static uint16_t stuck_counts[num_joints] = {0};
volatile bool motor1_enabled_flag = false;
volatile int  motor1_last_dir     = 0;
#if IS_TOP_BOARD
volatile bool joint1_coast = false;
#endif

// Joint 1 Multi-turn State
volatile int32_t joint1_rev_idx = 0;
static float     joint1_last_wrapped = 0.0;
static bool      joint1_inited = false;
volatile bool    g_zone_seeded = false;

volatile float   g_j1_sun   = 0.0f;   // sun_wrapped, the raw input to the carrier maths
volatile float   g_j1_mech  = 0.0f;   // mech_pos[1], the UNWRAPPED position the PID sees
volatile uint8_t g_j1_flags = 0;      // b0 stall_flag, b1 joint1_inited, b2 g_zone_seeded
volatile float   g_j1_cl    = 0.0f;   // carrier_local — the pre-divide accumulator, [0, 6pi)
volatile uint8_t g_j1_zo    = 0;      // zone_offset — extra whole sun-turns folded in
volatile float   g_j1_dsun  = 0.0f;   // last d_sun — the wrap detector's input
volatile int     g_j1_fg_lvl      = 1;      // debug: instantaneous raw level of GP10 (FG pin)
volatile bool    g_j1_fg_low_seen = false;  // debug: latched TRUE if GP10 ever read LOW since last print
volatile float   g_wheel_duty     = 0.0f;   // debug: signed wheel (j2) duty actually driven this cycle
volatile float   g_j1_pwm_hz = (float)FIT0441_TARGET_HZ;  // live j1 PWM carrier, config[7]

// Time Synchronization (Agent Wall Clock - Pico Hardware Clock)
volatile int64_t g_clock_offset_us = 0;

volatile bool g_loopback_mode = false;

volatile bool g_pwm_apply_pending = false;

volatile bool     g_lat_high    = false;   // GP27 raised, awaiting timed clear (Core 0 loop)
volatile uint32_t g_lat_high_us = 0;
volatile bool     g_pwm_high    = false;   // GP28 raised, awaiting timed clear (Core 1 loop)
volatile uint32_t g_pwm_high_us = 0;

volatile int64_t  g_lat_tc0_ns      = 0;   // Core 0 receive time, epoch ns (synced)
volatile int64_t  g_lat_tc1_ns      = 0;   // Core 1 PWM-apply time, epoch ns (synced)
volatile double   g_lat_tsend_sec   = 0;   // command t_sec (integer s, mod 100000, = emit + LEAD)
volatile double   g_lat_tsend_frac  = 0;   // command t_frac (fractional s)
volatile bool     g_lat_c0_report   = false;  // Core 0 receive report pending (LAT_C0)
volatile bool     g_lat_await_c1    = false;  // Core 1 stamp pending
volatile bool     g_lat_c1_report   = false;  // Core 1 apply report pending (LAT_C1)
volatile uint32_t g_lat_seq         = 0;    // one seq per command; shared by the C0 & C1 reports
volatile float    g_lat_transport_us = 0.0f; // transport (tc0-tsend+LEAD, µs) — piggybacks on feedback
volatile float    g_lat_sched_us     = 0.0f; // sched (tc1-tc0, µs) — piggybacks on feedback

static volatile uint32_t g_fb_seq = 0;

#define DEBUG_MOTOR      1
#if LATENCY_CB_DEBUG
#define DEBUG_MOTOR_MS   20    // 50 Hz (LAT_CB flood now runtime-off, so no crowding)
#else
#define DEBUG_MOTOR_MS   100   // 10 Hz (normal live-watch build)
#endif

volatile bool g_debug_motor = false;

volatile bool g_lat_cb_debug = false;

struct MotorDbg {
    const char *state;   // why the motor is (or isn't) driving
    float       target;
    float       pos;     // unwrapped mech_pos — frozen here = dead encoder
    float       err;
    float       vel;
    float       pwm;     // signed: + = dir false, - = dir true; 0 = motor off
    uint32_t    stall;   // counts to STALL_TIMEOUT_TICKS, then latches off
};
static MotorDbg g_mdbg[num_joints] = {};

volatile bool     g_offset_reanchor = false;

// Watchdogs & Flags
volatile bool     new_offset_received = false;
volatile uint32_t g_last_msg_ms = 0;
uint64_t now;
int count = 0;
int COUNT_2 = 0;
int first = 1;

volatile float latest_target_timestamp = 0.0;
static float   buffered_target_pos[num_joints] = {0.0f};
static float   buffered_timestamp = 0.0f;
static bool    has_valid_buffer = false;
static bool    hold_buffer_methodB = false;

// Micro-ROS Initialization State Machine
enum {
    STATE_WAITING_AGENT,
    STATE_SUPPORT_INITED,
    STATE_NODE_INITED,
    STATE_ENTITIES_INITED,
    STATE_EXECUTOR_INITED,
    STATE_READY
} uros_state = STATE_WAITING_AGENT;

// Helper: Publish debug string to /LEG_DEBUG topic
void publish_debug(const char *message) {
  debug_msg.data.data = (char *)message;
  debug_msg.data.size = strlen(message);
  rcl_ret_t ret = rcl_publish(&debug_uROS, &debug_msg, NULL);
  if (ret != RCL_RET_OK) {}
}

void reset_micro_ros_state() {
    if (uros_state >= STATE_ENTITIES_INITED) {
        publish_debug("Micro-ROS: Cleaning Up...");
    }
    rcl_ret_t rc;
    if (uros_state >= STATE_EXECUTOR_INITED) {
        rclc_executor_fini(&executor);
    }

    if (uros_state >= STATE_ENTITIES_INITED) {
        // Publishers
        rc = rcl_publisher_fini(&joint_state_publisher, &node);
        rc = rcl_publisher_fini(&pid_feedback_publisher, &node);
        rc = rcl_publisher_fini(&diag_publisher, &node);
        rc = rcl_publisher_fini(&debug_uROS, &node);

        // Subscribers
        rc = rcl_subscription_fini(&leg_controller_subscriber, &node);
        rc = rcl_subscription_fini(&offset_subscriber, &node);
        rc = rcl_subscription_fini(&pid_subscriber, &node);
        rc = rcl_subscription_fini(&config_subscriber, &node);
    }

    if (uros_state >= STATE_NODE_INITED) {
        rc = rcl_node_fini(&node);
    }

    // Destroy the Support structure (Allocators, Clock, Context)
    if (uros_state >= STATE_SUPPORT_INITED) {
        rc = rclc_support_fini(&support);
    }

    free_feedback_publisher_memory();
    free_subscriber_memory();

    memset(&executor, 0, sizeof(rclc_executor_t));
    memset(&node, 0, sizeof(rcl_node_t));
    memset(&support, 0, sizeof(rclc_support_t));

    // Reset global flags to initial "Waiting" state
    uros_state = STATE_WAITING_AGENT;
    has_valid_buffer = false;
}

bool init_uROS() {
    char debug_buffer[64];

    std_msgs__msg__Float32__init(&msg);
    std_msgs__msg__String__init(&debug_msg);

    if (!transport_initialized) {

        if (current_mode == MODE_ETH) {
            printf("Transport: Hardware Ethernet (W5500)\n");
            rmw_uros_set_custom_transport(
                false,
                NULL,
                w5500_transport_open,
                w5500_transport_close,
                w5500_transport_write,
                w5500_transport_read
            );

#ifdef USE_WIFI
        } else if (current_mode == MODE_WIFI) {
            printf("Transport: WiFi UDP (CYW43)\n");
            /* Agent IP is read from the persistent flash config. */
            const conn_config_t *cfg = conn_get_config();
            printf("[UROS] Agent: %u.%u.%u.%u:%u\n",
                   cfg->agent_ip[0], cfg->agent_ip[1],
                   cfg->agent_ip[2], cfg->agent_ip[3],
                   cfg->agent_port);
            set_microros_agent_ip(NULL, 0); /* not used in WiFi path */
            rmw_uros_set_custom_transport(
                false,
                NULL,
                wifi_transport_open,
                wifi_transport_close,
                wifi_transport_write,
                wifi_transport_read
            );
#endif /* USE_WIFI */

        } else {
            printf("Transport: Serial mode (Fallback)\n");
            rmw_uros_set_custom_transport(
                true,
                NULL,
                hybrid_transport_open,
                hybrid_transport_close,
                hybrid_transport_write,
                hybrid_transport_read
            );
        }

        transport_initialized = true;
    }

    allocator = rcl_get_default_allocator();

#if XBOT_MULTICAST_DISCOVERY && !defined(USE_WIFI)
    static bool discovery_tried = false;
    if (current_mode == MODE_ETH && !discovery_tried) {
        uint8_t d_ip[4];
        uint16_t d_port;
        discovery_tried = true;
        if (agent_discover_multicast(0, 4444, 3, 500, d_ip, &d_port)) {
            set_discovered_agent(d_ip, d_port);
            printf("[Init] Agent discovered: %u.%u.%u.%u:%u\n", d_ip[0], d_ip[1], d_ip[2], d_ip[3], d_port);
        } else {
            printf("[Init] Discovery failed, using configured agent\n");
        }
    }
#endif
    { const conn_config_t *_c = conn_get_config();
      printf("[Init] Pinging Agent %d.%d.%d.%d:%d ...\n",
             _c->agent_ip[0], _c->agent_ip[1], _c->agent_ip[2], _c->agent_ip[3],
             _c->agent_port); }

    rcl_ret_t ret = rmw_uros_ping_agent(500, 10);

    if (ret != RCL_RET_OK) {
        printf("[Init] Ping failed (Timeout 3s). Agent not reachable.\n");
        return false; // This will trigger the retry loop in vUROSTask
    }

    printf("[Init] Agent Found! Proceeding...\n");
    rclc_support_init(&support, 0, NULL, &allocator);
    uros_state = STATE_SUPPORT_INITED;
    sleep_ms(20); // Stability delay

    rclc_node_init_default(&node, node_name, "", &support);
    uros_state = STATE_NODE_INITED;
    sleep_ms(20);

    rclc_publisher_init_best_effort(
        &debug_uROS,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        debug_topic_name
    );
    sleep_ms(20);
    publish_debug("Init: Connected to Agent");

    qos_profile1.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile1.durability  = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos_profile1.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile1.depth       = 1;

    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth       = 1;

    qos_profile2.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile2.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile2.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile2.depth       = 1;

    rclc_publisher_init_best_effort(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        feedback_topic_name
    );
    sleep_ms(20);

    // PID Feedback
    rclc_publisher_init_best_effort(
        &pid_feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        pid_feedback_name
    );
    sleep_ms(20);

    diag_data[2]           = (float)BOARD_ID;   // initializer is all-zero now; set the id explicitly
    diag_msg.data.data     = diag_data;
    diag_msg.data.size     = 4 + 2 * num_joints;
    diag_msg.data.capacity = 4 + 2 * num_joints;
    diag_msg.layout.dim.size = 0; diag_msg.layout.dim.capacity = 0; diag_msg.layout.dim.data = NULL;
    rclc_publisher_init_best_effort(
        &diag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        diag_topic_name
    );
    sleep_ms(20);

    rclc_subscription_init(
        &leg_controller_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        subscription_topic_name,
        &qos_profile
    );
    sleep_ms(20);

    rclc_subscription_init(
        &offset_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        offset_topic_name,
        &qos_profile2
    );
    sleep_ms(20);

    rclc_subscription_init(
        &pid_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        pid_topic_name,
        &qos_profile1
    );
    sleep_ms(20);

    // Motor config subscriber (runtime PWM / braking / hysteresis tuning)
    rclc_subscription_init(
        &config_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        config_topic_name,
        &qos_profile
    );
    sleep_ms(20);

    uros_state = STATE_ENTITIES_INITED;

    allocate_feedback_publisher_memory();
    allocate_pid_feedback_publisher_memory();
    allocate_subscriber_memory();
    allocate_offset_subscriber_memory();
    allocate_pid_subscriber_memory();
    allocate_config_subscriber_memory();
    sleep_ms(50);

    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    uros_state = STATE_EXECUTOR_INITED;

    // --- Register Callbacks ---
    rclc_executor_add_subscription(
        &executor,
        &leg_controller_subscriber,
        &leg_controller_state,
        &leg_controller_state_callback,
        ON_NEW_DATA
    );
    sleep_ms(10);

    rclc_executor_add_subscription(
        &executor,
        &offset_subscriber,
        &offset_message,
        &offset_callback,
        ON_NEW_DATA
    );
    sleep_ms(10);

    rclc_executor_add_subscription(
        &executor,
        &pid_subscriber,
        &pid_message_sub,
        &pid_callback,
        ON_NEW_DATA
    );
    sleep_ms(10);

    rclc_executor_add_subscription(
        &executor,
        &config_subscriber,
        &config_message_sub,
        &config_callback,
        ON_NEW_DATA
    );
    sleep_ms(10);

    publish_debug("Init: Syncing Time...");

    bool sync_success = false;
    for (int i = 0; i < 5; ++i) { // 5 attempts x 100ms = 0.5s max block
        if (rmw_uros_sync_session(100) == RCL_RET_OK) {
            sync_success = true;
            publish_debug("Sync Success");
            break;
        }
        sleep_ms(10);
    }

    if (!sync_success) {
        return false;
    }

    // 1. Get current Agent Time in Nanoseconds
    int64_t agent_time_ns = rmw_uros_epoch_nanos();

    // 2. Get current Pico Hardware Time in Microseconds
    int64_t pico_time_us = time_us_64();

    // 3. Calculate Offset (Converted to Microseconds)
    g_clock_offset_us = (agent_time_ns / 1000) - pico_time_us;

    return true; // Initialization Complete
}

float __not_in_flash_func(normalize_angle_0_2pi)(float angle) {
  while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
  while (angle < 0.0f) angle += 2.0f * M_PI;
  return angle;
}

volatile float    target_callback_freq_hz = 0.0f;
volatile uint32_t target_callback_period_us = 0;

void leg_controller_state_callback(const void *callback_msg) {
    const std_msgs__msg__Float32MultiArray *received_msg = (const std_msgs__msg__Float32MultiArray *)callback_msg;

    if (!received_msg ||
        received_msg->data.size < CMD_MSG_SIZE ||
        received_msg->data.size > CMD_MSG_SIZE + 2) {
        publish_debug("Callback: Invalid message size for sync.");
        return;
    }

    g_last_msg_ms = (uint32_t)(time_us_64() / 1000);

    taskENTER_CRITICAL();
    {
        sio_hw->gpio_set = (1u << LATENCY_DEBUG_PIN);
        g_lat_high    = true;
        g_lat_high_us = (uint32_t)time_us_64();

        for (size_t i = 0; i < CMD_MSG_SIZE; i++) {
            target_joints_positions[i] = received_msg->data.data[i];
        }
        g_pwm_apply_pending = true;
#if IS_TOP_BOARD
        if (received_msg->data.size > CMD_MSG_SIZE) {
            joint1_coast = (received_msg->data.data[CMD_MSG_SIZE] >= 0.5f);
        }
        if (received_msg->data.size > CMD_MSG_SIZE + 1) {
            g_loopback_mode = (received_msg->data.data[CMD_MSG_SIZE + 1] >= 0.5f);
        }
#elif IS_BOT_BOARD
        if (received_msg->data.size > CMD_MSG_SIZE) {
            g_loopback_mode = (received_msg->data.data[CMD_MSG_SIZE] >= 0.5f);
        }
        if (received_msg->data.size > CMD_MSG_SIZE + 1 && !g_zone_seeded) {
            float z = received_msg->data.data[CMD_MSG_SIZE + 1];
            if (z >= 0.0f && z <= 2.5f) {
                g_zone_seeded  = true;
                joint1_rev_idx = (int32_t)(z + 0.5f);
                joint1_inited  = false;   // re-baseline tracking so next sensor tick latches new zone
            }
        }
#endif

        double t_sec  = received_msg->data.data[num_joints];
        double t_frac = received_msg->data.data[num_joints + 1];

        latest_target_timestamp = t_sec + t_frac;
    }
    taskEXIT_CRITICAL();

    g_lat_tc0_ns     = rmw_uros_epoch_nanos();
    g_lat_tsend_sec  = (double)received_msg->data.data[num_joints];
    g_lat_tsend_frac = (double)received_msg->data.data[num_joints + 1];
    g_lat_seq++;                 // one seq per command; both reports carry it
    g_lat_c0_report  = true;     // Core 0 loop publishes LAT_C0 (receive/transport)
    g_lat_await_c1   = true;     // Core 1 will stamp tc1, then Core 0 publishes LAT_C1

#if LATENCY_CB_DEBUG
    if (g_lat_cb_debug) {   // runtime gate (config[9]); default off so it does not flood /LEG_DEBUG
        static uint32_t g_lat_cb_count = 0;
        static char lat_dbg[96];
        snprintf(lat_dbg, sizeof(lat_dbg), "LAT_CB #%lu size=%u pin=%d",
                 (unsigned long)(++g_lat_cb_count), (unsigned)received_msg->data.size,
                 (int)LATENCY_DEBUG_PIN);
        publish_debug(lat_dbg);
    }
#endif
}

void offset_callback(const void *callback_msg) {
  const std_msgs__msg__Float32MultiArray *received_msg = (const std_msgs__msg__Float32MultiArray *)callback_msg;
  if (!received_msg || received_msg->data.size < num_joints) {
      publish_debug("Received invalid message in offset callback\n");
      return;
  }

  const bool restore = (received_msg->data.size > num_joints)
                    && (received_msg->data.data[num_joints] >= 0.5f);

  if (restore) {
      taskENTER_CRITICAL();
      for (size_t i = 0; i < num_joints; i++) {
          offset[i] = normalize_angle_0_2pi(received_msg->data.data[i]);
      }
#if IS_BOT_BOARD
      if (received_msg->data.size > num_joints + 1) {
          float z = received_msg->data.data[num_joints + 1];
          if (z >= 0.0f && z <= 2.5f) {
              joint1_rev_idx = (int32_t)(z + 0.5f);
              joint1_inited  = false;   // re-baseline the wrap tracker from the restored zone
          }
      }
      g_zone_seeded = true;   // the zone is now known; ignore any stale seed injection
#endif
      new_offset_received = true;
      g_offset_reanchor   = true;
      taskEXIT_CRITICAL();

      char rb[110];
      snprintf(rb, sizeof(rb), "offset_cb: RESTORED off1=%.3f zone=%d (no re-zero)",
               (double)offset[1], (int)joint1_rev_idx);
      publish_debug(rb);
      return;
  }

  static float last_applied[num_joints];
  static bool  have_applied = false;
  bool duplicate = have_applied;
  for (size_t i = 0; i < num_joints && duplicate; i++) {
      if (fabsf(received_msg->data.data[i] - last_applied[i]) > 1e-6f) duplicate = false;
  }
  if (duplicate) {
      publish_debug("offset_cb: duplicate payload — ignored (no re-zero)");
      return;
  }
  for (size_t i = 0; i < num_joints; i++) last_applied[i] = received_msg->data.data[i];
  have_applied = true;

  // if (xSemaphoreTake(shared_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  taskENTER_CRITICAL();
  for (size_t i = 0; i < num_joints; i++) {
      float cal_th = read_encoder_angle(i);
      if (cal_th >= 0.0f)
          offset[i] = normalize_angle_0_2pi(-cal_th);
  }
  new_offset_received = true;
  g_offset_reanchor   = true;

  taskEXIT_CRITICAL();

  {
      char ocb[96];
      snprintf(ocb, sizeof(ocb), "offset_cb: calibrate (no re-zero, zone held at %d)",
               (int)joint1_rev_idx);
      publish_debug(ocb);
  }
  // xSemaphoreGive(shared_data_mutex);

  for (size_t i = 0; i < num_joints; i++) {
      joint_limits[i][0] = 0.0;
      joint_limits[i][1] = 2.0 * M_PI;
  }
}

void pid_callback(const void * callback_msg)
{
  const std_msgs__msg__Float32MultiArray * received_msg = (const std_msgs__msg__Float32MultiArray *) callback_msg;
  if (received_msg == NULL || received_msg->data.size != 7) {
    publish_debug("pid_callback: NULL message");
    return;
  }

  const float joint_f = received_msg->data.data[0];
  const int   joint   = (int)(joint_f + 0.5f);
  if (joint < 0 || joint >= (int)num_joints) {
    publish_debug("pid_callback: invalid joint index");
    return;
  }

  const float kp_pos_in = received_msg->data.data[1];
  const float ki_pos_in = received_msg->data.data[2];
  const float kd_pos_in = received_msg->data.data[3];
  const float kp_vel_in = received_msg->data.data[4];
  const float ki_vel_in = received_msg->data.data[5];
  const float kd_vel_in = received_msg->data.data[6];

  // if (xSemaphoreTake(shared_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  taskENTER_CRITICAL();
  Kp_pos[joint] = kp_pos_in;  Ki_pos[joint] = ki_pos_in;  Kd_pos[joint] = kd_pos_in;
    Kp_vel[joint] = kp_vel_in;  Ki_vel[joint] = ki_vel_in;  Kd_vel[joint] = kd_vel_in;
  taskEXIT_CRITICAL();

  pid_message.data.data[0] = joint_f;
  pid_message.data.data[1] = kp_pos_in;
  pid_message.data.data[2] = ki_pos_in;
  pid_message.data.data[3] = kd_pos_in;
  pid_message.data.data[4] = kp_vel_in;
  pid_message.data.data[5] = ki_vel_in;
  pid_message.data.data[6] = kd_vel_in;
  pid_message.data.size    = 7;

  rcl_ret_t ret = rcl_publish(&pid_feedback_publisher, &pid_message, NULL);
  if (ret != RCL_RET_OK) {
    static uint8_t suppress = 0;
    if (suppress == 0) {
      char dbg[48];
      snprintf(dbg, sizeof(dbg), "pid_callback: publish ret=%d", (int)ret);
      publish_debug(dbg);
      suppress = 50;
    } else {
      suppress--;
    }
  }
}

void config_callback(const void * callback_msg)
{
    const std_msgs__msg__Float32MultiArray *m =
        (const std_msgs__msg__Float32MultiArray *)callback_msg;
    if (m == NULL || m->data.size < 6) {
        publish_debug("config_cb: need 6 floats");
        return;
    }
    taskENTER_CRITICAL();
    g_pwm_max[0]     = m->data.data[0];
    g_pwm_max[1]     = m->data.data[1];
#if IS_BOT_BOARD
    if (m->data.size >= 7) g_pwm_max[2] = m->data.data[6];
#endif
    g_k_brake        = m->data.data[2];
    g_brake_dist     = m->data.data[3];
    g_brake_vel_min  = m->data.data[4];
    g_hold_thresh    = m->data.data[5];
    if (m->data.size >= 9)  g_debug_motor  = (m->data.data[8] > 0.5f);
    if (m->data.size >= 10) g_lat_cb_debug = (m->data.data[9] > 0.5f);
    taskEXIT_CRITICAL();

#if IS_BOT_BOARD
    if (m->data.size >= 8) {
        float hz = m->data.data[7];
        if (hz >= 200.0f && hz <= 40000.0f) {
            float div = (float)MOTOR_SYSCLK_HZ / (hz * (float)(PWM_WRAP_VALUE + 1));
            if (div < 1.0f)   div = 1.0f;      // hardware minimum
            if (div > 255.0f) div = 255.0f;    // 8.4 fixed-point maximum
            pwm_set_clkdiv(pwm_gpio_to_slice_num(MOTOR_3_PWM_PIN), div);
            g_j1_pwm_hz = (float)MOTOR_SYSCLK_HZ / (div * (float)(PWM_WRAP_VALUE + 1));
        }
    }
#endif

    char dbg[110];
    snprintf(dbg, sizeof(dbg),
             "cfg: p0=%.0f p1=%.0f Kb=%.0f Bd=%.3f Bv=%.3f Bh=%.3f j1hz=%.0f",
             g_pwm_max[0], g_pwm_max[1],
             g_k_brake, g_brake_dist, g_brake_vel_min, g_hold_thresh,
             (double)g_j1_pwm_hz);
    publish_debug(dbg);
}

float last_carrier_local[num_joints] = {0.0};
char dbg_buf[200];

float current_joints_positions[num_joints] = {0.0};
float current_joints_velocities[num_joints] = {0.0};
// const float dt = 20.0;
const float dt = 1.0f / (float)CONTROL_LOOP_FREQ_HZ;           // loop timestep, derived from CONTROL_LOOP_FREQ_HZ — 0.5 ms at 2 kHz

float targets_local[CMD_MSG_SIZE] = {0.0};
float timestamp_local = 0.0;

float limits_min_local[num_joints] = {0.0};
float limits_max_local[num_joints] = {0.0};

const float VMIN_MOVE = 0.05;
const float VSTUCK = 0.01;
const float ERR_MOVE[num_joints] =
#if IS_BOT_BOARD
    {0.10f, 0.03f, 0.10f};
#else
    {0.10f, 0.003f};
#endif
const float STEP_INC = 0.50f * CTRL_RATE_SCALE;   // per-tick stiction ramp — scaled to be rate-independent
const float STEP_DEC = 0.35f * CTRL_RATE_SCALE;
const float PWM_MIN_CAP = 35.0;

const uint32_t STALL_TIMEOUT_TICKS  = 30u * CONTROL_LOOP_FREQ_HZ; // 30 s window (rate-independent)
const float    STALL_PWM_THRESH_STD = 70.0f; // 70 % of full range for standard joints
const float    STALL_PWM_THRESH_GRV = 70.0f; // absolute threshold for gravity joint
const float    STALL_MIN_PROGRESS   = 0.001f; // min position change (rad) per window — any motion counts

const uint32_t STALL_RETRY_OFF_TICKS   = 2u * CONTROL_LOOP_FREQ_HZ;              // 2.0 s cool-down (rate-independent)
const uint32_t STALL_RETRY_BURST_TICKS = (uint32_t)(0.4f * CONTROL_LOOP_FREQ_HZ); // 0.4 s drive burst

const float KAW_POS = 0.10;
const float KAW_VEL = 0.10;
const float I_DEADBAND_POS = 0.002;
const float I_DEADBAND_VEL = 0.02;

const float TWO_PI = 2.0 * M_PI;
const float WIN    = 0.20;
const float WINZ = 1e-3;
const float DIR_DZ      = 0.01;
const float PI_GUARD    = 0.10;

static float mech_pos[num_joints] = {0};

volatile uint32_t g_angle_read_duration_us = 0;
volatile uint32_t g_motor_control_duration_us = 0;
volatile uint32_t g_feedback_update_duration_us = 0;

void __not_in_flash_func(vMotorControlTask)(void *pvParameters) {

    init_pwm();
    init_spi();
    encoders_init();
#if IS_BOT_BOARD
    init_servo(SERVO_PIN);
#endif

    const uint64_t PERIOD_US = 1000000ull / CONTROL_LOOP_FREQ_HZ;   // 500 us at 2 kHz (Core 1 tick)
    uint64_t next_wake_time = time_us_64() + PERIOD_US;

    TickType_t prev_tick = xTaskGetTickCount();
    TickType_t curr_tick = 0;

    // Logic state variables (Dynamically sized by num_joints)
    static bool dir_prev[num_joints] = {false};
    float offset_local[num_joints] = {0.0};
    int32_t joint1_rev_idx_local = 0;

    // Slew memory (per joint) — persists across cycles
    static float last_cmd[num_joints] = {0.0f};

    // Stall protection state
    static uint32_t stall_ticks[num_joints]        = {0};
    static bool     stall_flag[num_joints]         = {false};
    static float    stall_last_target[num_joints]  = {-999.0f};
    static float    stall_pos_ref[num_joints]      = {0.0f};
    static uint32_t stall_recover_ticks[num_joints] = {0};   // cool-down / retry-burst counter

    while (true) {
        if (g_pwm_high && ((uint32_t)time_us_64() - g_pwm_high_us) >= (uint32_t)LATENCY_PULSE_US) {
            sio_hw->gpio_clr = (1u << DEBUG_PWM_APPLY_PIN);
            g_pwm_high = false;
        }

        uint64_t time_now = time_us_64();

        if (time_now > next_wake_time) {
            next_wake_time = time_now + PERIOD_US;
        }

        int64_t remaining_time = next_wake_time - time_now;

        if (remaining_time > 200) {
            sleep_us((uint32_t)(remaining_time - 150));
        }

        while (time_us_64() < next_wake_time) {
            __asm volatile ("nop");
        }

        next_wake_time += PERIOD_US;

        #if MOTOR_TASK_TIME_DEBUG
        uint64_t us_start = time_us_64();
        #endif

        sio_hw->gpio_set = (1u << MOTOR_DEBUG_PIN);

        taskENTER_CRITICAL();
        if (new_offset_received) {
            memcpy(offset_local, (void*)offset, num_joints * sizeof(float));
            joint1_rev_idx_local = joint1_rev_idx;
            new_offset_received = false;
        }
        taskEXIT_CRITICAL();

        spi_encoders_service_recovery();

        for (size_t i = 0; i < num_joints; i++) {
            float th = read_encoder_angle(i);

#if IS_BOT_BOARD
            if (i == 1) {
                if (th >= 0.0f) {
                    float sun_wrapped = normalize_angle_0_2pi(-th);
                    float carrier_wrapped = normalize_angle_0_2pi((-th) * 1.0/3.0);
                    static float joint1_last_sun_wrapped = 0.0;

                    if (!joint1_inited) {
                        joint1_last_sun_wrapped = sun_wrapped;
                        joint1_last_wrapped = carrier_wrapped;
                        joint1_inited = true;
                    } else {
                        float d_sun = sun_wrapped - joint1_last_sun_wrapped;
                        g_j1_dsun = d_sun;   // debug: the wrap detector's input
                        if (d_sun < -(float)M_PI)      joint1_rev_idx = (joint1_rev_idx + 1) % 3;  // 2π→0, forward
                        else if (d_sun > (float)M_PI)  joint1_rev_idx = (joint1_rev_idx + 2) % 3;  // 0→2π, backward

                        joint1_last_sun_wrapped = sun_wrapped;
                        joint1_last_wrapped = carrier_wrapped;
                    }

                    if (joint1_rev_idx < 0) joint1_rev_idx = 0;
                    if (joint1_rev_idx > 2) joint1_rev_idx = 2;

                    uint32_t zone_offset = 0;
                    const float off = 0.0f;
                    if (joint1_rev_idx == 0) {
                        if      (sun_wrapped > off + WINZ) zone_offset = 0;
                        else if (sun_wrapped < off - WINZ) zone_offset = 2;
                    } else if (joint1_rev_idx == 1) {
                        zone_offset = 0;
                    } else {
                        zone_offset = 1;
                    }

                    float carrier_local;
                    if (joint1_rev_idx == 0 && fabsf(sun_wrapped - off) < 0.01f) {
                        carrier_local = 0.0f;
                    } else if (joint1_rev_idx == 0 && sun_wrapped > off) {
                        carrier_local = sun_wrapped - off;
                    } else {
                        carrier_local = 2.0f * (float)M_PI - off
                                      + 2.0f * (float)M_PI * (float)zone_offset
                                      + sun_wrapped;
                    }
                    current_joints_positions[i] = carrier_local / 3.0f;
                    g_j1_sun = sun_wrapped;   // debug: raw input to the carrier maths
                    g_j1_cl  = carrier_local; // debug: pre-divide accumulator
                    g_j1_zo  = (uint8_t)zone_offset;
                }
            }
            else {
                if (th >= 0.0f) {
                    th = normalize_angle_0_2pi(th);
                    current_joints_positions[i] = th;
                }
            }
#else
            // --- TOP BOARD: Standard 1:1 reading for all joints ---
            if (th >= 0.0f) {
                th = normalize_angle_0_2pi(th);
                current_joints_positions[i] = th;
            }
#endif

            current_joints_velocities[i] = get_motor_velocity(i);
        }

#if IS_BOT_BOARD
        {
            int fg_lvl = gpio_get(MOTOR3_PIN);
            g_j1_fg_lvl = fg_lvl;
            if (fg_lvl == 0) g_j1_fg_low_seen = true;
        }
#endif

#if IS_BOT_BOARD
        // Bottom Board: Gear ratio adjustment for Joint 0
        current_joints_velocities[0] = current_joints_velocities[0] / 2.0;
#endif

        bool did_break = false;

        taskENTER_CRITICAL();
        {
            for (size_t i = 0; i < num_joints; i++) {
                targets_local[i]    = target_joints_positions[i];
                limits_min_local[i] = joint_limits[i][0];
                limits_max_local[i] = joint_limits[i][1];
            }
            targets_local[num_joints]   = target_joints_positions[num_joints];
            targets_local[num_joints+1] = target_joints_positions[num_joints+1];
#if IS_BOT_BOARD
            targets_local[num_joints+2] = target_joints_positions[num_joints+2];
#endif
            timestamp_local = latest_target_timestamp;
        }
        taskEXIT_CRITICAL();

if (g_offset_reanchor) {
    g_offset_reanchor = false;
    for (size_t j = 0; j < num_joints; j++) {
        integral_pos[j]         = 0.0f;
        prev_error_pos[j]       = 0.0f;
        integral_vel[j]         = 0.0f;
        prev_error_vel[j]       = 0.0f;
        last_cmd[j]             = 0.0f;
        min_pwm_move_runtime[j] = 0.0f;
        stuck_counts[j]         = 0;
        stall_flag[j]           = false;
        stall_ticks[j]          = 0;
        stall_last_target[j]    = targets_local[j];
        motor_control(j, false, false, 0.0f);
    }
}

#if DEBUG_MOTOR
#define DBG_MOTOR(st, p, e, v, pw)                                              \
    do {                                                                        \
        g_mdbg[i].state  = (st);                                                \
        g_mdbg[i].target = targets_local[i];                                    \
        g_mdbg[i].pos    = (p);                                                 \
        g_mdbg[i].err    = (e);                                                 \
        g_mdbg[i].vel    = (v);                                                 \
        g_mdbg[i].pwm    = (pw);                                                \
        g_mdbg[i].stall  = stall_ticks[i];                                      \
    } while (0)
#else
#define DBG_MOTOR(st, p, e, v, pw) do {} while (0)
#endif

for (size_t i = 0; i < num_joints; i++) {

    if (g_loopback_mode) {
        if (i < num_joints) current_joints_positions[i] = targets_local[i];
        motor_control(i, false, false, 0.0f);
        DBG_MOTOR("LOOPBACK", current_joints_positions[i], 0.0f,
                  current_joints_velocities[i], 0.0f);
        continue;
    }

#if IS_TOP_BOARD
    if (i == 1 && joint1_coast) {
        pwm_set_gpio_level(PIN_M1_RPWM, 0);
        pwm_set_gpio_level(PIN_M1_LPWM, 0);
        DBG_MOTOR("COAST", current_joints_positions[i], 0.0f,
                  current_joints_velocities[i], 0.0f);
        continue;
    }
#endif

#if IS_BOT_BOARD
    if (i == 2) {
        float speed = targets_local[2];
        bool  dir   = (speed >= 0.0f);
        float duty  = fabsf(speed);
        if (duty > MOTOR_PWM_MAX[2]) duty = MOTOR_PWM_MAX[2];
        g_wheel_duty = dir ? duty : -duty;   // debug: expose the wheel drive on the j1 line
        motor_control(2, duty > 0.0f, dir, duty);
        DBG_MOTOR("WHEEL", current_joints_positions[2], 0.0f,
                  current_joints_velocities[2], dir ? duty : -duty);
        continue;
    }
#endif

    // float target = 0.0;
    float target = targets_local[i];

    #if USE_METHODOLOGY_B
        double t_sec  = (double)targets_local[num_joints];
        double t_frac = (double)targets_local[num_joints+1];
        double message_timestamp = t_sec + t_frac;

        int64_t now_nanos      = rmw_uros_epoch_nanos();
        float   current_time_s = (float)now_nanos / 1e9f;

        if (!hold_buffer_methodB) {
            hold_buffer_methodB = true;
            for (int j = 0; j < num_joints; j++) buffered_target_pos[j] = targets_local[j];
            buffered_timestamp = message_timestamp;
        }

        if (hold_buffer_methodB) {
            if (current_time_s >= buffered_timestamp) {
                hold_buffer_methodB = false;
                for (int j = 0; j < num_joints; j++) current_joints_positions[j] = buffered_target_pos[j];
            }
        }
    #endif

    // Bounds Checking
    float  meas_raw = current_joints_positions[i];
    float  cur_vel  = current_joints_velocities[i];
    float  min_b    = limits_min_local[i];
    double max_b    = limits_max_local[i];

    bool out_of_bounds = false;
    if (fabs((max_b - min_b) - 2.0*M_PI) >= 1e-6) {
        if (min_b < max_b) out_of_bounds = (target < min_b || target > max_b);
        else                out_of_bounds = (target < min_b && target > max_b);
    }
    if (out_of_bounds) {
        DBG_MOTOR("OUTBOUNDS", meas_raw, 0.0f, cur_vel, 0.0f);
        did_break = true;
        break;
    }

    if (Kp_pos[i] < 0.01f && Kp_vel[i] < 0.01f) {
        integral_pos[i] = 0.0f; prev_error_pos[i] = 0.0f;
        integral_vel[i] = 0.0f; prev_error_vel[i] = 0.0f;
        last_cmd[i]     = 0.0f;
        min_pwm_move_runtime[i] = 0.0f;
        stuck_counts[i] = 0;
        motor_control(i, false, false, 0.0f);
        DBG_MOTOR("KP_ZERO", meas_raw, 0.0f, cur_vel, 0.0f);
        continue;
    }

    if (!encoder_healthy(i)) {
        integral_pos[i] = 0.0f; prev_error_pos[i] = 0.0f;
        integral_vel[i] = 0.0f; prev_error_vel[i] = 0.0f;
        last_cmd[i]     = 0.0f;
        min_pwm_move_runtime[i] = 0.0f;
        stuck_counts[i] = 0;
        motor_control(i, false, false, 0.0f);
        DBG_MOTOR("ENC_FAULT", meas_raw, 0.0f, cur_vel, 0.0f);
        continue;
    }

    float wrapped = fmodf(mech_pos[i], 2.0f * M_PI);
    if (wrapped < 0.0f) wrapped += 2.0f * M_PI;
    float delta = meas_raw - wrapped;
    if (delta >  M_PI) delta -= 2.0f * M_PI;
    else if (delta < -M_PI) delta += 2.0f * M_PI;
    mech_pos[i] += delta;

    {
        static float    vv_pos[num_joints] = {0};
        static uint64_t vv_us [num_joints] = {0};
        static bool     vv_init[num_joints] = {false};
        static bool     vv_phantom[num_joints] = {false};

        const uint64_t VV_WINDOW_US = 50000;
        const float    VV_MIN_MOVE  = 0.004f;

        uint64_t now_us = time_us_64();
        if (!vv_init[i]) {
            vv_pos[i] = mech_pos[i]; vv_us[i] = now_us; vv_init[i] = true;
        }
        if ((now_us - vv_us[i]) >= VV_WINDOW_US) {
            vv_phantom[i] = (fabsf(mech_pos[i] - vv_pos[i]) < VV_MIN_MOVE);
            vv_pos[i] = mech_pos[i];
            vv_us[i]  = now_us;
        }
        // Applied BEFORE velocity_pid and is_stuck so both see the truth.
        if (vv_phantom[i]) cur_vel = 0.0f;
    }

    if (GRAVITY_HOLD[i]) {
        static float    ev_pos [num_joints] = {0};
        static uint64_t ev_us  [num_joints] = {0};
        static bool     ev_init[num_joints] = {false};
        static float    ev_last[num_joints] = {0};
        const uint64_t  EV_WINDOW_US = 12000;   // 12 ms: clean above ±4-count noise, still responsive
        uint64_t ev_now = time_us_64();
        if (!ev_init[i]) { ev_pos[i] = mech_pos[i]; ev_us[i] = ev_now; ev_init[i] = true; }
        if ((ev_now - ev_us[i]) >= EV_WINDOW_US) {
            float dts = (float)(ev_now - ev_us[i]) * 1e-6f;
            ev_last[i] = (dts > 0.0f) ? fabsf(mech_pos[i] - ev_pos[i]) / dts : 0.0f;
            ev_pos[i] = mech_pos[i];
            ev_us[i]  = ev_now;
        }
        cur_vel = ev_last[i];   // replaces the coarse FG tacho for the velocity PID, is_stuck, brake
    }

    {
        static float   safe_target[num_joints];
        static bool    safe_init [num_joints] = { false };
        static bool    was_stale [num_joints] = { false };
        static bool    resuming  [num_joints] = { false };
        const uint32_t CMD_STALE_MS = 250;
        uint32_t cmd_age_ms = (uint32_t)(time_us_64() / 1000) - g_last_msg_ms;
        bool     stale      = (cmd_age_ms > CMD_STALE_MS);

        if (!safe_init[i]) { safe_target[i] = normalize_angle_0_2pi(mech_pos[i]); safe_init[i] = true; }

        if (stale) {
            if (!was_stale[i]) safe_target[i] = normalize_angle_0_2pi(mech_pos[i]);  // latch hold point ONCE
            resuming[i] = true;                                 // will ramp back when commands return
        } else if (resuming[i]) {
            float e_st = atan2f(sinf(target - safe_target[i]), cosf(target - safe_target[i]));
            float step = VEL_MAX[i] * dt;                       // max target advance per tick
            if (fabsf(e_st) <= step) { safe_target[i] = target; resuming[i] = false; }
            else safe_target[i] = normalize_angle_0_2pi(safe_target[i] + (e_st > 0.0f ? step : -step));
        } else {
            safe_target[i] = target;                            // NORMAL: exact pass-through — no rate limit, no jerk
        }
        was_stale[i] = stale;
        target = safe_target[i];
    }

    // Calculate Error & Direction (from position error ONLY)
    float diff_raw = target - mech_pos[i];
    float err      = atan2(sin(diff_raw), cos(diff_raw));

#if IS_BOT_BOARD
    if (i == 1) {
        // Debug snapshot of everything that could silently reset under the loop.
        g_j1_mech  = mech_pos[i];
        g_j1_flags = (uint8_t)((stall_flag[i]    ? 0x1 : 0)
                             | (joint1_inited    ? 0x2 : 0)
                             | (g_zone_seeded    ? 0x4 : 0));
    }
#endif

    // Derive direction strictly from position error (with your guards)
    bool direction = dir_prev[i];
    if (fabs(fabs(err) - M_PI) > PI_GUARD && fabs(err) >= DIR_DZ)
        direction = (err <= 0.0);   // err < 0 => direction=1 per your original convention
    dir_prev[i] = direction;        // keep legacy memory if you use it elsewhere

    // PID Calculations
    float vel_set = position_pid(i, target, mech_pos[i], dt);
    if (vel_set >  VEL_MAX[i]) vel_set =  VEL_MAX[i];
    if (vel_set < -VEL_MAX[i]) vel_set = -VEL_MAX[i];
    float cur_vel_signed = (vel_set < 0.0f) ? -cur_vel : cur_vel;
    float pwm_out = velocity_pid(i, vel_set, cur_vel_signed, dt);   // signed, in your PWM units

    // Intent & stop window
    bool  wants_move     = (fabs(err) > MARGIN[i]);
    bool  at_target      = !wants_move;
bool in_stop_window = (fabs(err) < MARGIN[i]) && (fabs(cur_vel) < VMIN_MOVE);

bool gravity_hold = GRAVITY_HOLD[i];

if (!gravity_hold && in_stop_window) {
    integral_pos[i] = 0.0f; prev_error_pos[i] = 0.0f;
    integral_vel[i] = 0.0f; prev_error_vel[i] = 0.0f;
    last_cmd[i]     = 0.0f;
    min_pwm_move_runtime[i] = 0.0f;
    stuck_counts[i]  = 0;
    float hold = MIN_PWM_HOLD[i];
    if (hold > 1e-3f)
        motor_control(i, true, direction, hold);
    else
        motor_control(i, false, direction, 0.0f);
    DBG_MOTOR("AT_TARGET", mech_pos[i], err, cur_vel,
              direction ? -hold : hold);
    continue;
}

// ------------------ Stiction bookkeeping ------------------
bool is_stuck = fabs(cur_vel) < VSTUCK;
if (wants_move) {
    if (is_stuck) {
        const float cap = fmaxf(0.0f, PWM_MIN_CAP - MIN_PWM_MOVE[i]);
        min_pwm_move_runtime[i] = fminf(min_pwm_move_runtime[i] + STEP_INC, cap);
        stuck_counts[i]++;
    } else {
        if (min_pwm_move_runtime[i] > 0.0f)
            min_pwm_move_runtime[i] = fmaxf(0.0f, min_pwm_move_runtime[i] - STEP_DEC);
        stuck_counts[i] = 0;
    }
} else if (at_target && min_pwm_move_runtime[i] > 0.0f) {
    min_pwm_move_runtime[i] = fmaxf(0.0f, min_pwm_move_runtime[i] - STEP_DEC);
}

float pwm_min_rt = MIN_PWM_MOVE[i] + min_pwm_move_runtime[i];

bool dir_final = direction;
float abs_pwm;

float drive = fabsf(pwm_out);

bool descent_brake = false;   // set below: short the motor to damp a gravity overspeed (TOP j1)

if (gravity_hold) {
    abs_pwm = drive;

    if (fabsf(err) < g_brake_dist && cur_vel > fabsf(vel_set) + g_brake_vel_min) {
        descent_brake = true;
    }
} else {
    // Original zero-cross + magnitude slew for all other joints
    float u_prev = last_cmd[i];
    bool prev_sign       = (u_prev >= 0.0f);
    bool sign_diff_final = (prev_sign != dir_final);
    bool prev_meaningful = (fabs(u_prev) > MIN_PWM_MOVE[i]);
    float u_start        = (sign_diff_final && prev_meaningful) ? 0.0f : u_prev;

    const float SLEW_UP   = SLEW_STEP_UP[i];
    const float SLEW_DOWN = 50.0f;
    float mag_now  = fabs(u_start);
    float mag_goal = drive;
    if (mag_goal > mag_now) mag_now = fminf(mag_now + SLEW_UP,   mag_goal);
    else                    mag_now = fmaxf(mag_now - SLEW_DOWN, mag_goal);
    abs_pwm = mag_now;
    if (abs_pwm > MOTOR_PWM_MAX[i]) abs_pwm = MOTOR_PWM_MAX[i];
}

if (wants_move && fabsf(err) > ERR_MOVE[i] && is_stuck && stuck_counts[i] >= 20 &&
    abs_pwm > 0.0f && abs_pwm < pwm_min_rt) {
    abs_pwm = pwm_min_rt;
}

if (wants_move && abs_pwm < MIN_PWM_MOVE[i]) {
    integral_vel[i] = 0.0f;
}

// Hard per-motor PWM cap — enforced after stiction, before driver
if (abs_pwm > MOTOR_PWM_MAX[i]) abs_pwm = MOTOR_PWM_MAX[i];

last_cmd[i] = dir_final ? +abs_pwm : -abs_pwm;

bool enable = gravity_hold || wants_move;

if (fabsf(target - stall_last_target[i]) > 0.01f) {
    stall_flag[i]          = false;
    stall_ticks[i]         = 0;
    stall_recover_ticks[i] = 0;
    stall_last_target[i]   = target;
}

if (stall_flag[i]) {
    uint32_t rt = stall_recover_ticks[i]++;
    if (rt < STALL_RETRY_OFF_TICKS) {
        // Cooling down — motor off.
        motor_control(i, false, dir_final, 0.0f);
        DBG_MOTOR("STALLED", mech_pos[i], err, cur_vel, 0.0f);
        continue;
    }
    if (rt == STALL_RETRY_OFF_TICKS)
        stall_pos_ref[i] = mech_pos[i];            // snapshot position at the retry-burst start
    if (rt >= STALL_RETRY_OFF_TICKS + STALL_RETRY_BURST_TICKS) {
        if (fabsf(mech_pos[i] - stall_pos_ref[i]) >= STALL_MIN_PROGRESS) {
            stall_flag[i]          = false;        // moving again — resume normal control
            stall_recover_ticks[i] = 0;
            stall_ticks[i]         = 0;
        } else {
            stall_recover_ticks[i] = 0;            // still jammed — cut power and cool down again
            stall_ticks[i]         = 0;            // fresh detection window on the next burst
            motor_control(i, false, dir_final, 0.0f);
            DBG_MOTOR("STALLED", mech_pos[i], err, cur_vel, 0.0f);
            continue;
        }
    }
    // Within the retry burst → fall through and let the PID drive this tick.
}

{
    float stall_thresh = gravity_hold ? STALL_PWM_THRESH_GRV : STALL_PWM_THRESH_STD;
    bool  high_pwm     = (abs_pwm >= stall_thresh);
    bool  not_at_tgt   = (fabsf(err) > MARGIN[i]);

    if (enable && high_pwm && not_at_tgt) {
        if (stall_ticks[i] == 0)
            stall_pos_ref[i] = mech_pos[i];  // snapshot position at window start

        stall_ticks[i]++;

        if (stall_ticks[i] >= STALL_TIMEOUT_TICKS) {
            float progress = fabsf(mech_pos[i] - stall_pos_ref[i]);
            if (progress < STALL_MIN_PROGRESS && fabs(cur_vel) < VSTUCK) {
                stall_flag[i]     = true;
                stall_ticks[i]    = 0;
                integral_pos[i]   = 0.0f;
                prev_error_pos[i] = 0.0f;
                integral_vel[i]   = 0.0f;
                prev_error_vel[i] = 0.0f;
                last_cmd[i]       = 0.0f;
                printf("STALL joint %d — motor cut. target=%.3f pos=%.3f pwm=%.1f\r\n",
                       i, target, current_joints_positions[i], abs_pwm);
                motor_control(i, false, dir_final, 0.0f);
                continue;
            } else {
                // Still making progress — reset window and keep going
                stall_ticks[i]   = 0;
                stall_pos_ref[i] = mech_pos[i];
            }
        }
    } else {
        stall_ticks[i] = 0;
    }
}

#if IS_BOT_BOARD
    static const int BK_TRIGGER = 15;   // ticks (~ms) stuck before the breakaway fires
    static const int BK_FWD     = 4;    // ticks driven in the commanded direction
    static const int BK_REV     = 1;    // ticks reversed (static-friction crack), per cycle
    static const int BK_BUDGET  = 200;  // max ticks (~ms) of breakaway per stuck episode
    static int bk_phase[num_joints]  = {0};
    static int bk_budget[num_joints] = {0};
    if (i == 1) {
        if (stuck_counts[i] == 0) { bk_phase[i] = 0; bk_budget[i] = BK_BUDGET; }  // refill while moving / at target
        if (enable && !stall_flag[i] && fabsf(err) > MARGIN[i]
            && stuck_counts[i] >= BK_TRIGGER && bk_budget[i] > 0) {
            bk_budget[i]--;
            bk_phase[i] = (bk_phase[i] + 1) % (BK_FWD + BK_REV);
            bool bk_rev = (bk_phase[i] >= BK_FWD);            // last BK_REV ticks of each cycle reverse
            bool bk_dir = bk_rev ? !dir_final : dir_final;
            motor_control(i, true, bk_dir, 100.0f);
            DBG_MOTOR("BREAKAWAY", mech_pos[i], err, cur_vel, bk_dir ? -100.0f : 100.0f);
            continue;
        }
    }
#endif
    {
        static bool sat_prev[num_joints] = { false };
        float applied = descent_brake ? 0.0f : abs_pwm;
        g_pwm_mag[i] = applied;
        bool sat_now = (applied >= MOTOR_PWM_MAX[i] - 0.01f);
        if (sat_now && !sat_prev[i]) g_sat_count[i]++;
        sat_prev[i] = sat_now;
    }

    if (descent_brake) {
        motor_control(i, false, dir_final, 0.0f);
        DBG_MOTOR("BRAKE", mech_pos[i], err, cur_vel, 0.0f);
    } else {
        motor_control(i, enable, dir_final, abs_pwm);
        DBG_MOTOR(enable ? "DRIVE" : "IDLE", mech_pos[i], err, cur_vel,
                  dir_final ? -abs_pwm : abs_pwm);
    }
}
#undef DBG_MOTOR

        if (g_pwm_apply_pending) {
            g_pwm_apply_pending = false;
            sio_hw->gpio_set = (1u << DEBUG_PWM_APPLY_PIN);
            g_pwm_high    = true;
            g_pwm_high_us = (uint32_t)time_us_64();
            if (g_lat_await_c1) {
                g_lat_tc1_ns    = rmw_uros_epoch_nanos();
                g_lat_await_c1  = false;
                g_lat_c1_report = true;
            }
        }

#if IS_BOT_BOARD
        {
            float servo_angle = (targets_local[num_joints+2] >= 0.5f)
                                ? SERVO_POS_ON_RAD : SERVO_POS_OFF_RAD;
            servo_write_rad(SERVO_PIN, servo_angle);
        }
#endif

        if (did_break) {
            sio_hw->gpio_clr = (1u << MOTOR_DEBUG_PIN);
            continue;
        }

        if(feedback_positions != NULL) {
            taskENTER_CRITICAL();
            {
                for (size_t i = 0; i < num_joints; i++) {
                    feedback_positions[i] = current_joints_positions[i];
                }
#if IS_BOT_BOARD
                feedback_positions[2] = current_joints_velocities[2] * WHEEL_RADIUS_M;
#endif

                int64_t current_agent_us = time_us_64() + g_clock_offset_us;
                int32_t seconds_full = (int32_t)(current_agent_us / 1000000);
                double fraction = (double)(current_agent_us % 1000000) / 1000000.0;

                // Safely index timestamp parts right after the joints
                feedback_positions[num_joints]   = (float)(seconds_full % 100000);
                feedback_positions[num_joints+1] = (float)fraction;
#if IS_BOT_BOARD
                // Mirror last commanded servo state so the host can confirm actuation
                feedback_positions[num_joints+2] = targets_local[num_joints+2];
#endif
            }
            taskEXIT_CRITICAL();
        }

        #if MOTOR_TASK_TIME_DEBUG
            uint64_t us_end = time_us_64();
            exec_time_us = (uint32_t)(us_end - us_start);
            curr_tick = xTaskGetTickCount();
            if (curr_tick > prev_tick) {
                uint32_t delta_ticks = (uint32_t)(curr_tick - prev_tick);
                float cycle_time_s = (float)delta_ticks * ((float)portTICK_PERIOD_MS / 1000.0f);
                if (cycle_time_s > 0.0f) measured_freq = 1.0f / cycle_time_s;
                prev_tick = curr_tick;
            }
        #endif

        sio_hw->gpio_clr = (1u << MOTOR_DEBUG_PIN);
    }
}

static volatile uint32_t uros_exec_us = 0;
static volatile float    uros_freq_hz = 0.0f;

uint16_t count_feedback = 0;

#define MOTOR_STACK_SIZE (24 * 1024)
#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3)

void vUROSTask(void *pvParameters) {
    printf("[UROS] Task Started (Pure Hardware Mode).\n");

    // Debug LED for monitoring network activity
    gpio_init(COMMS_DEBUG_PIN);
    gpio_set_dir(COMMS_DEBUG_PIN, GPIO_OUT);

    gpio_init(LATENCY_DEBUG_PIN);
    gpio_set_dir(LATENCY_DEBUG_PIN, GPIO_OUT);
    gpio_put(LATENCY_DEBUG_PIN, 0);
    gpio_init(DEBUG_PWM_APPLY_PIN);
    gpio_set_dir(DEBUG_PWM_APPLY_PIN, GPIO_OUT);
    gpio_put(DEBUG_PWM_APPLY_PIN, 0);

#ifndef USE_WIFI

    // Initialize W5500 SPI Interface & Register Callbacks
    wizchip_spi_initialize();
    wizchip_initialize();

    // Hardware Reset Sequence
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 0);
    sleep_ms(1);
    gpio_put(PIN_RST, 1);
    sleep_ms(150);

    // Initialize Internal Buffers (2KB TX / 2KB RX per socket)
    uint8_t bufSize[] = {2, 2, 2, 2, 2, 2, 2, 2};
    if (wizchip_init(bufSize, bufSize) == -1) {
        printf("[FATAL] W5500 SPI Failure! Check wiring.\n");
        while(1) { vTaskDelay(1000); }
    }

    uint8_t leg_num = BOARD_ID / 10;
    uint8_t pos_num = BOARD_ID % 10;
#if BOARD_ID == 41
    wiz_NetInfo netInfo = {
        .mac  = {0x00, 0x08, 0xDC, 0x01, leg_num, pos_num},
        .ip   = {192, 168, 5, (uint8_t)BOARD_ID},
        .sn   = {255, 255, 255, 0},
        .gw   = {192, 168, 5, 1},
        .dns  = {8, 8, 8, 8},
        .dhcp = NETINFO_STATIC
    };
    printf("[UROS] W5500 ready.  IP: 192.168.5.%d\n", BOARD_ID);
#else
    wiz_NetInfo netInfo = {
        .mac  = {0x00, 0x08, 0xDC, 0x01, leg_num, pos_num},
        .ip   = {192, 168, 1, (uint8_t)BOARD_ID},
        .sn   = {255, 255, 255, 0},
        .gw   = {192, 168, 1, 1},
        .dns  = {8, 8, 8, 8},
        .dhcp = NETINFO_STATIC
    };
    printf("[UROS] W5500 ready.  IP: 192.168.1.%d\n", BOARD_ID);
#endif
    ctlnetwork(CN_SET_NETINFO, (void*)&netInfo);

    // Verify SPI (W5500 version register must read 0x04)
    uint8_t version = getVERSIONR();
    if (version == 0x04) {
        printf("[W5500] Chip OK (ver=0x%02X).\n", version);
    } else {
        printf("[W5500] SPI FAILURE — expected 0x04, got 0x%02X.\n", version);
        while(1) vTaskDelay(100);
    }

    uint8_t phycfgr_hw = getPHYCFGR();
    printf("[W5500] HW strap PHYCFGR=0x%02X — forcing auto-MDIX (0xF8)\n", phycfgr_hw);
    setPHYCFGR(0x78);   /* RST=0, OPMD=1, OPMDC=111 — reset with new config    */
    sleep_ms(20);
    setPHYCFGR(0xF8);   /* RST=1, OPMD=1, OPMDC=111 — all-capable + auto-MDIX */
    sleep_ms(200);
    printf("[W5500] PHYCFGR=0x%02X after PHY init\n", getPHYCFGR());

    // Open UDP Socket 0 for Micro-ROS (port 4444)
    if (socket(0, Sn_MR_UDP, 4444, 0) != 0) {
        printf("[FATAL] Failed to open W5500 UDP socket!\n");
        while(1);
    }

#else  /* USE_WIFI */
    while (!connection_wifi_init()) {
        printf("[UROS] WiFi init failed — retrying in 5 s...\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

#endif /* USE_WIFI */

    printf("[UROS] Network UP! Launching Motor Task on Core 1...\n");

    TaskHandle_t motor_task_handle;
    xTaskCreate(
        vMotorControlTask,      // Function
        "MotorControl",         // Name
        MOTOR_STACK_SIZE,       // Stack
        NULL,                   // Params
        MOTOR_CONTROL_PRIORITY, // Priority
        &motor_task_handle      // Handle
    );
    // CRITICAL: Pin to Core 1
    vTaskCoreAffinitySet(motor_task_handle, (1 << 1));

    const uint64_t PERIOD_US = 1000000 / FEEDBACK_PUBLISH_HZ;
    uint64_t next_publish_time = time_us_64();

    // Initialize Watchdog Timer
    g_last_msg_ms = (uint32_t)(time_us_64() / 1000);

    while (true) {
        // --- A. CONNECTION MANAGEMENT ---
        printf("Connecting to Micro-ROS Agent...\n");
        bool connected = init_uROS(); // Ping Agent & Create Entities
        printf("Connection Result: %s\n", connected ? "SUCCESS" : "FAILED");

        if (connected) {
            // Reset timers upon successful connection
            next_publish_time = time_us_64();
            g_last_msg_ms = (uint32_t)(time_us_64() / 1000);
            uint32_t last_diag_ms = (uint32_t)(time_us_64() / 1000);
            uint32_t last_mdbg_ms = (uint32_t)(time_us_64() / 1000);
            uint32_t last_sync_ms = (uint32_t)(time_us_64() / 1000);

            // --- B. ACTIVE SESSION LOOP ---
            while (true) {
                if (g_lat_high && ((uint32_t)time_us_64() - g_lat_high_us) >= (uint32_t)LATENCY_PULSE_US) {
                    sio_hw->gpio_clr = (1u << LATENCY_DEBUG_PIN);
                    g_lat_high = false;
                }
#if LATENCY_CB_DEBUG
                if (g_lat_c0_report) {
                    g_lat_c0_report = false;
                    const int64_t WRAP_NS = 100000LL * 1000000000LL;   // t_sec wraps every 100000 s
                    const int64_t LEAD_NS = 50LL * 1000000LL;          // offset_bridge lead = 50 ms
                    int64_t msg_ns   = (int64_t)(g_lat_tsend_sec) * 1000000000LL
                                     + (int64_t)(g_lat_tsend_frac * 1e9 + 0.5);
                    int64_t board_ns = g_lat_tc0_ns % WRAP_NS;
                    int64_t tport    = board_ns - msg_ns;
                    if (tport < -(WRAP_NS / 2)) tport += WRAP_NS;      // wrap guard
                    tport += LEAD_NS;
                    g_lat_transport_us = (float)((double)tport / 1000.0);   // signed µs, for feedback
                }
                if (g_lat_c1_report) {
                    g_lat_c1_report = false;
                    g_lat_sched_us = (float)((double)(g_lat_tc1_ns - g_lat_tc0_ns) / 1000.0);
                }
#endif
                sio_hw->gpio_set = (1u << COMMS_DEBUG_PIN);

                // Check for new subscriptions/services with 0 timeout
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

                uint64_t now = time_us_64();

                if (now >= next_publish_time) {
                    hw_feedback(); // Send Joint States

                    // Schedule next publication
                    next_publish_time += PERIOD_US;

                    if (now > next_publish_time + PERIOD_US) {
                        next_publish_time = now + PERIOD_US;
                    }
                }

                // Set Debug Pin LOW (End of active cycle)
                sio_hw->gpio_clr = (1u << COMMS_DEBUG_PIN);

                uint32_t now_ms = (uint32_t)(now / 1000);

#if LATENCY_CB_DEBUG && (LATENCY_RESYNC_MS > 0)
                if ((now_ms - last_sync_ms) >= (uint32_t)LATENCY_RESYNC_MS) {
                    last_sync_ms = now_ms;
                    rcl_ret_t sync_rc = rmw_uros_sync_session(LATENCY_RESYNC_TIMEOUT_MS);
                    (void)sync_rc;   // best-effort: keep the last good offset if this attempt fails
                }
#endif

                if ((now_ms - last_diag_ms) >= 1000) {
                    last_diag_ms = now_ms;
                    diag_data[0] = (float)xPortGetFreeHeapSize();              // free heap, BYTES
                    diag_data[1] = (float)(now / 1000000);                    // uptime in seconds
                    // diag_data[2] = BOARD_ID (set once at init, unchanged)
                    diag_data[3] = (float)xPortGetMinimumEverFreeHeapSize();  // worst-case watermark, BYTES
                    for (int mi = 0; mi < (int)num_joints; ++mi) {
                        diag_data[4 + mi]              = (float)g_sat_count[mi];
                        diag_data[4 + num_joints + mi] = g_pwm_mag[mi];
                    }
                    rcl_ret_t diag_rc = rcl_publish(&diag_publisher, &diag_msg, NULL);
                    (void)diag_rc;
                }

#if DEBUG_MOTOR
                if (g_debug_motor && (now_ms - last_mdbg_ms) >= DEBUG_MOTOR_MS) {
                    last_mdbg_ms = now_ms;
                    char dbg[320];   // the j1 line carries the full state snapshot
                    for (size_t i = 0; i < num_joints; i++) {
                        MotorDbg d = g_mdbg[i];   // copy — core 1 keeps writing
                        unsigned raw = (i < 2) ? g_enc_raw[i]    : 0;
                        unsigned rej = (i < 2) ? g_enc_reject[i] : 0;

#if IS_BOT_BOARD
                        if (i == 1) {
                            float carrier = fmodf(d.pos, 2.0f * (float)M_PI);
                            if (carrier < 0.0f) carrier += 2.0f * (float)M_PI;

                            float inz  = g_j1_sun / 3.0f;
                            float zmax = (float)(joint1_rev_idx + 1) * (2.0f * (float)M_PI / 3.0f);

                            const float ZONEW = 2.0f * (float)M_PI / 3.0f;   // 2.094 rad per zone
                            int   tZ   = (int)(d.target / ZONEW);
                            if (tZ < 0) tZ = 0; else if (tZ > 2) tZ = 2;
                            float tinz = d.target - (float)tZ * ZONEW;

                            uint8_t f = g_j1_flags;
                            float duty_pct = 100.0f * (float)g_last_level[1] / (float)PWM_WRAP_VALUE;
                            snprintf(dbg, sizeof(dbg),
                                "%s j1 %-9s tgt:%.3f tZ:%d tinz:%.3f | glob:%.3f inz:%.3f z:%d zmax:%.3f err:%+.3f | sun:%.3f dsun:%+.4f cl:%.3f zo:%u off:%.3f | dir:%u en:%u pwm:%.1f duty:%u(%.0f%%) | vel:%.2f iv:%+.1f | raw:%u stall:%lu fg[j0/j1/wh]:%lu/%lu/%lu gp10:%d lo:%d wheel:%+.0f f:%c%c%c",
                                node_name, d.state ? d.state : "INIT",
                                (double)d.target, tZ, (double)tinz,
                                (double)carrier, (double)inz, (int)joint1_rev_idx, (double)zmax,
                                (double)d.err,
                                (double)g_j1_sun, (double)g_j1_dsun, (double)g_j1_cl,
                                (unsigned)g_j1_zo, (double)offset[1],
                                (unsigned)g_last_dir[1], (unsigned)g_last_en[1],
                                (double)g_last_pwm[1], (unsigned)g_last_level[1], (double)duty_pct,
                                (double)d.vel, (double)integral_vel[1],
                                raw, (unsigned long)d.stall,
                                (unsigned long)get_total_pulses(0), (unsigned long)get_total_pulses(1), (unsigned long)get_total_pulses(2),
                                g_j1_fg_lvl, g_j1_fg_low_seen ? 1 : 0, (double)g_wheel_duty,
                                (f & 0x1) ? 'S' : '-', (f & 0x2) ? 'I' : '-', (f & 0x4) ? 'Z' : '-');
                            g_j1_fg_low_seen = false;   // reset latch: 'lo' reports low-seen since the previous print
                            publish_debug(dbg);
                            continue;
                        }
#endif
                        snprintf(dbg, sizeof(dbg),
                            "%s j%u %-9s tgt:%.3f pos:%.3f err:%.3f vel:%.2f pwm:%.1f stall:%lu raw:%u rej:%u",
                            node_name, (unsigned)i, d.state ? d.state : "INIT",
                            (double)d.target, (double)d.pos, (double)d.err,
                            (double)d.vel, (double)d.pwm, (unsigned long)d.stall, raw, rej);
                        publish_debug(dbg);
                    }
                }
#endif

                if ((now_ms - g_last_msg_ms) > 500) {
#if IS_BOT_BOARD
                    taskENTER_CRITICAL();
                    target_joints_positions[2] = 0.0f;
                    taskEXIT_CRITICAL();
#endif
                }

                static uint32_t last_ping_ms     = 0;
                static uint8_t  ping_fail_streak = 0;
                if ((now_ms - last_ping_ms) > 2000) {
                    last_ping_ms = now_ms;
                    if ((now_ms - g_last_msg_ms) < 1500) {
                        ping_fail_streak = 0;          // commands flowing -> agent alive
                    } else if (rmw_uros_ping_agent(500, 3) != RMW_RET_OK) {
                        if (++ping_fail_streak >= 2) { // ~sustained silence, not one bad window
                            printf("[COMMS] Agent lost (%u) — reconnecting...\n",
                                   (unsigned)ping_fail_streak);
                            break;
                        }
                    } else {
                        ping_fail_streak = 0;          // ping answered -> alive
                    }
                }

                // NO SLEEP: Run as fast as possible to minimize latency jitter.
            }
        }

        reset_micro_ros_state();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void hw_feedback() {

    rcl_ret_t rc;
    static float last_published_positions[num_joints] = {0.0f};

    const float POSITION_TOLERANCE = 1e-6f;

    taskENTER_CRITICAL();
    {
        memcpy(feedback_positions_temp, feedback_positions, CMD_MSG_SIZE * sizeof(float));
    }
    taskEXIT_CRITICAL();

    bool data_changed = false;

    for (size_t i = 0; i < num_joints; i++) {
        // If the difference exceeds our tolerance, mark as changed and exit loop
        if (fabs(feedback_positions_temp[i] - last_published_positions[i]) > POSITION_TOLERANCE) {
            data_changed = true;
            break;
        }
    }

    // A. Update the history buffer for the next cycle's comparison
    for (size_t i = 0; i < num_joints; i++) {
        last_published_positions[i] = feedback_positions_temp[i];
    }

    // B. Copy the temp buffer into the actual ROS message structure
    memcpy(feedback_msg.data.data, feedback_positions_temp, CMD_MSG_SIZE * sizeof(float));

#if IS_BOT_BOARD
    feedback_msg.data.data[CMD_MSG_SIZE]     = (float)joint1_rev_idx;  // zone 0/1/2
    feedback_msg.data.data[CMD_MSG_SIZE + 1] = (float)(g_fb_seq++);
#if LATENCY_CB_DEBUG
    feedback_msg.data.data[CMD_MSG_SIZE + 2] = g_lat_transport_us;   // transport µs
    feedback_msg.data.data[CMD_MSG_SIZE + 3] = g_lat_sched_us;       // sched µs
    feedback_msg.data.size = CMD_MSG_SIZE + 4;
#else
    feedback_msg.data.size = CMD_MSG_SIZE + 2;   // 8 floats
#endif
#else
    feedback_msg.data.data[CMD_MSG_SIZE] = (float)(g_fb_seq++);
#if LATENCY_CB_DEBUG
    feedback_msg.data.data[CMD_MSG_SIZE + 1] = g_lat_transport_us;   // transport µs
    feedback_msg.data.data[CMD_MSG_SIZE + 2] = g_lat_sched_us;       // sched µs
    feedback_msg.data.size = CMD_MSG_SIZE + 3;
#else
    feedback_msg.data.size = CMD_MSG_SIZE + 1;   // 5 floats
#endif
#endif

    // E. Publish to the ROS Agent
    rc = rcl_publish(&joint_state_publisher, &feedback_msg, NULL);
}