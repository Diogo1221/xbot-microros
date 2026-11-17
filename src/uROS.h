
#ifndef UROS_H
#define UROS_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/subscription.h>
#include <rclc/executor.h>
#include "rcl/logging.h"
#include "rcutils/logging_macros.h"

#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

// Geometry & TF Messages
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>

// Control & Trajectory Messages
#include <control_msgs/action/follow_joint_trajectory.h>
#include <control_msgs/msg/joint_trajectory_controller_state.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <sensor_msgs/msg/joint_state.h>

// Service Messages
#include <std_srvs/srv/trigger.h>

#include "pico/mutex.h"
#include "hardware/sync.h" // Required for taskENTER_CRITICAL()

#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>

#define RAD_TO_DEG 57.2958

#define MAX_GOALS 1
#define QUEUE_SIZE 5
#define TIMEOUT_MS 100
#define ATTEMPS 100

extern rcl_node_t       node;
extern rcl_allocator_t  allocator;
extern rclc_support_t   support;
extern rclc_executor_t  executor;
extern rcl_timer_t      timer;

extern rmw_qos_profile_t qos_profile;

extern rcl_publisher_t debug_uROS;

extern float *feedback_positions;       // Global feedback array
extern float *feedback_positions_temp;  // Thread-safe snapshot buffer

extern float *target_positions;         // Target positions array
extern float *offset_positions;         // Offset array
extern float *pid_values;               // PID Tuning values
extern float *pid_feedback_values;

extern std_msgs__msg__Float32 msg;
extern std_msgs__msg__String  debug_msg;

extern std_msgs__msg__Float32MultiArray feedback_msg;
extern std_msgs__msg__Float32MultiArray leg_controller_state;
extern std_msgs__msg__Float32MultiArray offset_message;
extern std_msgs__msg__Float32MultiArray pid_message;

// --- Initialization ---
bool init_uROS(void);

// --- Tasks ---
void vUROSTask(void *pvParameters);
void vMotorControlTask(void *pvParameters);

// --- Callbacks (ISR / Subscription Handlers) ---
void leg_controller_state_callback(const void *callback_msg);
void offset_callback(const void *callback_msg);
void pid_callback(const void *callback_msg);
void config_callback(const void *callback_msg);
void joint_states_callback(const void *msg);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

// --- Helpers ---
void hw_feedback(void);
void publish_debug(const char *message);
float normalize_angle_0_2pi(float angle);

#endif // UROS_H