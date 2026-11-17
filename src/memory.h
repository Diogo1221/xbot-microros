#ifndef MEMORY_H
#define MEMORY_H

#include <stddef.h> // for size_t
typedef struct std_msgs__msg__Float32MultiArray std_msgs__msg__Float32MultiArray;

extern float *feedback_positions;
extern float *feedback_positions_temp;
extern float *pid_feedback_values;
extern float *target_positions;
extern float *offset_positions;
extern float *pid_values;

extern std_msgs__msg__Float32MultiArray feedback_msg;
extern std_msgs__msg__Float32MultiArray pid_message;
extern std_msgs__msg__Float32MultiArray leg_controller_state;
extern std_msgs__msg__Float32MultiArray offset_message;

extern std_msgs__msg__Float32MultiArray pid_message_sub;
extern std_msgs__msg__Float32MultiArray config_message_sub;
extern float *config_values;

void allocate_feedback_publisher_memory();
void allocate_pid_feedback_publisher_memory();
void allocate_subscriber_memory();
void allocate_offset_subscriber_memory();
void allocate_pid_subscriber_memory();
void allocate_config_subscriber_memory();

// Dummy free functions (kept for API compatibility)
void free_feedback_publisher_memory();
void free_subscriber_memory();

#endif

// void allocate_feedback_publisher_memory();

// void free_feedback_publisher_memory();

// void allocate_pid_feedback_publisher_memory();

// void allocate_subscriber_memory() ;

// void allocate_offset_subscriber_memory() ;

// void allocate_pid_subscriber_memory() ;

// void free_subscriber_memory() ;

// #endif // MEMORY_H_H

