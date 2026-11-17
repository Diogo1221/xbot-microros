#include "memory.h"
#include "uROS.h"
#include "board_config.h"
#include <string.h>
#include <micro_ros_utilities/string_utilities.h>

void safe_free(void **ptr) {
    if (ptr != NULL && *ptr != NULL) {
        vPortFree(*ptr);
        *ptr = NULL;
    }
}

void allocate_feedback_publisher_memory() {
  // 1. Global Arrays (Keep strict persistence)
  if (feedback_positions == NULL) {
      feedback_positions = (float *) pvPortMalloc(CMD_MSG_SIZE * sizeof(float));
      memset(feedback_positions, 0, CMD_MSG_SIZE * sizeof(float));
  }
  if (feedback_positions_temp == NULL) {
      feedback_positions_temp = (float *) pvPortMalloc(CMD_MSG_SIZE * sizeof(float));
      memset(feedback_positions_temp, 0, CMD_MSG_SIZE * sizeof(float));
  }

  // 2. Init Message
  std_msgs__msg__Float32MultiArray__init(&feedback_msg);

#if IS_BOT_BOARD
  const size_t fb_capacity = CMD_MSG_SIZE + 2 + num_joints;
#else
  const size_t fb_capacity = CMD_MSG_SIZE + 1 + num_joints;
#endif
  feedback_msg.data.capacity = fb_capacity;
  feedback_msg.data.size = fb_capacity;
  feedback_msg.data.data = (float *) pvPortMalloc(feedback_msg.data.capacity * sizeof(float));
  if (feedback_msg.data.data) memset(feedback_msg.data.data, 0, feedback_msg.data.capacity * sizeof(float));

  feedback_msg.layout.dim.size = 0;
  feedback_msg.layout.dim.capacity = 0;
  feedback_msg.layout.dim.data = NULL; // No pointer, no string, no malloc
  feedback_msg.layout.data_offset = 0;
}

void allocate_pid_feedback_publisher_memory() {
  // 1. Global Array
  if (pid_feedback_values == NULL) {
      pid_feedback_values = (float *) pvPortMalloc(7 * sizeof(float));
      memset(pid_feedback_values, 0, 7 * sizeof(float));
  }

  // 2. Init Message
  std_msgs__msg__Float32MultiArray__init(&pid_message);

  // 3. Internal Buffer
  pid_message.data.capacity = 7;
  pid_message.data.size = 7;
  pid_message.data.data = (float *) pvPortMalloc(7 * sizeof(float));
  if(pid_message.data.data) memset(pid_message.data.data, 0, 7 * sizeof(float));

  // 4. OPTIMIZATION: REMOVE LAYOUT METADATA
  pid_message.layout.dim.size = 0;
  pid_message.layout.dim.capacity = 0;
  pid_message.layout.dim.data = NULL;
  pid_message.layout.data_offset = 0;
}
void allocate_subscriber_memory() {
#if IS_TOP_BOARD
  const size_t recv_capacity = CMD_MSG_SIZE + 2;
#else
  const size_t recv_capacity = CMD_MSG_SIZE + 2;
#endif
  if (target_positions == NULL) {
      target_positions = (float *)pvPortMalloc(recv_capacity * sizeof(float));
      memset(target_positions, 0, recv_capacity * sizeof(float));
  }
  std_msgs__msg__Float32MultiArray__init(&leg_controller_state);
  leg_controller_state.data.data     = target_positions;
  leg_controller_state.data.capacity = recv_capacity;
  leg_controller_state.data.size     = 0;

  // [OPTIMIZATION APPLIED HERE] Remove Layout Setup
  leg_controller_state.layout.dim.size = 0;
  leg_controller_state.layout.dim.capacity = 0;
  leg_controller_state.layout.dim.data = NULL; // Free up the heap memory!
  leg_controller_state.layout.data_offset = 0;
}
void allocate_offset_subscriber_memory() {
  const size_t off_capacity = num_joints + 2;
  if (offset_positions == NULL) {
      offset_positions = (float *)pvPortMalloc(off_capacity * sizeof(float));
      memset(offset_positions, 0, off_capacity * sizeof(float));
  }
  std_msgs__msg__Float32MultiArray__init(&offset_message);
  offset_message.data.data = offset_positions;
  offset_message.data.capacity = off_capacity;
  offset_message.data.size = 0;

  offset_message.layout.dim.capacity = 1;
  offset_message.layout.dim.size = 1;
  offset_message.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)
      pvPortMalloc(sizeof(std_msgs__msg__MultiArrayDimension));
  rosidl_runtime_c__String__init(&offset_message.layout.dim.data[0].label);
  rosidl_runtime_c__String__assign(&offset_message.layout.dim.data[0].label, "offset");
}

void allocate_pid_subscriber_memory() {
  if (pid_values == NULL) {
      pid_values = (float*) pvPortMalloc(7 * sizeof(float));
      memset(pid_values, 0, 7 * sizeof(float));
  }
  std_msgs__msg__Float32MultiArray__init(&pid_message_sub);
  pid_message_sub.data.data = pid_values;
  pid_message_sub.data.capacity = 7;
  pid_message_sub.data.size = 0;

  pid_message_sub.layout.dim.size = 0;
  pid_message_sub.layout.dim.capacity = 0;
  pid_message_sub.layout.dim.data = NULL;
}

void allocate_config_subscriber_memory() {
    const size_t cfg_capacity = 12;
    if (config_values == NULL) {
        config_values = (float*) pvPortMalloc(cfg_capacity * sizeof(float));
        memset(config_values, 0, cfg_capacity * sizeof(float));
    }
    std_msgs__msg__Float32MultiArray__init(&config_message_sub);
    config_message_sub.data.data     = config_values;
    config_message_sub.data.capacity = cfg_capacity;
    config_message_sub.data.size     = 0;
    config_message_sub.layout.dim.size     = 0;
    config_message_sub.layout.dim.capacity = 0;
    config_message_sub.layout.dim.data     = NULL;
}

void free_feedback_publisher_memory() {
  // 1. DO NOT FREE 'feedback_positions'! (Motor Task needs it)

  // 2. Free the separate message buffer we allocated
  safe_free((void**)&feedback_msg.data.data);
  safe_free((void**)&pid_message.data.data);

  // 3. Free Layouts
  if (feedback_msg.layout.dim.data) {
      rosidl_runtime_c__String__fini(&feedback_msg.layout.dim.data[0].label);
      safe_free((void**)&feedback_msg.layout.dim.data);
  }
  if (pid_message.layout.dim.data) {
      rosidl_runtime_c__String__fini(&pid_message.layout.dim.data[0].label);
      safe_free((void**)&pid_message.layout.dim.data);
  }

  // 4. Fini structs
  std_msgs__msg__Float32MultiArray__fini(&feedback_msg);
  std_msgs__msg__Float32MultiArray__fini(&pid_message);
}

void free_subscriber_memory() {
  leg_controller_state.data.data = NULL;
  leg_controller_state.data.size = 0;
  leg_controller_state.data.capacity = 0;

  offset_message.data.data = NULL;
  offset_message.data.size = 0;
  offset_message.data.capacity = 0;

  pid_message_sub.data.data = NULL;
  pid_message_sub.data.size = 0;
  pid_message_sub.data.capacity = 0;

  // 2. Clean up layouts
  if (leg_controller_state.layout.dim.data) {
       rosidl_runtime_c__String__fini(&leg_controller_state.layout.dim.data[0].label);
       safe_free((void**)&leg_controller_state.layout.dim.data);
  }
  if (offset_message.layout.dim.data) {
       rosidl_runtime_c__String__fini(&offset_message.layout.dim.data[0].label);
       safe_free((void**)&offset_message.layout.dim.data);
  }

  // 3. Fini Messages
  std_msgs__msg__Float32MultiArray__fini(&leg_controller_state);
  std_msgs__msg__Float32MultiArray__fini(&offset_message);
  std_msgs__msg__Float32MultiArray__fini(&pid_message_sub);

  // 4. DO NOT FREE 'target_positions', 'offset_positions', etc.
}