#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// #define BOARD_ID 11  // L1 TOP
// #define BOARD_ID 12  // L1 BOT

// #define BOARD_ID 21  // L2 TOP
// #define BOARD_ID 22  // L2 BOT

// #define BOARD_ID 31  // L3 TOP
// #define BOARD_ID 32  // L3 BOT

// #define BOARD_ID 41  // L4 TOP
#define BOARD_ID 42  // L4 BOT

#define BOARD_POS (BOARD_ID % 10)

#if BOARD_POS == 1
    #define IS_TOP_BOARD 1
    #define IS_BOT_BOARD 0
    #define num_joints 2
    #define CMD_MSG_SIZE (num_joints + 2)   // [j0, j1, t_sec, t_frac]
#elif BOARD_POS == 2
    #define IS_TOP_BOARD 0
    #define IS_BOT_BOARD 1
    #define num_joints 3
    #define CMD_MSG_SIZE (num_joints + 3)   // [j0, j1, j2, t_sec, t_frac, servo_en]
#else
    #error "Invalid BOARD_ID position. Must end in 1 (Top) or 2 (Bot)."
#endif

#define WIFI_SSID_DEFAULT        "your_network_ssid"
#define WIFI_PASSWORD_DEFAULT    "your_network_password"

#if (BOARD_ID == 41)
  #define WIFI_AGENT_IP_DEFAULT  {192, 168, 5, 1}
#else
  #define WIFI_AGENT_IP_DEFAULT  {192, 168, 1, 100}
#endif
#define WIFI_AGENT_PORT_DEFAULT  8888

#ifndef XBOT_MULTICAST_DISCOVERY
#define XBOT_MULTICAST_DISCOVERY 1
#endif

#endif // BOARD_CONFIG_H