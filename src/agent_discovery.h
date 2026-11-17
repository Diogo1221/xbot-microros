#ifndef AGENT_DISCOVERY_H
#define AGENT_DISCOVERY_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Multicast discovery of the micro-ROS Agent (XRCE GET_INFO -> INFO). */
bool agent_discover_multicast(uint8_t sock, uint16_t local_port,
                              uint32_t attempts, uint32_t period_ms,
                              uint8_t ip[4], uint16_t *port);

size_t agent_discovery_build_get_info(uint8_t *buf, size_t cap);
bool agent_discovery_parse_info(const uint8_t *buf, size_t len,
                                uint8_t ip[4], uint16_t *port);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_DISCOVERY_H */
