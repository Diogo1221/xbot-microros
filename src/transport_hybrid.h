
#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Micro-ROS Client Transport Profile
#include <uxr/client/profile/transport/custom/custom_transport.h>

extern bool g_is_eth_transport;

void set_microros_agent_ip(const char* ip, int port);
void set_discovered_agent(const uint8_t ip[4], uint16_t port);

bool w5500_transport_open(struct uxrCustomTransport * transport);
bool w5500_transport_close(struct uxrCustomTransport * transport);

size_t w5500_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t w5500_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

bool serial_transport_open(struct uxrCustomTransport * transport);
bool serial_transport_close(struct uxrCustomTransport * transport);

size_t serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

bool hybrid_transport_open(struct uxrCustomTransport * transport);
bool hybrid_transport_close(struct uxrCustomTransport * transport);

size_t hybrid_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t hybrid_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // MICRO_ROS_PICOSDK