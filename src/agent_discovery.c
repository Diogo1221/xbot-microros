/* Adapted from Micro-XRCE-DDS-Client discovery.c (Apache-2.0, eProsima). */
#include "agent_discovery.h"

#include <string.h>

#include <ucdr/microcdr.h>
#include <uxr/client/core/type/xrce_types.h>

#ifndef AGENT_DISCOVERY_NO_IO
#include "pico/stdlib.h"
#include "wizchip_conf.h"
#include "socket.h"
#endif

#define SESSION_ID_WITHOUT_CLIENT_KEY 0x80
#define SUBMESSAGE_ID_GET_INFO        2
#define SUBMESSAGE_ID_INFO            6
#define GET_INFO_MSG_SIZE             8
#define GET_INFO_REQUEST_ID           9
#define DISCOVERY_PORT                7400

void uxr_serialize_message_header(ucdrBuffer *ub, uint8_t session_id, uint8_t stream_id,
                                  uint16_t seq_num, const uint8_t *key);
void uxr_deserialize_message_header(ucdrBuffer *ub, uint8_t *session_id, uint8_t *stream_id,
                                    uint16_t *seq_num, uint8_t *key);
bool uxr_buffer_submessage_header(ucdrBuffer *ub, uint8_t id, uint16_t length, uint8_t flags);
bool uxr_read_submessage_header(ucdrBuffer *ub, uint8_t *id, uint16_t *length, uint8_t *flags);

size_t agent_discovery_build_get_info(uint8_t *buf, size_t cap)
{
    ucdrBuffer ub;
    GET_INFO_Payload p;

    ucdr_init_buffer(&ub, buf, (uint32_t)cap);
    p.base.request_id = (RequestId){{0x00, GET_INFO_REQUEST_ID}};
    p.base.object_id = DDS_XRCE_OBJECTID_AGENT;
    p.info_mask = INFO_CONFIGURATION | INFO_ACTIVITY;

    uxr_serialize_message_header(&ub, SESSION_ID_WITHOUT_CLIENT_KEY, 0, 0, 0);
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_GET_INFO, GET_INFO_MSG_SIZE, 0);
    uxr_serialize_GET_INFO_Payload(&ub, &p);
    return ucdr_buffer_length(&ub);
}

static bool read_info(ucdrBuffer *b, INFO_Payload *o)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectReply(b, &o->base);
    ret &= ucdr_deserialize_bool(b, &o->object_info.optional_config);
    if (o->object_info.optional_config) {
        ret &= uxr_deserialize_ObjectVariant(b, &o->object_info.config);
    }
    ret &= ucdr_deserialize_bool(b, &o->object_info.optional_activity);
    if (o->object_info.optional_activity) {
        TransportLocatorSeq *s = &o->object_info.activity._.agent.address_seq;
        ret &= ucdr_deserialize_uint8_t(b, &o->object_info.activity.kind);
        ret &= o->object_info.activity.kind == DDS_XRCE_OBJK_AGENT;
        if (ret) {
            ret &= ucdr_deserialize_int16_t(b, &o->object_info.activity._.agent.availability);
            ret &= ucdr_deserialize_uint32_t(b, &s->size);
            if (s->size > UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX) {
                s->size = UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX;
            }
            for (uint32_t i = 0; i < s->size && ret; i++) {
                ret &= uxr_deserialize_TransportLocator(b, &s->data[i]);
            }
        }
    }
    return ret;
}

bool agent_discovery_parse_info(const uint8_t *buf, size_t len, uint8_t ip[4], uint16_t *port)
{
    static INFO_Payload pl;
    ucdrBuffer ub;
    uint8_t sid, stream, key[4], id, flags;
    uint16_t seq, slen;

    ucdr_init_buffer(&ub, (uint8_t *)buf, (uint32_t)len);
    uxr_deserialize_message_header(&ub, &sid, &stream, &seq, key);
    if (!uxr_read_submessage_header(&ub, &id, &slen, &flags) || id != SUBMESSAGE_ID_INFO) {
        return false;
    }
    if (!read_info(&ub, &pl) || !pl.object_info.optional_config || !pl.object_info.optional_activity) {
        return false;
    }
    if (memcmp(pl.object_info.config._.agent.xrce_version.data,
               DDS_XRCE_XRCE_VERSION.data, sizeof(DDS_XRCE_XRCE_VERSION.data)) != 0) {
        return false;
    }
    TransportLocatorSeq *s = &pl.object_info.activity._.agent.address_seq;
    for (uint32_t i = 0; i < s->size; i++) {
        if (s->data[i].format == ADDRESS_FORMAT_MEDIUM) {
            memcpy(ip, s->data[i]._.medium_locator.address, 4);
            *port = s->data[i]._.medium_locator.locator_port;
            return true;
        }
    }
    return false;
}

#ifndef AGENT_DISCOVERY_NO_IO
static bool same_subnet(const uint8_t a[4], const uint8_t b[4], const uint8_t m[4])
{
    for (int i = 0; i < 4; i++) {
        if ((a[i] & m[i]) != (b[i] & m[i])) return false;
    }
    return true;
}

bool agent_discover_multicast(uint8_t sock, uint16_t local_port,
                              uint32_t attempts, uint32_t period_ms,
                              uint8_t ip[4], uint16_t *port)
{
    static uint8_t group[4] = {239, 255, 0, 2};
    static uint8_t group_mac[6] = {0x01, 0x00, 0x5E, 0x7F, 0x00, 0x02};
    uint8_t msg[32], rx[256], rip[4], lip[4], mask[4];
    uint16_t rport;
    bool found = false;

    size_t n = agent_discovery_build_get_info(msg, sizeof msg);
    getSIPR(lip);
    getSUBR(mask);

    for (int i = 0; i < 50 && wizphy_getphylink() != PHY_LINK_ON; i++) sleep_ms(100);
    sleep_ms(100);

    close(sock);
    setSn_DIPR(sock, group);
    setSn_DPORT(sock, DISCOVERY_PORT);
    setSn_DHAR(sock, group_mac);
    if (socket(sock, Sn_MR_UDP, local_port, SF_MULTI_ENABLE) != sock) return false;

    for (uint32_t a = 0; a < attempts && !found; a++) {
        setSn_DHAR(sock, group_mac);
        if (sendto(sock, msg, (uint16_t)n, group, DISCOVERY_PORT) < 0) continue;
        uint64_t end = time_us_64() + (uint64_t)period_ms * 1000;
        while (!found && time_us_64() < end) {
            if (getSn_RX_RSR(sock) == 0) continue;
            int32_t r = recvfrom(sock, rx, sizeof rx, rip, &rport);
            if (r <= 0 || !agent_discovery_parse_info(rx, (size_t)r, ip, port)) continue;
            if (!same_subnet(ip, lip, mask)) memcpy(ip, rip, 4);
            found = same_subnet(ip, lip, mask);
        }
    }
    close(sock);
    return found;
}
#endif /* AGENT_DISCOVERY_NO_IO */
