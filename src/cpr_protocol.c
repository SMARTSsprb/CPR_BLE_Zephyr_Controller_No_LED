/**
 * @file    cpr_protocol.c
 * @brief   CPR Binary Protocol Serialization Implementation (Zephyr RTOS)
 * @version 9.0.0
 */

#include "cpr_protocol.h"
#include <zephyr/kernel.h>
#include <string.h>

uint8_t cpr_protocol_calc_checksum(const uint8_t *buf, size_t n)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < n; ++i) {
        cs ^= buf[i];
    }
    return cs;
}

CprPacket cpr_protocol_build_packet(uint8_t type, int8_t depth, uint16_t packet_id, const uint8_t device_id[3])
{
    CprPacket p;
    memset(&p, 0, sizeof(p));

    p.magic[0]    = PACKET_MAGIC_0;
    p.magic[1]    = PACKET_MAGIC_1;
    p.version     = PROTOCOL_VER;
    p.type        = type;
    p.timestamp   = k_uptime_get_32();
    p.id          = packet_id;
    p.depth       = depth;
    p.deviceID[0] = device_id[0];
    p.deviceID[1] = device_id[1];
    p.deviceID[2] = device_id[2];
    p.checksum    = cpr_protocol_calc_checksum((const uint8_t *)&p, sizeof(p) - 1u);

    return p;
}

AnnouncePacket cpr_protocol_build_announce(uint16_t packet_id, const uint8_t device_id[3], int8_t battery_pct)
{
    AnnouncePacket p;
    memset(&p, 0, sizeof(p));

    p.magic[0]       = PACKET_MAGIC_0;
    p.magic[1]       = PACKET_MAGIC_1;
    p.version        = PROTOCOL_VER;
    p.type           = PTYPE_ANNOUNCE;
    p.timestamp      = k_uptime_get_32();
    p.id             = packet_id;
    p.depth          = battery_pct;
    p.deviceID[0]    = device_id[0];
    p.deviceID[1]    = device_id[1];
    p.deviceID[2]    = device_id[2];
    p.manikinName[0] = MANIKIN_NAME_STR[0];
    p.manikinName[1] = MANIKIN_NAME_STR[1];
    p.manikinName[2] = MANIKIN_NAME_STR[2];
    p.manikinName[3] = '\0';
    p.manikinType    = MANIKIN_TYPE_DEFAULT;
    p.checksum       = cpr_protocol_calc_checksum((const uint8_t *)&p, sizeof(p) - 1u);

    return p;
}

