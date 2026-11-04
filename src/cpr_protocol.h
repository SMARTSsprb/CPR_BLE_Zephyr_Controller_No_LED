/**
 * @file    cpr_protocol.h
 * @brief   CPR Binary Protocol Serialization & Builders
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t cpr_protocol_calc_checksum(const uint8_t *buf, size_t n);

CprPacket cpr_protocol_build_packet(uint8_t type, int8_t depth, uint16_t packet_id, const uint8_t device_id[3]);

AnnouncePacket cpr_protocol_build_announce(uint16_t packet_id, const uint8_t device_id[3], int8_t battery_pct);

#ifdef __cplusplus
}
#endif

