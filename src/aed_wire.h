#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Companion protocol v1: magic, version, event, little-endian sequence. */
enum aed_event { AED_HELLO = 0, AED_POWER_OFF, AED_POWER_ON, AED_SIM_DELIVERED };
struct aed_stream { bool ready; uint16_t last; };

static inline int aed_decode(struct aed_stream *s, const uint8_t *p, size_t n)
{
    if (!p || n != 5 || p[0] != 0xae || p[1] != 1 || p[2] > AED_SIM_DELIVERED) return -1;
    uint16_t seq = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
    if (!s->ready && p[2] != AED_HELLO) return -1;
    if (s->ready) {
        uint16_t delta = (uint16_t)(seq - s->last);
        if (p[2] == AED_HELLO || delta == 0 || delta >= 0x8000) return -1;
    }
    s->last = seq;
    s->ready = true;
    return p[2];
}
