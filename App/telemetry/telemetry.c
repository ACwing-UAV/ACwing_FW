/**
 * @file  telemetry.c
 */
#include "telemetry.h"
#include <string.h>

uint16_t tlm_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t tlm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out, size_t cap)
{
    const size_t total = (size_t)len + TLM_OVERHEAD;
    if (cap < total) return 0;
    out[0] = TLM_SYNC0;
    out[1] = TLM_SYNC1;
    out[2] = type;
    out[3] = len;
    memcpy(&out[4], payload, len);
    const uint16_t crc = tlm_crc16(&out[2], (size_t)len + 2u);
    out[4 + len] = (uint8_t)(crc & 0xFFu);
    out[5 + len] = (uint8_t)(crc >> 8);
    return total;
}
