/**
 * @file  proto_v2.c
 * @brief Frame encoders of protocol v2 (see proto_v2.h).
 */
#include "proto_v2.h"
#include "telemetry.h"
#include <string.h>

size_t pv2_frame(uint8_t type, const void *payload, uint8_t len, uint8_t *out, size_t cap)
{
    return tlm_encode(type, payload, len, out, cap);
}

size_t pv2_event(uint8_t level, uint16_t code, int32_t arg, const char *text, uint8_t *out, size_t cap)
{
    uint8_t pl[sizeof(pv2_event_hdr_t) + PV2_EVENT_TEXT_MAX];
    size_t n = text ? strlen(text) : 0u;
    if (n > PV2_EVENT_TEXT_MAX) n = PV2_EVENT_TEXT_MAX;
    pv2_event_hdr_t h;
    h.level = level; h.code = code; h.arg = arg; h.len = (uint8_t)n;
    memcpy(pl, &h, sizeof h);
    if (n) memcpy(pl + sizeof h, text, n);
    return pv2_frame(PV2_FT_EVENT, pl, (uint8_t)(sizeof h + n), out, cap);
}

size_t pv2_ack(uint8_t op, uint8_t seq, uint8_t result, uint8_t reason, uint8_t *out, size_t cap)
{
    const pv2_ack_t a = { op, seq, result, reason };
    return pv2_frame(PV2_FT_ACK, &a, sizeof a, out, cap);
}

size_t pv2_param(uint16_t id, float value, uint8_t flags, uint16_t index, uint16_t count, uint8_t *out, size_t cap)
{
    pv2_param_t p;
    p.id = id;
    memcpy(&p.value, &value, sizeof value);
    p.flags = flags; p.index = index; p.count = count;
    return pv2_frame(PV2_FT_PARAM, &p, sizeof p, out, cap);
}

size_t pv2_checks(uint8_t ready, const pv2_check_t *items, uint8_t n, uint8_t *out, size_t cap)
{
    uint8_t pl[2u + 4u * PV2_CHECKS_MAX];
    if (n > PV2_CHECKS_MAX) n = PV2_CHECKS_MAX;
    pl[0] = ready ? 1u : 0u;
    pl[1] = n;
    memcpy(&pl[2], items, 4u * n);
    return pv2_frame(PV2_FT_CHECKS, pl, (uint8_t)(2u + 4u * n), out, cap);
}

size_t pv2_motors(const pv2_motor_t *m, uint8_t n, uint8_t *out, size_t cap)
{
    uint8_t pl[1u + 4u * PV2_MOTORS_MAX];
    if (n > PV2_MOTORS_MAX) n = PV2_MOTORS_MAX;
    pl[0] = n;
    memcpy(&pl[1], m, 4u * n);
    return pv2_frame(PV2_FT_MOTORS, pl, (uint8_t)(1u + 4u * n), out, cap);
}

size_t pv2_calib(const pv2_calib_hdr_t *h, const void *data, uint8_t data_len, uint8_t *out, size_t cap)
{
    uint8_t pl[sizeof(pv2_calib_hdr_t) + sizeof(pv2_cal_sticks_t)];
    if (data_len > sizeof(pv2_cal_sticks_t)) return 0;
    memcpy(pl, h, sizeof *h);
    if (data_len) memcpy(pl + sizeof *h, data, data_len);
    return pv2_frame(PV2_FT_CALIB, pl, (uint8_t)(sizeof *h + data_len), out, cap);
}

float pv2_rd_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, sizeof f);       /* little-endian target and host */
    return f;
}
