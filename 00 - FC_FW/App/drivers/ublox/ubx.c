/**
 * @file  ubx.c
 * @brief UBX parser, builders and NAV-PVT decoder (see ubx.h).
 */
#include "ubx.h"
#include <string.h>

enum { S_SYNC1 = 0, S_SYNC2, S_CLS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };

void ubx_parser_reset(ubx_parser_t *p)
{
    const uint32_t bad = p->bad_ck, skip = p->skipped;
    memset(p, 0, sizeof *p);
    p->bad_ck = bad; p->skipped = skip;
}

static void ck_add(ubx_parser_t *p, uint8_t b) { p->ck_a = (uint8_t)(p->ck_a + b); p->ck_b = (uint8_t)(p->ck_b + p->ck_a); }

int ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC1: if (b == UBX_SYNC1) p->state = S_SYNC2; break;
    case S_SYNC2:
        if (b == UBX_SYNC2) { p->state = S_CLS; p->ck_a = p->ck_b = 0u; }
        else p->state = (b == UBX_SYNC1) ? S_SYNC2 : S_SYNC1;
        break;
    case S_CLS:  p->cls = b; ck_add(p, b); p->state = S_ID; break;
    case S_ID:   p->id = b;  ck_add(p, b); p->state = S_LEN1; break;
    case S_LEN1: p->len = b; ck_add(p, b); p->state = S_LEN2; break;
    case S_LEN2:
        p->len = (uint16_t)(p->len | ((uint16_t)b << 8)); ck_add(p, b);
        p->n = 0u;
        p->state = p->len ? S_PAYLOAD : S_CKA;
        break;
    case S_PAYLOAD:
        ck_add(p, b);
        if (p->n < UBX_MAX_PAYLOAD) p->buf[p->n] = b;
        if (++p->n >= p->len) p->state = S_CKA;
        break;
    case S_CKA:
        if (b != p->ck_a) { p->bad_ck++; p->state = (b == UBX_SYNC1) ? S_SYNC2 : S_SYNC1; break; }
        p->state = S_CKB;
        break;
    case S_CKB:
        p->state = S_SYNC1;
        if (b != p->ck_b) { p->bad_ck++; break; }
        if (p->len > UBX_MAX_PAYLOAD) { p->skipped++; break; }
        return 1;
    default: p->state = S_SYNC1; break;
    }
    return 0;
}

void ubx_checksum(const uint8_t *d, uint16_t n, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0u, b = 0u;
    for (uint16_t i = 0; i < n; ++i) { a = (uint8_t)(a + d[i]); b = (uint8_t)(b + a); }
    *ck_a = a; *ck_b = b;
}

uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t cap)
{
    const uint32_t total = 8u + (uint32_t)len;
    if (total > cap) return 0u;
    out[0] = UBX_SYNC1; out[1] = UBX_SYNC2; out[2] = cls; out[3] = id;
    out[4] = (uint8_t)len; out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(&out[6], payload, len);
    ubx_checksum(&out[2], (uint16_t)(4u + len), &out[6 + len], &out[7 + len]);
    return (uint16_t)total;
}

/* value size in bytes from the key: bits 28..30 = 1 (1 bit), 2 (1 B), 3 (2 B), 4 (4 B), 5 (8 B, not used here) */
static uint8_t key_size(uint32_t key)
{
    switch ((key >> 28) & 0x7u) {
    case 1u: case 2u: return 1u;
    case 3u: return 2u;
    case 4u: return 4u;
    default: return 0u;
    }
}

uint16_t ubx_valset(uint8_t layers, const ubx_kv_t *kv, uint8_t n, uint8_t *out, uint16_t cap)
{
    uint8_t pl[64];
    uint16_t len = 0u;
    pl[len++] = 0u; pl[len++] = layers; pl[len++] = 0u; pl[len++] = 0u;
    for (uint8_t i = 0; i < n; ++i) {
        const uint8_t sz = key_size(kv[i].key);
        if (!sz || len + 4u + sz > sizeof pl) return 0u;
        for (uint8_t k = 0; k < 4u; ++k) pl[len++] = (uint8_t)(kv[i].key >> (8u * k));
        for (uint8_t k = 0; k < sz; ++k) pl[len++] = (uint8_t)(kv[i].value >> (8u * k));
    }
    return ubx_frame(UBX_CLS_CFG, UBX_ID_CFG_VALSET, pl, len, out, cap);
}

static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t u32le(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

int ubx_decode_pvt(const uint8_t *d, uint16_t len, ubx_pvt_t *v)
{
    if (len != UBX_LEN_NAV_PVT) return -1;
    v->itow_ms = u32le(&d[0]);
    v->year = u16le(&d[4]);
    v->month = d[6]; v->day = d[7]; v->hour = d[8]; v->min = d[9]; v->sec = d[10];
    v->valid = d[11];
    v->fix_type = d[20];
    v->flags = d[21];
    v->num_sv = d[23];
    v->lon_e7 = (int32_t)u32le(&d[24]);
    v->lat_e7 = (int32_t)u32le(&d[28]);
    v->h_ellips_mm = (int32_t)u32le(&d[32]);
    v->h_msl_mm = (int32_t)u32le(&d[36]);
    v->h_acc_mm = u32le(&d[40]);
    v->v_acc_mm = u32le(&d[44]);
    for (int i = 0; i < 3; ++i) v->vel_ned_mms[i] = (int32_t)u32le(&d[48 + 4 * i]);
    v->gspeed_mms = (int32_t)u32le(&d[60]);
    v->head_mot_e5 = (int32_t)u32le(&d[64]);
    v->s_acc_mms = u32le(&d[68]);
    v->pdop_x100 = u16le(&d[76]);
    v->invalid_llh = (uint8_t)(d[78] & 0x01u);
    return 0;
}

uint8_t ubx_pvt_usable(const ubx_pvt_t *v)
{
    return (uint8_t)((v->fix_type == UBX_FIX_3D || v->fix_type == UBX_FIX_GNSS_DR) &&
                     (v->flags & 0x01u) && !v->invalid_llh);
}
