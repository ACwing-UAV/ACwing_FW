/**
 * @file  rc_crsf.c
 * @brief CRSF parser (see rc_crsf.h).
 */
#include "rc_crsf.h"
#include <string.h>

uint8_t rc_crsf_crc8(const uint8_t *d, uint8_t n)
{
    uint8_t c = 0;
    for (uint8_t i = 0; i < n; ++i) {
        c ^= d[i];
        for (int k = 0; k < 8; ++k) c = (uint8_t)((c & 0x80u) ? (uint8_t)((c << 1) ^ 0xD5u) : (uint8_t)(c << 1));
    }
    return c;
}

void rc_unpack11(const uint8_t *src, uint8_t n_ch, uint16_t *us)
{
    uint32_t acc = 0; uint8_t bits = 0, j = 0;
    for (uint8_t ch = 0; ch < n_ch; ++ch) {
        while (bits < 11u) { acc |= (uint32_t)src[j++] << bits; bits = (uint8_t)(bits + 8u); }
        const uint16_t v = (uint16_t)(acc & 0x7FFu);
        acc >>= 11; bits = (uint8_t)(bits - 11u);
        us[ch] = (uint16_t)(((int32_t)v * 5) / 8 + 880);          /* 172 -> 987, 992 -> 1500, 1811 -> 2011 */
    }
}

void rc_crsf_init(rc_crsf_t *p) { memset(p, 0, sizeof *p); }

static const uint16_t k_power_mw[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };

static uint8_t decode(const uint8_t *f, rc_frame_t *frame, rc_link_t *link)
{
    const uint8_t type = f[2], plen = (uint8_t)(f[1] - 2u);
    const uint8_t *pl = &f[3];
    if (type == CRSF_T_RC_CHANNELS && plen == 22u) {
        memset(frame, 0, sizeof *frame);
        rc_unpack11(pl, 16u, frame->us);
        frame->n = 16u;
        return RC_GOT_CHANNELS;
    }
    if (type == CRSF_T_LINK_STATS && plen >= 10u) {
        link->rssi = (int8_t)(-(int)(pl[4] ? pl[1] : pl[0]));       /* RSSI of the active antenna, sent as -dBm */
        link->lq = pl[2];
        link->snr = (int8_t)pl[3];
        link->tx_power_mw = (pl[6] < sizeof k_power_mw / sizeof k_power_mw[0]) ? k_power_mw[pl[6]] : 0u;
        return RC_GOT_LINK;
    }
    return RC_NONE;
}

uint8_t rc_crsf_feed(rc_crsf_t *p, uint8_t b, rc_frame_t *frame, rc_link_t *link)
{
    if (p->n == 0u && b != CRSF_ADDR_FC && b != 0xEEu && b != 0xEAu) return RC_NONE;   /* wait for an address */
    if (p->n == 1u && (b < 2u || b > CRSF_MAX_FRAME - 2u)) { p->n = 0; return RC_NONE; }  /* impossible length */
    p->buf[p->n++] = b;
    if (p->n < 2u || p->n < (uint8_t)(p->buf[1] + 2u)) return RC_NONE;
    /* whole frame */
    const uint8_t len = p->buf[1];
    p->n = 0;
    if (rc_crsf_crc8(&p->buf[2], (uint8_t)(len - 1u)) != p->buf[len + 1u]) return RC_NONE;
    return decode(p->buf, frame, link);
}
