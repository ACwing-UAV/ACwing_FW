/**
 * @file  rc_dsm.c
 * @brief Spektrum DSM serial parser (see rc_dsm.h).
 */
#include "rc_dsm.h"
#include <string.h>

void rc_dsm_init(rc_dsm_t *p)
{
    memset(p, 0, sizeof *p);
    for (uint8_t i = 0; i < DSM_CH; ++i) p->us[i] = 1500u;
}

/* system byte -> 11-bit (2048) resolution */
static uint8_t is_2048(uint8_t sys) { return (uint8_t)(sys != 0x01u); }

uint8_t rc_dsm_feed(rc_dsm_t *p, uint8_t b, uint32_t now, rc_frame_t *frame)
{
    if (p->have_last && (now - p->last_ms) >= DSM_GAP_MS) p->n = 0;     /* a gap: frame start */
    p->last_ms = now; p->have_last = 1u;
    if (p->n >= DSM_FRAME) return RC_NONE;                             /* wait for the next gap */
    p->buf[p->n++] = b;
    if (p->n < DSM_FRAME) return RC_NONE;
    const uint8_t hi = is_2048(p->buf[1]);
    for (uint8_t w = 0; w < 7u; ++w) {
        const uint16_t v = (uint16_t)((p->buf[2 + 2 * w] << 8) | p->buf[3 + 2 * w]);
        if (v == 0xFFFFu) continue;                                    /* unused slot */
        const uint8_t id = hi ? (uint8_t)((v >> 11) & 0x0Fu) : (uint8_t)((v >> 10) & 0x0Fu);
        const uint16_t val = hi ? (uint16_t)(v & 0x07FFu) : (uint16_t)(v & 0x03FFu);
        if (id >= DSM_CH) continue;
        /* Spektrum: 2048 -> us = 0.583 v + 903; 1024 -> us = 1.166 v + 903 */
        p->us[id] = hi ? (uint16_t)((val * 583u) / 1000u + 903u) : (uint16_t)((val * 1166u) / 1000u + 903u);
    }
    memset(frame, 0, sizeof *frame);
    memcpy(frame->us, p->us, sizeof p->us);
    frame->n = DSM_CH;
    return RC_GOT_CHANNELS;
}
