/**
 * @file  rc_ibus.c
 * @brief i-BUS parser (see rc_ibus.h).
 */
#include "rc_ibus.h"
#include <string.h>

void rc_ibus_init(rc_ibus_t *p) { memset(p, 0, sizeof *p); }

uint8_t rc_ibus_feed(rc_ibus_t *p, uint8_t b, rc_frame_t *frame)
{
    if (p->n == 0u && b != 0x20u) return RC_NONE;
    if (p->n == 1u && b != 0x40u) { p->n = (b == 0x20u) ? 1u : 0u; return RC_NONE; }
    p->buf[p->n++] = b;
    if (p->n < IBUS_FRAME) return RC_NONE;
    p->n = 0;
    uint16_t sum = 0xFFFFu;
    for (uint8_t i = 0; i < 30u; ++i) sum = (uint16_t)(sum - p->buf[i]);
    if (sum != (uint16_t)(p->buf[30] | (p->buf[31] << 8))) return RC_NONE;
    memset(frame, 0, sizeof *frame);
    for (uint8_t ch = 0; ch < 14u; ++ch) frame->us[ch] = (uint16_t)((p->buf[2 + 2 * ch] | (p->buf[3 + 2 * ch] << 8)) & 0x0FFFu);
    frame->n = 14u;
    return RC_GOT_CHANNELS;
}
