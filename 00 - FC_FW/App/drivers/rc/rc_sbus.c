/**
 * @file  rc_sbus.c
 * @brief SBUS parser (see rc_sbus.h).
 */
#include "rc_sbus.h"
#include "rc_crsf.h"
#include <string.h>

void rc_sbus_init(rc_sbus_t *p) { memset(p, 0, sizeof *p); }

static uint8_t end_ok(uint8_t e) { return (uint8_t)(e == 0x00u || (e & 0x0Fu) == 0x04u); }

uint8_t rc_sbus_feed(rc_sbus_t *p, uint8_t b, rc_frame_t *frame)
{
    if (p->n == 0u && b != 0x0Fu) return RC_NONE;
    p->buf[p->n++] = b;
    if (p->n < SBUS_FRAME) return RC_NONE;
    p->n = 0;
    if (!end_ok(p->buf[24]) || (p->buf[23] & 0xF0u)) {
        /* out of sync: retry from the next 0x0F inside this buffer */
        for (uint8_t i = 1; i < SBUS_FRAME; ++i)
            if (p->buf[i] == 0x0Fu) { memmove(p->buf, &p->buf[i], SBUS_FRAME - i); p->n = (uint8_t)(SBUS_FRAME - i); break; }
        return RC_NONE;
    }
    memset(frame, 0, sizeof *frame);
    rc_unpack11(&p->buf[1], 16u, frame->us);
    frame->n = 16u;
    frame->frame_lost = (uint8_t)((p->buf[23] >> 2) & 1u);
    frame->failsafe = (uint8_t)((p->buf[23] >> 3) & 1u);
    return RC_GOT_CHANNELS;
}
