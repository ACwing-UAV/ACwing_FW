/**
 * @file  imu_watch.c
 * @brief IMU stall watch (see imu_watch.h).
 */
#include "imu_watch.h"
#include "proto_v2.h"
#include <string.h>

void imw_init(imw_t *w, uint32_t now_ms)
{
    memset(w, 0, sizeof *w);
    w->last_ms = now_ms;
    w->state = IMW_OK;
}

void imw_sample(imw_t *w, uint32_t now_ms)
{
    w->last_ms = now_ms;
    if (w->state == IMW_LOST && !w->back) { w->back = 1u; w->back_ms = now_ms; }
}

void imw_step(imw_t *w, uint32_t now_ms, imw_out_t *o)
{
    memset(o, 0, sizeof *o);
    const uint32_t quiet = now_ms - w->last_ms;
    if (quiet > 0x80000000u) return;                             /* a sample stamped after now: just arrived */

    if (w->state == IMW_OK) {
        if (quiet <= IMW_STALL_MS) return;
        w->state = IMW_LOST;
        w->since_ms = now_ms;
        w->next_try_ms = now_ms;                                 /* first attempt at once */
        w->tries = 0u;
        w->back = 0u;
        w->outages++;
        o->event = 1u; o->level = PV2_EV_ERROR; o->code = PV2_E_IMU_LOST; o->arg = (int32_t)w->outages;
    }

    /* lost */
    if (w->back && quiet > IMW_STALL_MS) w->back = 0u;           /* came back briefly, then stopped again */
    if (w->back && (now_ms - w->back_ms) >= IMW_OK_MS && quiet <= IMW_FLOW_MS) {
        w->state = IMW_OK;
        w->recoveries++;
        o->event = 1u; o->level = PV2_EV_INFO; o->code = PV2_E_IMU_RECOVERED;
        o->arg = (int32_t)(now_ms - w->since_ms);
        return;
    }
    if (!w->back && (int32_t)(now_ms - w->next_try_ms) >= 0) {
        o->recover = 1u;
        if (w->tries < 0xFFFFu) w->tries++;
        w->next_try_ms = now_ms + IMW_RETRY_MS;
    }
}

uint8_t imw_ok(const imw_t *w) { return (uint8_t)(w->state == IMW_OK); }

uint8_t imw_feed_wdg(const imw_t *w, uint32_t now_ms, uint8_t armed)
{
    if (armed || w->state == IMW_OK) return 1u;
    return (uint8_t)((now_ms - w->since_ms) < IMW_RESET_MS);
}
