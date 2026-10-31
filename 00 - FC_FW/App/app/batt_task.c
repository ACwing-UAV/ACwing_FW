/**
 * @file  batt_task.c
 * @brief BattTask (see batt_task.h).
 */
#include "batt_task.h"
#include "fc_core.h"
#include "params.h"
#include "proto_v2.h"
#include "spsc.h"
#include "log_feed.h"
#include <string.h>

typedef struct { float scale_v, scale_i; } scales_t;
SPSC_DEFINE(s_scq, scales_t, 4u);

static batt_task_cfg_t s_cfg;
static uint32_t        s_last;
static uint8_t         s_have_last;
static uint8_t         s_sig_level = 0xFFu, s_sig_present = 0xFFu;
static uint8_t         s_was_armed;
static uint8_t         s_log_div;
static float           s_used_at_arm;

static volatile uint32_t s_fl_seq;
static batt_flight_t     s_fl;

void batt_task_init(const batt_task_cfg_t *cfg)
{
    s_cfg = *cfg;
    batt_init(&s_cfg.batt);
    spsc_reset(&s_scq);
    s_have_last = 0u;
    s_sig_level = s_sig_present = 0xFFu;
    s_was_armed = 0u;
    s_used_at_arm = 0.0f;
    memset(&s_fl, 0, sizeof s_fl);
    s_fl_seq = 0u;
}

int batt_task_post_scales(float sv, float si)
{
    const scales_t s = { sv, si };
    return spsc_push(&s_scq, &s);
}

void batt_task_last_flight(batt_flight_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_fl_seq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        batt_flight_t v = s_fl;
        SPSC_BARRIER();
        if (s_fl_seq == s0) { *out = v; return; }
    }
}

void batt_task_step(uint32_t now)
{
    const uint32_t dt = s_have_last ? (now - s_last) : BATT_TASK_PERIOD_MS;
    s_last = now; s_have_last = 1u;

    scales_t sc;
    while (spsc_pop(&s_scq, &sc)) batt_set_scales(sc.scale_v, sc.scale_i);

    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);

    float pv = 0.0f, pi = 0.0f;
    if (s_cfg.adc_read && s_cfg.adc_read(&pv, &pi) == 0) {
        const batt_limits_t L = {
            PARAM(P_BATT_WARN), PARAM(P_BATT_CRIT), PARAM(P_BATT_EMERG),
            PARAM(P_BATT_CAPACITY), (uint8_t)(PARAM(P_BATT_CELLS) + 0.5f)
        };
        batt_step(pv, pi, dt, f.armed, &L);
    }
    const batt_state_t *b = batt_state();

    /* signal the flight core on a change of level or presence */
    if (b->level != s_sig_level || b->present != s_sig_present) {
        const float mv = b->cell_v * 1000.0f;
        const fc_batt_sig_t sig = { b->present, b->level, (uint16_t)(mv > 65535.0f ? 65535.0f : mv) };
        if (fc_batt_signal(&sig) == 0) { s_sig_level = b->level; s_sig_present = b->present; }
        /* queue full: retried at the next step */
    }

    /* charge of the flight, for the BATTERY calibration */
    if (f.armed && !s_was_armed) s_used_at_arm = b->used_mah;
    if (!f.armed && s_was_armed) {
        batt_cfg_t c;
        batt_get_cfg(&c);
        s_fl_seq++;
        SPSC_BARRIER();
        s_fl.count++;
        s_fl.used_mah = b->used_mah - s_used_at_arm;
        s_fl.scale_i = c.scale_i;
        SPSC_BARRIER();
        s_fl_seq++;
    }
    s_was_armed = f.armed;

    /* flight log, 10 Hz */
    if (++s_log_div >= 5u) { s_log_div = 0u; logf_power(now, b); }

    fc_batt_heartbeat(now);
}

/* ------------------------------------------------------------------ RTOS shell */
#ifndef HOST_TEST
#include "cmsis_os2.h"

static void BattTask(void *arg)
{
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        batt_task_step(osKernelGetTickCount());
        next += BATT_TASK_PERIOD_MS;
        (void)osDelayUntil(next);
    }
}

void batt_task_start(const batt_task_cfg_t *cfg)
{
    batt_task_init(cfg);
    /* above the AppTask / ExtTask, below the ImuTask: the battery state must
     * reach the flight core even while the app protocol is busy */
    static const osThreadAttr_t a = { .name = "batt", .priority = osPriorityNormal,
                                      .stack_size = 768 };
    (void)osThreadNew(BattTask, NULL, &a);
}
#endif
