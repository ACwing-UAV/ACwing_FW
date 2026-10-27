/**
 * @file  fc_shared.c
 */
#include "fc_shared.h"
#include "spsc.h"

volatile uint8_t g_fsh_cal_want;

static volatile uint32_t s_seq;
static fsh_est_t         s_est;

SPSC_DEFINE(s_calq, cal_sample_t, 32u);
SPSC_DEFINE(s_appq, fsh_apply_t, 4u);

void fsh_reset(void)
{
    s_seq = 0u;
    g_fsh_cal_want = 0u;
    spsc_reset(&s_calq);
    spsc_reset(&s_appq);
}

void fsh_est_write(const fsh_est_t *s)
{
    s_seq++;
    SPSC_BARRIER();
    s_est = *s;
    SPSC_BARRIER();
    s_seq++;
}

void fsh_est_read(fsh_est_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_seq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        const fsh_est_t v = s_est;
        SPSC_BARRIER();
        if (s_seq == s0) { *out = v; return; }
    }
}

int fsh_cal_push(const cal_sample_t *s) { return spsc_push(&s_calq, s); }
int fsh_cal_pop(cal_sample_t *s)        { return spsc_pop(&s_calq, s); }

int fsh_apply_post(uint8_t type, const cal_set_t *set)
{
    fsh_apply_t a;
    a.type = type;
    a.set  = *set;
    return spsc_push(&s_appq, &a);
}

int fsh_apply_take(fsh_apply_t *a) { return spsc_pop(&s_appq, a); }
