/**
 * @file  cal_sticks.c
 * @brief STICKS calibration procedure (see cal_sticks.h).
 */
#include "cal_sticks.h"
#include <string.h>

void cs_start(cs_proc_t *p)
{
    memset(p, 0, sizeof *p);
    p->step = 1u;
}

/* AUX position: 0 low, 1 mid, 2 high, -1 no signal */
static int aux_position(uint16_t us)
{
    if (us < 800u) return -1;
    if (us < 1300u) return 0;
    return (us > 1700u) ? 2 : 1;
}

static void track_ranges(cs_proc_t *p, const uint16_t s[4], const uint16_t aux[4])
{
    for (int i = 0; i < 4; ++i) {
        if (s[i] < p->r.min[i]) p->r.min[i] = s[i];
        if (s[i] > p->r.max[i]) p->r.max[i] = s[i];
        const int pos = aux_position(aux[i]);
        if (pos >= 0) p->aux_seen[i] |= (uint8_t)(1u << pos);
    }
}

void cs_sample(cs_proc_t *p, const uint16_t s[4], const uint16_t aux[4])
{
    memcpy(p->live, s, sizeof p->live);
    if (p->step == 2u) track_ranges(p, s, aux);
}

/* number of positions seen -> 0 unused (one), 2 two-position, 3 three-position */
static uint8_t aux_kind(uint8_t seen)
{
    const uint8_t n = (uint8_t)((seen & 1u) + ((seen >> 1) & 1u) + ((seen >> 2) & 1u));
    return (n >= 3u) ? 3u : (n == 2u ? 2u : 0u);
}

uint8_t cs_valid(const cal_sticks_t *c)
{
    if (!(c->min[0] + 2u * CS_MIN_SPAN < c->max[0])) return 0u;                    /* throttle range */
    for (int i = 1; i < 4; ++i)
        if (!(c->min[i] + CS_MIN_SPAN < c->centre[i] && c->centre[i] + CS_MIN_SPAN < c->max[i])) return 0u;
    return 1u;
}

uint8_t cs_capture(cs_proc_t *p)
{
    if (p->step == 1u) {
        for (int i = 0; i < 4; ++i) if (p->live[i] < 800u || p->live[i] > 2200u) return 0u;   /* no receiver yet */
        for (int i = 0; i < 4; ++i) { p->r.centre[i] = p->r.min[i] = p->r.max[i] = p->live[i]; }
        memset(p->aux_seen, 0, sizeof p->aux_seen);
        p->step = 2u;
        return 1u;
    }
    if (p->step == 2u) {
        if (!cs_valid(&p->r)) return 0u;
        for (int i = 0; i < 4; ++i) p->r.aux_pos[i] = aux_kind(p->aux_seen[i]);
        p->step = 3u;
        return 1u;
    }
    return 0u;
}

uint8_t cs_progress(const cs_proc_t *p)
{
    if (p->step >= 3u) return 100u;
    if (p->step != 2u) return 0u;
    uint8_t ends = 0;
    if (p->r.max[0] > p->r.min[0] + 2u * CS_MIN_SPAN) ends = 2u;
    for (int i = 1; i < 4; ++i) {
        ends = (uint8_t)(ends + (p->r.min[i] + CS_MIN_SPAN < p->r.centre[i]));
        ends = (uint8_t)(ends + (p->r.centre[i] + CS_MIN_SPAN < p->r.max[i]));
    }
    return (uint8_t)(ends * 100u / 8u);
}
