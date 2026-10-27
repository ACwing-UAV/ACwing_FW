/**
 * @file  ack_cache.c
 */
#include "ack_cache.h"
#include <string.h>

typedef struct { uint8_t used, state, op, seq, result, reason; uint32_t age; } entry_t;

static entry_t  s_e[ACK_CACHE_LEN];
static uint32_t s_clock;

void ackc_reset(void) { memset(s_e, 0, sizeof s_e); s_clock = 0; }

static entry_t *find(uint8_t op, uint8_t seq)
{
    for (unsigned i = 0; i < ACK_CACHE_LEN; ++i)
        if (s_e[i].used && s_e[i].op == op && s_e[i].seq == seq) return &s_e[i];
    return 0;
}

static entry_t *slot(void)
{
    entry_t *best = &s_e[0];
    for (unsigned i = 0; i < ACK_CACHE_LEN; ++i) {
        if (!s_e[i].used) return &s_e[i];
        if (s_e[i].age < best->age) best = &s_e[i];      /* oldest */
    }
    return best;
}

uint8_t ackc_begin(uint8_t op, uint8_t seq, uint8_t *result, uint8_t *reason)
{
    entry_t *e = find(op, seq);
    if (e) {
        if (e->state == ACKC_DONE) { *result = e->result; *reason = e->reason; }
        return e->state;
    }
    e = slot();
    e->used = 1; e->state = ACKC_PENDING; e->op = op; e->seq = seq;
    e->result = e->reason = 0; e->age = ++s_clock;
    return ACKC_NEW;
}

void ackc_finish(uint8_t op, uint8_t seq, uint8_t result, uint8_t reason)
{
    entry_t *e = find(op, seq);
    if (!e) { e = slot(); e->used = 1; e->op = op; e->seq = seq; e->age = ++s_clock; }
    e->state = ACKC_DONE; e->result = result; e->reason = reason;
}
