/**
 * @file  app_link.c
 * @brief CMD reception: arrival time, RC store, command queue (see app_link.h).
 */
#include "app_link.h"
#include "proto_v2.h"
#include <string.h>

#if defined(__arm__)
#define APPL_BARRIER() __asm volatile ("dmb" ::: "memory")
#else
#define APPL_BARRIER() __sync_synchronize()
#endif

#define QMASK (APPL_QUEUE_LEN - 1u)
#if (APPL_QUEUE_LEN & QMASK) != 0
#error "APPL_QUEUE_LEN must be a power of 2"
#endif

static appl_cmd_t        s_q[APPL_QUEUE_LEN];
static volatile uint32_t s_head, s_tail, s_qdrop;

static volatile uint32_t s_last_ms;
static volatile uint8_t  s_ever;
static volatile uint8_t  s_down;

static volatile uint32_t s_rc_seq;        /* odd while the writer is in the middle */
static volatile uint32_t s_rc_ms;
static volatile uint8_t  s_rc_ever;
static appl_rc_t         s_rc;

void appl_reset(void)
{
    s_head = s_tail = s_qdrop = 0u;
    s_last_ms = 0u; s_ever = 0u; s_down = 0u;
    s_rc_seq = 0u; s_rc_ms = 0u; s_rc_ever = 0u;
    memset(&s_rc, 0, sizeof s_rc);
}

/* ------------------------------------------------------------ BLE context */
void appl_on_cmd_write(const uint8_t *d, uint16_t len, uint32_t now_ms)
{
    if (!d || len == 0u) return;

    /* every CMD frame refreshes the app-link time: one atomic 32-bit store */
    s_last_ms = now_ms;
    APPL_BARRIER();
    s_ever = 1u;
    s_down = 0u;

    const uint8_t op = d[0];
    if (op < PV2_OP_HEARTBEAT) return;         /* legacy 0x01/0x02: handled inline */
    if (op == PV2_OP_HEARTBEAT) return;        /* time only */

    if (op == PV2_OP_RC) {
        if (len < 11u) return;                 /* op seq thr yaw pitch roll buttons */
        s_rc_seq++;                            /* odd: write in progress */
        APPL_BARRIER();
        s_rc.thr     = pv2_rd_i16(&d[2]);
        s_rc.yaw     = pv2_rd_i16(&d[4]);
        s_rc.pitch   = pv2_rd_i16(&d[6]);
        s_rc.roll    = pv2_rd_i16(&d[8]);
        s_rc.buttons = d[10];
        s_rc_ms      = now_ms;
        APPL_BARRIER();
        s_rc_seq++;                            /* even: consistent */
        s_rc_ever = 1u;
        return;
    }

    /* any other command: copy into the queue, never block */
    const uint32_t head = s_head;
    if (head - s_tail >= APPL_QUEUE_LEN) { s_qdrop++; return; }
    appl_cmd_t *c = &s_q[head & QMASK];
    c->len = (uint8_t)((len > APPL_CMD_MAX) ? APPL_CMD_MAX : len);
    memcpy(c->b, d, c->len);
    APPL_BARRIER();
    s_head = head + 1u;
}

void appl_on_connect(void)    { }
void appl_on_disconnect(void) { s_down = 1u; }

/* ------------------------------------------------------------ AppTask */
int appl_pop(appl_cmd_t *out)
{
    const uint32_t tail = s_tail;
    if (s_head == tail) return 0;
    APPL_BARRIER();
    *out = s_q[tail & QMASK];
    APPL_BARRIER();
    s_tail = tail + 1u;
    return 1;
}

uint32_t appl_queue_dropped(void) { return s_qdrop; }

/* ------------------------------------------------------------ readers */
uint8_t  appl_ever(void)      { return s_ever; }
uint32_t appl_last_ms(void)   { return s_last_ms; }
uint8_t  appl_link_down(void) { return s_down; }

uint32_t appl_silence_ms(uint32_t now_ms, uint32_t down_floor_ms)
{
    if (!s_ever) return UINT32_MAX;
    uint32_t s = now_ms - s_last_ms;
    if (s > 0x80000000u) s = 0u;              /* stamp newer than 'now': just arrived */
    if (s_down && s < down_floor_ms) s = down_floor_ms;
    return s;
}

int appl_rc_read(appl_rc_t *out, uint32_t *t_ms)
{
    if (!s_rc_ever) return 0;
    for (int tries = 0; tries < 4; ++tries) {
        const uint32_t s0 = s_rc_seq;
        if (s0 & 1u) continue;                 /* writer busy: retry */
        APPL_BARRIER();
        const appl_rc_t v = s_rc;
        const uint32_t  t = s_rc_ms;
        APPL_BARRIER();
        if (s_rc_seq == s0) { *out = v; if (t_ms) *t_ms = t; return 1; }
    }
    return 0;                                  /* keep the caller's previous copy */
}
