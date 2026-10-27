/**
 * @file  tlm_ble.c
 * @brief BLE telemetry queues and link state (see tlm_ble.h).
 */
#include "tlm_ble.h"
#include "proto_v2.h"
#include <string.h>

#ifndef APP_BLE_STATE_DECIM_DEFAULT
#include "app_config.h"
#define APP_BLE_STATE_DECIM_DEFAULT APP_BLE_STATE_DECIM
#endif

#if defined(__arm__)
#define TLM_BARRIER() __asm volatile ("dmb" ::: "memory")
#else
#define TLM_BARRIER() __sync_synchronize()
#endif

#define RING_MASK (TLM_BLE_RING_SIZE - 1u)
#if (TLM_BLE_RING_SIZE & RING_MASK) != 0
#error "TLM_BLE_RING_SIZE must be a power of 2"
#endif

enum { RHI = 0, RLO = 1, NRING = 2 };

typedef struct {
    uint8_t           buf[TLM_BLE_RING_SIZE];
    volatile uint32_t head;          /* written only by the producer */
    volatile uint32_t tail;          /* written only by the consumer */
} ring_t;

static ring_t            s_r[NRING];
static volatile uint32_t s_dropped;
static volatile uint8_t  s_state;
static volatile uint16_t s_mtu = TLM_BLE_MTU_DEFAULT;
static volatile uint8_t  s_err;
static volatile uint8_t  s_initReq, s_subReq, s_open;
static volatile uint8_t  s_decim = APP_BLE_STATE_DECIM_DEFAULT;
static uint16_t          s_lastSeq;
static uint8_t           s_haveSeq;

volatile uint8_t g_dbg_ble_stall;

/* consumer: frame being sent across notifications, and the pending plan */
static int8_t   s_cur = -1;          /* ring of a partially sent frame, -1 none */
static uint16_t s_curRem;
static uint32_t s_planTail[NRING];
static int8_t   s_planCur;
static uint16_t s_planRem, s_planTotal;
static uint8_t  s_planValid;

void tlm_ble_reset(void)
{
    for (int i = 0; i < NRING; ++i) s_r[i].head = s_r[i].tail = 0u;
    s_dropped = 0u;
    s_state = TLM_BLE_OFF;
    s_mtu = TLM_BLE_MTU_DEFAULT;
    s_err = 0u;
    s_initReq = s_subReq = s_open = 0u;
    s_decim = APP_BLE_STATE_DECIM_DEFAULT;
    s_haveSeq = 0u;
    s_cur = -1; s_curRem = 0u; s_planValid = 0u;
    g_dbg_ble_stall = 0u;
}

/* ------------------------------------------------------------ producers */
/* fill level (out of 100) up to which a frame type may be queued */
static uint32_t level_pct(uint8_t type)
{
    switch (type) {
    case PV2_FT_ACK: case PV2_FT_EVENT: case PV2_FT_CAPS:   return 100u;
    case PV2_FT_CALIB: case PV2_FT_PARAM: case PV2_FT_INIT:
    case PV2_FT_STATUS:                                      return 85u;
    case PV2_FT_FLIGHT: case PV2_FT_CHECKS:                  return 70u;
    case PV2_FT_POWER: case PV2_FT_NAV:                      return 55u;
    case PV2_FT_STATE:                                       return 50u;
    default:                                                 return 45u;   /* MOTORS, SYSTEM */
    }
}

static int push(ring_t *r, const uint8_t *frame, uint16_t len)
{
    if (len < 6u || frame[0] != 0xA5u || frame[1] != 0x5Au || frame[3] + 6u != len) { s_dropped++; return -1; }
    const uint32_t head = r->head;
    const uint32_t used = head - r->tail;
    if (used + len > (TLM_BLE_RING_SIZE * level_pct(frame[2])) / 100u) { s_dropped++; return -1; }
    const uint32_t pos   = head & RING_MASK;
    const uint32_t first = (len < TLM_BLE_RING_SIZE - pos) ? len : (TLM_BLE_RING_SIZE - pos);
    memcpy(&r->buf[pos], frame, first);
    memcpy(&r->buf[0], frame + first, len - first);
    TLM_BARRIER();                 /* data visible before the new head */
    r->head = head + len;          /* only whole frames become visible */
    return 0;
}

int tlm_ble_push(const uint8_t *frame, uint16_t len)
{
    if (s_state != TLM_BLE_STREAMING || !s_open) return 1;
    return push(&s_r[RLO], frame, len);
}

int tlm_ble_push_hi(const uint8_t *frame, uint16_t len)
{
    if (s_state != TLM_BLE_STREAMING) return 1;
    return push(&s_r[RHI], frame, len);
}

uint16_t tlm_ble_hi_free(void)
{
    return (uint16_t)(TLM_BLE_RING_SIZE - (s_r[RHI].head - s_r[RHI].tail));
}

int tlm_ble_want_state(uint16_t seq)
{
    if (s_state != TLM_BLE_STREAMING || !s_open) return 0;
    const uint16_t n = s_decim ? s_decim : 1u;
    if (s_haveSeq) {
        const uint16_t d = (uint16_t)(seq - s_lastSeq);   /* wrap-safe; a reboot gives a large d */
        if (d == 0u || d < n) return 0;
    }
    s_lastSeq = seq;
    s_haveSeq = 1u;
    return 1;
}

int tlm_ble_take_init_request(void)
{
    if (!s_initReq) return 0;
    s_initReq = 0u;
    return 1;
}

int tlm_ble_take_subscribe(void)
{
    if (!s_subReq) return 0;
    s_subReq = 0u;
    return 1;
}

void tlm_ble_open_streams(void) { s_open = 1u; }

/* ------------------------------------------------------------ consumer */
static uint32_t avail(int i, const uint32_t *t) { return s_r[i].head - t[i]; }

uint16_t tlm_ble_queued(void)
{
    return (uint16_t)((s_r[RHI].head - s_r[RHI].tail) + (s_r[RLO].head - s_r[RLO].tail));
}

uint16_t tlm_ble_peek(uint8_t *dst, uint16_t max)
{
    uint32_t t[NRING] = { s_r[RHI].tail, s_r[RLO].tail };
    int8_t   cur = s_cur;
    uint16_t rem = s_curRem, n = 0;
    TLM_BARRIER();                 /* heads read before the data */
    while (n < max) {
        if (cur < 0) {             /* frame boundary: HI first */
            if (avail(RHI, t))      cur = RHI;
            else if (avail(RLO, t)) cur = RLO;
            else break;
            rem = (uint16_t)(s_r[cur].buf[(t[cur] + 3u) & RING_MASK] + 6u);
        }
        ring_t *r = &s_r[cur];
        uint16_t take = (uint16_t)(max - n);
        if (take > rem) take = rem;
        const uint32_t pos   = t[cur] & RING_MASK;
        const uint32_t first = (take < TLM_BLE_RING_SIZE - pos) ? take : (TLM_BLE_RING_SIZE - pos);
        memcpy(dst + n, &r->buf[pos], first);
        memcpy(dst + n + first, &r->buf[0], take - first);
        t[cur] += take; n = (uint16_t)(n + take); rem = (uint16_t)(rem - take);
        if (rem == 0u) cur = -1;
    }
    s_planTail[RHI] = t[RHI]; s_planTail[RLO] = t[RLO];
    s_planCur = cur; s_planRem = rem; s_planTotal = n; s_planValid = 1u;
    return n;
}

void tlm_ble_consume(uint16_t n)
{
    if (!s_planValid || n != s_planTotal) return;   /* only the last peek, whole */
    TLM_BARRIER();                                   /* data read before freeing */
    s_r[RHI].tail = s_planTail[RHI];
    s_r[RLO].tail = s_planTail[RLO];
    s_cur = s_planCur; s_curRem = s_planRem;
    s_planValid = 0u;
}

void tlm_ble_discard(void)
{
    s_r[RHI].tail = s_r[RHI].head;
    s_r[RLO].tail = s_r[RLO].head;
    s_cur = -1; s_curRem = 0u; s_planValid = 0u;
}

/* ------------------------------------------------------------ link events */
void tlm_ble_set_state(tlm_ble_state_t st)
{
    const uint8_t prev = s_state;
    if (st == TLM_BLE_STREAMING && prev != TLM_BLE_STREAMING) {
        s_haveSeq = 0u;
        s_open = 0u;               /* LO closed until CAPS, INIT, CHECKS are queued */
        s_subReq = 1u;
    }
    if (st != TLM_BLE_STREAMING) s_open = 0u;
    if (st != TLM_BLE_STREAMING && st != TLM_BLE_CONNECTED) s_mtu = TLM_BLE_MTU_DEFAULT;
    s_state = (uint8_t)st;
}

void tlm_ble_set_mtu(uint16_t att_mtu)
{
    if (att_mtu < TLM_BLE_MTU_DEFAULT) att_mtu = TLM_BLE_MTU_DEFAULT;
    s_mtu = att_mtu;
}

void tlm_ble_set_error(uint8_t ble_status) { s_err = ble_status; }

void tlm_ble_on_command(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0u) return;
    switch (data[0]) {
    case TLM_BLE_CMD_SEND_INIT:
        s_initReq = 1u;
        break;
    case TLM_BLE_CMD_STATE_DECIM:
        if (len >= 2u && data[1] >= 1u) { s_decim = data[1]; s_haveSeq = 0u; }
        break;
    default:
        break;                     /* protocol v2: app_link.c */
    }
}

/* ------------------------------------------------------------ diagnostics */
tlm_ble_state_t tlm_ble_state(void)  { return (tlm_ble_state_t)s_state; }
uint16_t tlm_ble_mtu(void)           { return s_mtu; }
uint16_t tlm_ble_chunk_max(void)     { return (uint16_t)(s_mtu - 3u); }
uint32_t tlm_ble_dropped(void)       { return s_dropped; }
uint8_t  tlm_ble_error(void)         { return s_err; }
uint8_t  tlm_ble_state_decim(void)   { return s_decim; }
