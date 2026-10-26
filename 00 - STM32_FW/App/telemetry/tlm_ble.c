/**
 * @file  tlm_ble.c
 * @brief BLE telemetry queue and link state (see tlm_ble.h).
 */
#include "tlm_ble.h"
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

static uint8_t           s_ring[TLM_BLE_RING_SIZE];
static volatile uint32_t s_head;          /* written only by the producer */
static volatile uint32_t s_tail;          /* written only by the consumer */
static volatile uint32_t s_dropped;
static volatile uint8_t  s_state;
static volatile uint16_t s_mtu = TLM_BLE_MTU_DEFAULT;
static volatile uint8_t  s_err;
static volatile uint8_t  s_initReq;
static volatile uint8_t  s_decim = APP_BLE_STATE_DECIM_DEFAULT;
static uint16_t          s_lastSeq;
static uint8_t           s_haveSeq;

void tlm_ble_reset(void)
{
    s_head = s_tail = 0u;
    s_dropped = 0u;
    s_state = TLM_BLE_OFF;
    s_mtu = TLM_BLE_MTU_DEFAULT;
    s_err = 0u;
    s_initReq = 0u;
    s_decim = APP_BLE_STATE_DECIM_DEFAULT;
    s_haveSeq = 0u;
}

/* ------------------------------------------------------------ producer */
int tlm_ble_push(const uint8_t *frame, uint16_t len)
{
    if (s_state != TLM_BLE_STREAMING) return 1;
    const uint32_t head = s_head;
    const uint32_t used = head - s_tail;
    if (len == 0u || used + len > TLM_BLE_RING_SIZE) { s_dropped++; return -1; }
    const uint32_t pos   = head & RING_MASK;
    const uint32_t first = (len < TLM_BLE_RING_SIZE - pos) ? len : (TLM_BLE_RING_SIZE - pos);
    memcpy(&s_ring[pos], frame, first);
    memcpy(&s_ring[0], frame + first, len - first);
    TLM_BARRIER();                 /* data visible before the new head */
    s_head = head + len;           /* only whole frames become visible */
    return 0;
}

int tlm_ble_want_state(uint16_t seq)
{
    if (s_state != TLM_BLE_STREAMING) return 0;
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

/* ------------------------------------------------------------ consumer */
uint16_t tlm_ble_queued(void)
{
    return (uint16_t)(s_head - s_tail);
}

uint16_t tlm_ble_peek(uint8_t *dst, uint16_t max)
{
    const uint32_t tail  = s_tail;
    const uint32_t avail = s_head - tail;
    TLM_BARRIER();                 /* head read before the data */
    const uint32_t n     = (avail < max) ? avail : max;
    const uint32_t pos   = tail & RING_MASK;
    const uint32_t first = (n < TLM_BLE_RING_SIZE - pos) ? n : (TLM_BLE_RING_SIZE - pos);
    memcpy(dst, &s_ring[pos], first);
    memcpy(dst + first, &s_ring[0], n - first);
    return (uint16_t)n;
}

void tlm_ble_consume(uint16_t n)
{
    const uint32_t avail = s_head - s_tail;
    TLM_BARRIER();                 /* data read before freeing the space */
    s_tail += (n < avail) ? n : avail;
}

void tlm_ble_discard(void)
{
    s_tail = s_head;
}

/* ------------------------------------------------------------ link events */
void tlm_ble_set_state(tlm_ble_state_t st)
{
    const uint8_t prev = s_state;
    if (st == TLM_BLE_STREAMING && prev != TLM_BLE_STREAMING) {
        s_haveSeq = 0u;
        s_initReq = 1u;            /* the new client needs INIT right away */
    }
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
        break;
    }
}

/* ------------------------------------------------------------ diagnostics */
tlm_ble_state_t tlm_ble_state(void)  { return (tlm_ble_state_t)s_state; }
uint16_t tlm_ble_mtu(void)           { return s_mtu; }
uint16_t tlm_ble_chunk_max(void)     { return (uint16_t)(s_mtu - 3u); }
uint32_t tlm_ble_dropped(void)       { return s_dropped; }
uint8_t  tlm_ble_error(void)         { return s_err; }
uint8_t  tlm_ble_state_decim(void)   { return s_decim; }
