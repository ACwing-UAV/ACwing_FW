/**
 * @file  log_queue.c
 * @brief Record queues (see log_queue.h).
 */
#include "log_queue.h"
#include "spsc.h"
#include <string.h>

typedef struct {
    uint8_t          *buf;
    uint32_t          size;               /* power of two */
    volatile uint32_t head, tail;         /* free-running byte counters */
    volatile uint16_t drops;              /* producer increments ...   */
    uint16_t          drops_seen;         /* ... consumer reads         */
} ring_t;

static uint8_t s_b_imu[LOG_Q_IMU_SIZE], s_b_batt[LOG_Q_BATT_SIZE];
static ring_t  s_r[LOG_SRC_COUNT] = {
    { s_b_imu,  LOG_Q_IMU_SIZE,  0, 0, 0, 0 },
    { s_b_batt, LOG_Q_BATT_SIZE, 0, 0, 0, 0 },
};
static volatile uint8_t s_active;

uint8_t log_q_active(void) { return s_active; }
void    log_q_set_active(uint8_t on) { s_active = on; }

void log_q_reset(void)
{
    for (int i = 0; i < LOG_SRC_COUNT; ++i) { s_r[i].head = s_r[i].tail = 0; s_r[i].drops = s_r[i].drops_seen = 0; }
    s_active = 0;
}

int log_q_put(uint8_t src, uint8_t type, const void *p, uint8_t len)
{
    if (src >= LOG_SRC_COUNT) return -1;
    ring_t *r = &s_r[src];
    const uint32_t h = r->head;
    SPSC_BARRIER();
    const uint32_t used = h - r->tail;
    const uint32_t n = 2u + len;
    if (r->size - used < n) { r->drops = (uint16_t)(r->drops + 1u); return -1; }
    const uint32_t m = r->size - 1u;
    r->buf[h & m] = type;
    r->buf[(h + 1u) & m] = len;
    const uint8_t *b = (const uint8_t *)p;
    for (uint32_t k = 0; k < len; ++k) r->buf[(h + 2u + k) & m] = b[k];
    SPSC_BARRIER();
    r->head = h + n;
    return 0;
}

uint32_t log_q_get(uint8_t src, uint8_t *type, void *p, uint8_t cap)
{
    if (src >= LOG_SRC_COUNT) return 0;
    ring_t *r = &s_r[src];
    const uint32_t t = r->tail;
    const uint32_t h = r->head;
    SPSC_BARRIER();
    if (h == t) return 0;
    const uint32_t m = r->size - 1u;
    *type = r->buf[t & m];
    const uint8_t len = r->buf[(t + 1u) & m];
    uint8_t *b = (uint8_t *)p;
    for (uint32_t k = 0; k < len; ++k) {
        const uint8_t v = r->buf[(t + 2u + k) & m];
        if (k < cap) b[k] = v;
    }
    SPSC_BARRIER();
    r->tail = t + 2u + len;
    return (uint32_t)(len > cap ? cap : len) + 1u;
}

uint16_t log_q_take_drops(uint8_t src)
{
    if (src >= LOG_SRC_COUNT) return 0;
    ring_t *r = &s_r[src];
    const uint16_t d = r->drops;
    const uint16_t n = (uint16_t)(d - r->drops_seen);
    r->drops_seen = d;
    return n;
}

_Static_assert((LOG_Q_IMU_SIZE & (LOG_Q_IMU_SIZE - 1u)) == 0u, "power of two");
_Static_assert((LOG_Q_BATT_SIZE & (LOG_Q_BATT_SIZE - 1u)) == 0u, "power of two");
