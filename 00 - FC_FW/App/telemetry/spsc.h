/**
 * @file  spsc.h
 * @brief Lock-free single-producer single-consumer queue of fixed-size items.
 *
 * One thread pushes, one thread pops; no mutex, no blocking, safe on Cortex-M
 * (32-bit aligned index stores are atomic, barriers order data and index).
 * Capacity N must be a power of 2.
 */
#ifndef SPSC_H
#define SPSC_H

#include <stdint.h>
#include <string.h>

#if defined(__arm__)
#define SPSC_BARRIER() __asm volatile ("dmb" ::: "memory")
#else
#define SPSC_BARRIER() __sync_synchronize()
#endif

typedef struct {
    volatile uint32_t head;      /**< written by the producer only */
    volatile uint32_t tail;      /**< written by the consumer only */
    uint32_t          mask;      /**< N - 1 */
    uint32_t          item;      /**< sizeof item */
    uint8_t          *buf;
} spsc_t;

#define SPSC_DEFINE(name, type, n)                                          \
    static uint8_t name##_buf[(n) * sizeof(type)];                          \
    static spsc_t  name = { 0u, 0u, (n) - 1u, sizeof(type), name##_buf }

static inline void spsc_reset(spsc_t *q) { q->head = q->tail = 0u; }

static inline int spsc_push(spsc_t *q, const void *it)
{
    const uint32_t h = q->head;
    if (h - q->tail > q->mask) return -1;                 /**< full */
    memcpy(&q->buf[(h & q->mask) * q->item], it, q->item);
    SPSC_BARRIER();
    q->head = h + 1u;
    return 0;
}

static inline int spsc_pop(spsc_t *q, void *it)
{
    const uint32_t t = q->tail;
    if (q->head == t) return 0;                           /**< empty */
    SPSC_BARRIER();
    memcpy(it, &q->buf[(t & q->mask) * q->item], q->item);
    SPSC_BARRIER();
    q->tail = t + 1u;
    return 1;
}

static inline uint32_t spsc_count(const spsc_t *q) { return q->head - q->tail; }

#endif /* SPSC_H */
