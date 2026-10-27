/**
 * @file  ack_cache.h
 * @brief Last (op, seq) -> ACK pairs (HANDOVER_FW.md §5, "Sequence and ACK rules").
 *
 * The app re-sends a command without ACK after 600 ms, twice at most, with
 * the same seq. A repeated (op, seq) must get the cached ACK again and must
 * NOT be executed a second time (a repeated ARM, TAKEOFF or MOTOR_TEST must
 * not happen twice). Entries are PENDING while the flight core has not
 * answered yet: a repeat of a pending command is ignored (its ACK will come).
 *
 * Host-testable.
 */
#ifndef ACK_CACHE_H
#define ACK_CACHE_H

#include <stdint.h>

#define ACK_CACHE_LEN 32u     /* >= 16 required */

enum { ACKC_NEW = 0, ACKC_PENDING = 1, ACKC_DONE = 2 };

void    ackc_reset(void);
/* Looks up (op, seq). ACKC_DONE: result/reason filled, re-send them.
 * ACKC_PENDING: ignore. ACKC_NEW: a pending entry was created, execute it. */
uint8_t ackc_begin(uint8_t op, uint8_t seq, uint8_t *result, uint8_t *reason);
/* Completes the entry (or creates it if it was evicted meanwhile). */
void    ackc_finish(uint8_t op, uint8_t seq, uint8_t result, uint8_t reason);

#endif /* ACK_CACHE_H */
