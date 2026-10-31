/**
 * @file  log_queue.h
 * @brief Record queues from the producing tasks to the LogTask.
 *
 * One lock-free single-producer single-consumer byte ring per producing task
 * (ImuTask, BattTask). A record goes in whole or not at all; a full ring
 * counts a drop, reported later in the log as LR_DROP. Producers call
 * log_q_put() only while log_q_active() is 1 (a log is open), so nothing is
 * queued when the log is off.
 *
 * Host-testable.
 */
#ifndef LOG_QUEUE_H
#define LOG_QUEUE_H

#include <stdint.h>

enum { LOG_SRC_IMU = 0, LOG_SRC_BATT = 1, LOG_SRC_COUNT };

#define LOG_Q_IMU_SIZE   4096u    /**< ~1.2 s of ImuTask records: covers a sector erase */
#define LOG_Q_BATT_SIZE  512u

/* ---- producers ---- */
uint8_t log_q_active(void);
int     log_q_put(uint8_t src, uint8_t type, const void *payload, uint8_t len);   /**< 0 = queued */

/* ---- LogTask ---- */
void     log_q_reset(void);
void     log_q_set_active(uint8_t on);
/** next record of 'src': returns payload length + 1, 0 if empty */
uint32_t log_q_get(uint8_t src, uint8_t *type, void *payload, uint8_t cap);
uint16_t log_q_take_drops(uint8_t src);

#endif /* LOG_QUEUE_H */
