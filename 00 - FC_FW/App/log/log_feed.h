/**
 * @file  log_feed.h
 * @brief Producers' side of the flight log: build a record from the values
 *        the task already has and queue it (log/log_queue.h).
 *
 * Every function returns at once when no log is open (log_q_active() = 0).
 * ImuTask: logf_att, logf_vert, logf_flight, logf_est, logf_event (via the
 * flight core's on_event hook). BattTask: logf_power. Host-testable.
 */
#ifndef LOG_FEED_H
#define LOG_FEED_H

#include <stdint.h>
#include "fc_core.h"
#include "battery.h"

/** clock for the event records (ms) */
void logf_set_clock(uint32_t (*now_ms)(void));

/** Rate divider of the records: 1 when `period_ms` has elapsed since the last record. The next time advances by
 * exactly one period (not to `now`), so the average rate is 1000 / period_ms even when the caller runs on a period
 * that does not divide it (the ImuTask: ~4.4 ms at the real 916 Hz of the IMU, which gave ~46 Hz instead of 50).
 * After a gap of more than one period (log paused, first call) it starts again from `now`. */
uint8_t logf_due(uint32_t *next_ms, uint32_t now_ms, uint32_t period_ms);

void logf_att(uint32_t t, const float q[4], const float gyro[3], const float acc_g[3]);
void logf_vert(uint32_t t, const fc_vert_t *v, const fc_setpoint_t *sp, uint16_t tof_mm, uint32_t baro_pa);
void logf_flight(uint32_t t, const fc_state_t *s);
void logf_est(uint32_t t, const float gyro_bias[3], float att_unc_deg, float vib_mg,
              float loop_hz, float jitter_us, float cpu_pct, float mag_interf_pct);
void logf_event(const fc_event_t *e);           /**< fc_config_t.on_event */
void logf_power(uint32_t t, const batt_state_t *b);

#endif /* LOG_FEED_H */
