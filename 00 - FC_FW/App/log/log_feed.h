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

void logf_att(uint32_t t, const float q[4], const float gyro[3], const float acc_g[3]);
void logf_vert(uint32_t t, const fc_vert_t *v, const fc_setpoint_t *sp, uint16_t tof_mm, uint32_t baro_pa);
void logf_flight(uint32_t t, const fc_state_t *s);
void logf_est(uint32_t t, const float gyro_bias[3], float att_unc_deg, float vib_mg,
              float loop_hz, float jitter_us, float cpu_pct, float mag_interf_pct);
void logf_event(const fc_event_t *e);           /**< fc_config_t.on_event */
void logf_power(uint32_t t, const batt_state_t *b);

#endif /* LOG_FEED_H */
