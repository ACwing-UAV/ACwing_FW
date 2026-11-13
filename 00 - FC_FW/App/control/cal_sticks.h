/**
 * @file  cal_sticks.h
 * @brief STICKS calibration procedure (HANDOVER_FW §10.8): centres, then the end points, then the AUX switches.
 *
 *   step 1 (WAITING_USER): sticks at rest; the live values are shown; capture stores the centres
 *   step 2 (RUNNING):      the pilot moves every stick to its end points and every switch through its positions;
 *                          min/max are tracked, AUX positions counted (low < 1300 us, mid, high > 1700 us)
 *   capture -> DONE if every stick moved at least 150 us on each side of its centre (throttle: its range)
 *
 * Order of the sticks: throttle, yaw, pitch, roll (rc_state_t.stick_us). The throttle "centre" is its rest
 * position (bottom); only its min/max are used. Pure logic, no I/O; host-testable.
 */
#ifndef CAL_STICKS_H
#define CAL_STICKS_H

#include <stdint.h>

#define CS_MIN_SPAN 150u            /**< us each side of the centre (throttle: 300 us total) */

typedef struct { uint16_t centre[4], min[4], max[4]; uint8_t aux_pos[4]; } cal_sticks_t;   /**< 28 bytes */

typedef struct {
    uint8_t      step;              /**< 1 centres, 2 ranges, 3 done */
    uint16_t     live[4];
    cal_sticks_t r;
    uint8_t      aux_seen[4];       /**< bit 0 low, 1 mid, 2 high */
} cs_proc_t;

void    cs_start(cs_proc_t *p);
void    cs_sample(cs_proc_t *p, const uint16_t stick_us[4], const uint16_t aux_us[4]);
/** step 1 -> 2 (centres stored): 1, 0 without a valid reading; step 2 -> 3 (result valid): 1, too little travel: 0 */
uint8_t cs_capture(cs_proc_t *p);
/** 0..100: share of the eight stick ends reached (step 2) */
uint8_t cs_progress(const cs_proc_t *p);
uint8_t cs_valid(const cal_sticks_t *c);

#endif /* CAL_STICKS_H */
