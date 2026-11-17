/**
 * @file  cal_motors.h
 * @brief MOTORS calibration procedure (HANDOVER_FW §10.7): order and direction check, props off.
 *
 *   CAL_START(7) -> WAITING_USER, step 1. The user taps a motor in the app, which sends MOTOR_TEST(m): the flight
 *   core spins that output (fc_core.c). This procedure only follows it: `motor` / `spinning` while a test runs,
 *   the `ok_mask` bit of every motor that has spun. CAL_SAVE is accepted in any state (the user decides).
 *
 *   Direction: the brushed motors of this board turn one way only (low-side MOSFET, no H-bridge), so
 *   MOTOR_REVERSE cannot be done in software: it is refused with PV2_R_FIXED_DIRECTION and `rev_mask` stays 0.
 *   A motor that spins the wrong way is a wiring or a motor-type fault (CW and CCW 8520 motors differ).
 *
 * Pure logic, no I/O; host-testable.
 */
#ifndef CAL_MOTORS_H
#define CAL_MOTORS_H

#include <stdint.h>

typedef struct {
    uint8_t motor;      /**< motor spinning now (1..n), or the last one that spun; 0 none yet */
    uint8_t spinning;   /**< 1 while a MOTOR_TEST runs */
    uint8_t ok_mask;    /**< bit m-1: motor m has spun during this calibration */
} cm_proc_t;

void    cm_start(cm_proc_t *p);
/** test_motor: the motor the flight core is spinning now (fc_state_t.test_motor), 0 none.
 *  Returns 1 when the CALIB data changed (send a frame). */
uint8_t cm_sample(cm_proc_t *p, uint8_t test_motor);

#endif /* CAL_MOTORS_H */
