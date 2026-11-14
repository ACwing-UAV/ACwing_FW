/**
 * @file  control_hooks.h
 * @brief Control hook points, already called by the estimation task at the
 *        right rates. In phase 1 they are empty ("weak" definitions in
 *        control_hooks.c): to add control, just redefine them, without
 *        touching the task or the estimator.
 *
 * Order within a 960 Hz step (see app_main.c):
 *   IEKF propagation -> ctrl_rate_step() -> [every 4 steps] acc update ->
 *   ctrl_attitude_step() -> [when available] mag update
 * ctrl_rate_step() runs right after the propagation, before the corrections,
 * to minimise the gyro -> motors latency.
 */
#ifndef CONTROL_HOOKS_H
#define CONTROL_HOOKS_H

#include <stdint.h>

typedef struct {
    float    gyro[3];      /**< rad/s: mean of the step samples, estimated bias removed */
    float    dt;           /**< s */
    uint32_t step;
} ctrl_rate_in_t;

typedef struct {
    float    q[4];         /**< IEKF attitude, body->world NWU, [w x y z] */
    float    dt;
    uint32_t step;
} ctrl_att_in_t;

void ctrl_init(void);
void ctrl_rate_step(const ctrl_rate_in_t *in);      /**< 960 Hz */
void ctrl_attitude_step(const ctrl_att_in_t *in);   /**< 240 Hz */

/* phase 3 (app/ctrl_loop.c): the loops, the mixer and the motors */
/** LEVEL calibration trim (deg), from the ImuTask when a calibration is applied */
void    ctrl_set_level_trim(float roll_deg, float pitch_deg);
/** motor output 0..100 % for the MOTORS frame (0 while disarmed); any task (byte reads) */
uint8_t ctrl_motor_pct(uint8_t motor_index);
/** ImuTask, ~250 Hz: armed and no rate step for CTRL_STALL_MS (the IMU or the estimator stopped) -> motors off.
 *  Returns 1 while stalled. A motor command frozen at its last value would fly the drone away. */
#define CTRL_STALL_MS 50u
uint8_t ctrl_service(uint32_t now_ms);

#endif /* CONTROL_HOOKS_H */
