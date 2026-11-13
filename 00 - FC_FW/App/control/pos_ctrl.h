/**
 * @file  pos_ctrl.h
 * @brief Horizontal position hold (POS_HOLD, and the automatic actions with a position source): sticks or the held
 *        point -> roll / pitch setpoints for the tilt loop.
 *
 *   position   p  (world NW plane, m): dead reckoning of the measured velocity, p += Rz(yaw) v_b dt
 *   stick out of the deadband: v_sp = stick * v_max (heading frame); the held point follows the drone
 *   stick released:           brake (v_sp = 0) until |v| < PC_LATCH_V or PC_BRAKE_MAX_S, then hold that point
 *   holding:                  v_sp = clamp(kp_pos * (p_sp - p), PC_HOLD_VMAX)
 *   velocity loop (world frame, the wind is fixed there): a = kp_vel (v_sp - v) + I, then turned to the heading
 *   frame (x forward, y left); the tilt atan(|a| / g) <= tilt_max, split along the direction of a
 *   tilt:  pitch = tilt a_x / |a|  (+ nose down, forward),   roll = -tilt a_y / |a|  (+ right wing down, -y)
 *
 * The velocity comes from the optical flow (app_main.c: flow rate minus the body rotation, times the ToF height) in
 * body axes; for the tilts of a hold (< 30 deg) it is taken as the velocity in the heading frame. The position is
 * relative and drifts slowly with the flow scale error: it is only used to come back to a point held a few seconds
 * or minutes ago. Design (control-loops.md): velocity loop crossover kp_vel 1.8 rad/s, a third of the tilt loop
 * (6 rad/s), PI zero at 0.5 rad/s; position loop kp_pos 0.8 1/s.
 *
 * Pure logic, flight-control context. Host-testable.
 */
#ifndef POS_CTRL_H
#define POS_CTRL_H

#include <stdint.h>

#define PC_DEADBAND     0.05f   /**< stick fraction: inside, the drone holds            */
#define PC_LATCH_V      0.2f    /**< m/s: braked enough to hold the point               */
#define PC_BRAKE_MAX_S  1.5f    /**< s: hold the point anyway after this long braking    */
#define PC_HOLD_VMAX    1.0f    /**< m/s: speed back to the held point                   */

typedef struct {
    float kp_pos;               /**< 1/s, position error -> velocity                     */
    float kp_vel, ki_vel;       /**< 1/s, 1/s^2: velocity error -> acceleration           */
    float i_lim;                /**< m/s^2: |I| (wind, trim)                              */
} pc_gains_t;

enum { PC_MOVE = 0, PC_BRAKE = 1, PC_HOLD = 2 };

typedef struct {
    pc_gains_t g;
    float   p[2];               /**< world NW plane, m, since the last reset            */
    float   p_sp[2];            /**< held point                                          */
    float   i[2];               /**< world frame, m/s^2 (wind, trim)                     */
    float   v_sp[2];            /**< heading frame, m/s (log, tests)                     */
    float   brake_s;
    uint8_t state;              /**< PC_MOVE / PC_BRAKE / PC_HOLD                        */
} pos_ctrl_t;

void pc_gains_default(pc_gains_t *g);
void pc_init(pos_ctrl_t *c, const pc_gains_t *g);
/** not holding (flag off): forget the point and the integrator; the next step starts braking */
void pc_reset(pos_ctrl_t *c);

typedef struct {
    float vel_b[2];             /**< m/s, body x forward, y left (flow)                   */
    float yaw;                  /**< rad, heading of the attitude (NWU, + CCW)            */
    float stick_fwd, stick_left;/**< -1..1 (pitch stick + forward, roll stick + right -> stick_left = -roll) */
    float v_max;                /**< m/s at full stick                                    */
    float tilt_max;             /**< rad                                                  */
    float dt;                   /**< s                                                    */
} pc_in_t;

/** one step: roll / pitch setpoints (rad, FLU) for the tilt loop */
void pc_step(pos_ctrl_t *c, const pc_in_t *in, float *roll, float *pitch);

#endif /* POS_CTRL_H */
