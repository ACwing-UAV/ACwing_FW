/**
 * @file  att_ctrl.h
 * @brief Attitude and rate controllers (phase 3): cascaded P on the tilt, PID on the body rates.
 *
 *   ImuTask, 240 Hz  att_ctrl_attitude(): attitude setpoint (ANGLE-type modes) -> body-rate setpoint
 *   ImuTask, 960 Hz  att_ctrl_rate():     body-rate setpoint, gyro -> torque command
 *
 * Frames and signs (vehicle-model.md): body FLU, world NWU, quaternion body->world [w x y z].
 *   roll  + = right wing down (rotation about +x),
 *   pitch + = nose down       (rotation about +y, FLU),
 *   yaw   + = counter-clockwise seen from above (rotation about +z).
 *
 * Tilt control uses the reduced attitude: the error is the rotation that brings the body z axis onto the desired
 * one, so the heading never enters the roll / pitch loops; yaw is a rate command (no heading hold).
 *
 * The torque command is in **normalised thrust per motor**: torque 0.1 on roll means the left motors get +0.1 and
 * the right ones -0.1 of the maximum thrust (see mixer.h). The gains are therefore independent of the battery and
 * of the motor curve, which the mixer compensates (ADR 0010).
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef ATT_CTRL_H
#define ATT_CTRL_H

#include <stdint.h>

/** second-order IIR section (direct form II transposed) */
typedef struct { float b0, b1, b2, a1, a2, z1, z2; } biquad_t;

void  biquad_lowpass(biquad_t *f, float fc_hz, float fs_hz);           /**< Butterworth; fc <= 0: pass-through */
void  biquad_notch(biquad_t *f, float fc_hz, float q, float fs_hz);    /**< fc <= 0: pass-through            */
float biquad_step(biquad_t *f, float x);
void  biquad_reset(biquad_t *f, float x);                              /**< steady state at x                */

typedef struct {
    float rate_p[3], rate_i[3], rate_d[3];   /**< per axis roll, pitch, yaw: torque per rad/s, per rad, per rad/s^2 */
    float att_p;                             /**< 1/s: tilt error (rad) -> rate setpoint (rad/s)                 */
    float rate_max_tilt;                     /**< rad/s: limit of the roll / pitch rate asked by the tilt loop   */
    float i_lim;                             /**< |I| per axis, normalised torque                                */
    float out_lim;                           /**< |torque| per axis                                              */
    float gyro_lpf_hz, dterm_lpf_hz;         /**< 0 = off                                                        */
    float notch_hz, notch_q;                 /**< gyro notch, 0 Hz = off                                         */
    float rate_hz;                           /**< rate-loop frequency (960)                                      */
} att_gains_t;

/** gains of the 1S micro quads (vehicle-model.md, ADR 0010): starting point, to be tuned on the first flights */
void att_gains_default(att_gains_t *g);

/** attitude setpoint from the flight core */
typedef struct {
    uint8_t acro;                            /**< 1: rates from the sticks (ACRO), 0: angles                     */
    float   roll, pitch;                     /**< rad (ANGLE-type)                                               */
    float   roll_rate, pitch_rate, yaw_rate; /**< rad/s, FLU (ACRO roll/pitch; yaw in every mode)                */
} att_sp_t;

typedef struct {
    att_gains_t g;
    biquad_t    gyro_lpf[3], gyro_notch[3], dterm_lpf[3];
    float       rate_sp[3];                  /**< rad/s, from att_ctrl_attitude() or ACRO                        */
    float       i[3];
    float       prev_rate[3];                /**< filtered gyro of the last step (D on the measurement)          */
    uint8_t     have_prev;
    float       gyro_f[3];                   /**< filtered gyro                                                  */
    float       torque[3];                   /**< last output                                                    */
    float       tilt_err;                    /**< rad, last tilt error (log, tests)                              */
} att_ctrl_t;

void att_ctrl_init(att_ctrl_t *c, const att_gains_t *g);
/** new gains (parameters changed): filters rebuilt only if their frequency changed; states kept */
void att_ctrl_set_gains(att_ctrl_t *c, const att_gains_t *g);
/** integrators, D memory and filters reset (disarmed, or on the ground at idle) */
void att_ctrl_reset(att_ctrl_t *c);

/** 240 Hz: q body->world, setpoint -> c->rate_sp (ACRO: the setpoint rates as they are) */
void att_ctrl_attitude(att_ctrl_t *c, const float q[4], const att_sp_t *sp);
/** 960 Hz: gyro rad/s (bias removed) -> torque[3]; integrate = 0 freezes the integrators (on the ground) */
void att_ctrl_rate(att_ctrl_t *c, const float gyro[3], uint8_t integrate, float torque[3]);

#endif /* ATT_CTRL_H */
