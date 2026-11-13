/**
 * @file  mixer.h
 * @brief Quad-X mixer: collective + torques in normalised thrust -> motor duty (phase 3, ADR 0010 §1).
 *
 * Motor numbering as the app and Betaflight (HANDOVER_FW §10.7):
 *   M1 rear right CW, M2 front right CCW, M3 rear left CCW, M4 front left CW.
 *
 *              roll   pitch  yaw          roll + : right wing down -> left motors up
 *     M1 RR    -1     +1     +1           pitch +: nose down       -> rear motors up
 *     M2 FR    -1     -1     -1           yaw   +: counter-clockwise -> the CW propellers (M1, M4) up
 *     M3 RL    +1     +1     -1                   (their reaction torque turns the frame CCW)
 *     M4 FL    +1     -1     +1
 *
 * Per motor: t_i = T + roll*R_i + pitch*P_i + yaw*Y_i, in normalised thrust (thrust / maximum thrust at
 * V_ref = 4.2 V per cell). Saturation keeps the attitude first: roll/pitch are scaled only if they alone do not
 * fit, then yaw is scaled to the room left, then the collective is moved (down at full throttle, up at low
 * throttle: "airmode") so that every motor stays in [idle, 1]. Then each t_i goes through the inverse thrust curve
 * f(u') = theta u'^2 + (1 - theta) u' and the battery compensation V_ref / V_cell, and is clipped to 0..1.
 *
 * Host-testable.
 */
#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

#define MIXER_MOTORS 4u
#define MIXER_VREF   4.2f       /**< V per cell: the reference of the normalised thrust */

typedef struct {
    float idle;                 /**< normalised thrust of a motor at idle (armed)                    */
    float theta;                /**< thrust curve: 1 quadratic (default), 0 linear (param thrCurve)  */
} mixer_cfg_t;

typedef struct {
    float   t[MIXER_MOTORS];    /**< normalised thrust per motor after the saturation               */
    float   duty[MIXER_MOTORS]; /**< PWM duty 0..1                                                  */
    float   collective;         /**< collective after the saturation                                */
    float   batt_k;             /**< battery compensation applied (V_ref / V_cell, 1 if unknown)     */
    uint8_t sat_rp, sat_yaw, sat_col; /**< roll/pitch scaled, yaw scaled, collective moved          */
} mixer_out_t;

/** inverse thrust curve: normalised thrust t (0..1) -> duty at V_ref */
float mixer_curve_inv(float t, float theta);

/** collective T and torques (roll, pitch, yaw) -> out; v_cell = 0: battery not known, no compensation */
void mixer_quadx(float collective, const float torque[3], float v_cell, const mixer_cfg_t *cfg, mixer_out_t *out);

#endif /* MIXER_H */
