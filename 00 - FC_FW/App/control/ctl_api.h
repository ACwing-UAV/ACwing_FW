/**
 * @file  ctl_api.h
 * @brief The interface of a control law (ADR 0013): what a law reads, what it commands, and the hooks it implements.
 *
 * Three layers, each knowing only the one below through this header:
 *
 *   flight manager (fc_core.c)   decides WHAT: modes, automatic actions, failsafes, geofence, input arbitration.
 *                                Publishes a reference (ctl_ref_t): per axis group, a kind and a value.
 *   control law (ctl_law_t)      decides HOW: from the reference and the vehicle state (ctl_get_*) to a command
 *                                (ctl_set_*). Swappable: registered with ctl_register(), chosen by the parameter
 *                                ctlLaw while disarmed.
 *   actuators (app/ctrl_loop.c)  allocation (quad-X mixer), thrust curve, battery compensation, motor outputs and
 *                                the safety rules no law can bypass: no output while disarmed, duty 0 with
 *                                motors_off, the stall watchdog, the motor test.
 *
 * A law implements four functions, all called in the flight-control context (ImuTask), never concurrently:
 *   outer(dt)     ~250 Hz, after the flight manager has published the reference: vertical and horizontal loops
 *   attitude(dt)  240 Hz, after each attitude update of the estimator
 *   rate(dt)      960 Hz, right after the gyro propagation: must call ctl_set_thrust_torque() or
 *                 ctl_set_motor_thrust() once (a rate step without a command writes duty 0)
 *   reset()       on disarm and when the law becomes active
 *
 * Units: SI, body FLU, world NWU (vehicle-model.md). Thrust and torques in normalised thrust per motor (fraction of
 * the maximum thrust of one motor at 4.2 V per cell, ADR 0010): the actuator layer removes the battery and the
 * thrust curve, so a law need not know them.
 */
#ifndef CTL_API_H
#define CTL_API_H

#include <stdint.h>

/* ------------------------------------------------------------------ reference (flight manager -> law) */
/** attitude group: roll/pitch as angles (yaw always a rate), or the three body rates */
typedef enum { CTL_ATT_ANGLE = 0, CTL_ATT_RATE = 1 } ctl_att_kind_t;
/** vertical group */
typedef enum {
    CTL_V_COLLECTIVE = 0,  /**< open loop: ref.collective; with vz_ceiling < CTL_NO_LIMIT the law limits it so the
                                climb rate stays below vz_ceiling (geofence braking curve)                       */
    CTL_V_CLIMB      = 1,  /**< climb rate vz_sp                                                                 */
    CTL_V_HEIGHT     = 2,  /**< height h_sp, climbing at most vz_up, descending at most vz_dn                    */
} ctl_vert_kind_t;
/** horizontal group */
typedef enum {
    CTL_H_NONE     = 0,    /**< roll / pitch from the attitude group                                            */
    CTL_H_VELOCITY = 1,    /**< velocity vel_sp in the heading frame; zero = hold the point where it stopped     */
} ctl_horiz_kind_t;

#define CTL_NO_LIMIT 1e9f

typedef struct {
    uint8_t armed;          /**< 0: the law must command nothing (the actuators ignore it anyway)              */
    uint8_t in_air;         /**< airborne: integrators may run                                                  */
    uint8_t motors_off;     /**< armed with the motors stopped (hand launch): the actuators write duty 0       */

    uint8_t att;            /**< ctl_att_kind_t                                                                 */
    float   roll, pitch;    /**< rad, CTL_ATT_ANGLE (FLU: + right wing down, + nose down)                       */
    float   rate[3];        /**< rad/s, body: roll, pitch (CTL_ATT_RATE), yaw (always)                          */

    uint8_t vert;           /**< ctl_vert_kind_t                                                                */
    float   collective;     /**< 0..1, CTL_V_COLLECTIVE                                                         */
    float   vz_sp;          /**< m/s, CTL_V_CLIMB                                                               */
    float   h_sp;           /**< m, CTL_V_HEIGHT (the frame of ctl_get_height())                                */
    float   vz_up, vz_dn;   /**< m/s, limits of CTL_V_HEIGHT (both > 0)                                         */
    float   vz_ceiling;     /**< m/s: geofence. Closed loop: a positive climb rate is cut to it. Collective:
                                 the climb-rate limit above. CTL_NO_LIMIT = none                                */
    float   thrust_max;     /**< 0..1: upper limit of the collective (take-off spool-up ramp), 1 = none        */

    uint8_t horiz;          /**< ctl_horiz_kind_t                                                               */
    float   vel_sp[2];      /**< m/s, heading frame (x forward, y left), CTL_H_VELOCITY                         */
    float   v_max;          /**< m/s: largest horizontal speed                                                  */
    float   tilt_max;       /**< rad: largest tilt the law may ask (every mode)                                 */
} ctl_ref_t;

/* ------------------------------------------------------------------ vehicle state (getters) */
void    ctl_get_ref(ctl_ref_t *ref);
void    ctl_get_attitude(float q[4]);       /**< body -> world, [w x y z]                                       */
void    ctl_get_rates(float w[3]);          /**< rad/s, body, gyro bias removed (latest rate step)              */
float   ctl_get_yaw(void);                  /**< rad, NWU, + counter-clockwise                                  */
float   ctl_get_cos_tilt(void);             /**< R33                                                            */
uint8_t ctl_height_valid(void);             /**< an altitude estimate is running                               */
float   ctl_get_height(void);               /**< m, + up                                                        */
float   ctl_get_vz(void);                   /**< m/s, + up                                                      */
float   ctl_get_accel_up(void);             /**< m/s^2, world vertical, gravity and bias removed (10 Hz LPF)    */
uint8_t ctl_vel_valid(void);                /**< the horizontal velocity is measured (optical flow usable)      */
void    ctl_get_vel_body(float v[2]);       /**< m/s, body x forward, y left                                    */
float   ctl_get_hover_thrust(void);         /**< normalised collective that holds a hover (estimate, ADR 0010)  */
float   ctl_get_cell_voltage(void);         /**< V per cell under load, 0 = unknown                             */
void    ctl_get_level_trim(float trim[2]);  /**< rad: roll, pitch of the LEVEL calibration, to subtract         */
float   ctl_get_param(uint16_t id);         /**< any parameter by its id (HANDOVER_FW §8); NaN if unknown       */

/* ------------------------------------------------------------------ commands (setters) */
/** collective 0..1 and torques (roll, pitch, yaw) in normalised thrust per motor: the actuator layer allocates
 *  them with the quad-X mixer (roll/pitch first, then yaw, airmode) */
void    ctl_set_thrust_torque(float collective, const float torque[3]);
/** the thrust of each motor 0..1 (M1 rear right, M2 front right, M3 rear left, M4 front left): for a law with its
 *  own allocation; the actuator layer still applies the thrust curve and the battery compensation */
void    ctl_set_motor_thrust(const float thrust[4]);

/** what the law reports back to the flight manager, the log and the telemetry (outer step) */
typedef struct {
    float   collective;     /**< collective commanded                                                           */
    float   roll, pitch;    /**< rad, attitude targets in use (from the reference or from the horizontal loop) */
    float   vz_sp;          /**< m/s, climb rate asked by the vertical loop (0 in open loop)                    */
    uint8_t saturated;      /**< the collective is at a limit of the law (no hover-thrust learning there)      */
} ctl_status_t;
void    ctl_set_status(const ctl_status_t *st);
void    ctl_get_status(ctl_status_t *st);

/* ------------------------------------------------------------------ the law */
typedef struct {
    const char *name;
    void (*init)(void);
    void (*reset)(void);
    void (*outer)(float dt);
    void (*attitude)(float dt);
    void (*rate)(float dt);
} ctl_law_t;

#define CTL_LAWS_MAX 4u
/** adds a law; its index (0, 1, ...) is the value of the parameter ctlLaw that selects it. -1 if full.
 *  The default law (cascade PID, ctl_law_cascade.c) is index 0. */
int     ctl_register(const ctl_law_t *law);
/** Define this function in your own source file (it is weak here) and call ctl_register() in it for each of your
 *  laws: it runs at every flight-core init, after the default law is registered and before ctlLaw is applied.
 *  Laws are registered in this order: the first one is index 1. */
void    ctl_user_laws(void);
/** the law running now (index), its name */
int     ctl_active(void);
const char *ctl_active_name(void);

#endif /* CTL_API_H */
