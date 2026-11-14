/**
 * @file  fc_core.h
 * @brief Flight core: arming, modes, automatic actions, failsafes, motor test
 *        (HANDOVER_FW.md §6, §7, §10.7).
 *
 * Runs ONLY in the flight-control context: fc_service() is called by the
 * ImuTask, the loop that runs the estimator and the attitude controller
 * (>= 50 Hz, about 250 Hz here). The flight state has a single owner and no
 * lock: the AppTask talks to it through lock-free single-producer
 * single-consumer queues
 *     AppTask -> core   commands      (fc_post)
 *     core -> AppTask   ACK results   (fc_take_result)
 *     core -> AppTask   events        (fc_take_event)
 *     core -> AppTask   state         (fc_snapshot, sequence counter)
 * and reads the app link only through app_link.h (an atomic timestamp and the
 * RC seqlock). Nothing here waits on BLE: if the BLE stack, the CPU2, the
 * radio or the AppTask stall, the app-link silence grows and the failsafe
 * fires on time.
 *
 * Host-testable: no HAL, no RTOS (time is passed in, outputs via hooks).
 */
#ifndef FC_CORE_H
#define FC_CORE_H

#include <stdint.h>

/* ---- timing of the app-link supervision (§6) ---- */
#define FC_RC_STALE_MS       250u    /**< stop using the last RC values          */
#define FC_FS_SHORT_MS       1000u   /**< fsAppShort                             */
#define FC_FS_LONG_MS        3000u   /**< fsAppLong + EVENT 20                   */
#define FC_LINK_BACK_MS      500u    /**< link considered back                   */
#define FC_GROUND_DISARM_MS  3000u   /**< armed on the ground, app silent        */
#define FC_ARM_RC_FRESH_MS   500u    /**< throttle rule applies to RC this recent */
#define FC_NO_TAKEOFF_MS     10000u  /**< auto-disarm if not taken off           */
#define FC_STICK_CANCEL      300     /**< 30 % of the stick                       */
/* ---- automatic flight (§7.2, §7.3) ---- */
#define FC_TAKEOFF_VZ        0.6f    /**< m/s, climb during TAKEOFF               */
#define FC_LIFTOFF_DH        0.15f   /**< m above the ground level = airborne      */
#define FC_LIFTOFF_TIMEOUT_MS 6000u  /**< no lift-off -> TAKEOFF aborted           */
#define FC_LAND_SLOW_VZ      0.3f    /**< m/s below 1 m with ToF                   */
#define FC_TOUCH_VZ          0.12f   /**< |vz| below this ...                      */
#define FC_TOUCH_MS          700u    /**< ... for this long (descent commanded)    */
#define FC_TOUCH_MS_NO_TOF   1000u
#define FC_FLOW_LOST_Q       30u     /**< flow quality below this ...              */
#define FC_FLOW_BACK_Q       40u
#define FC_FLOW_LOST_MS      500u
#define FC_FLOW_CHECK_H      0.15f   /* m: FLOW_LOST is judged only above this height,
                                        never on the ground, in TAKEOFF or LAND */

/* ---- hover thrust (ADR 0010 §2-3): seed = parameter hoverThr, then hover_est in flight ---- */
#define FC_HOVER_SPOOL_MS    500u    /**< no update this long after the lift-off    */
#define FC_HOVER_MIN_TOF_H   0.15f   /**< m: ground effect below (2 prop diameters) */
#define FC_THR_LOW           (-0.9f) /**< stick "at the bottom"                     */
/* ---- in air / on ground without an altitude estimate (§6) ---- */
#define FC_AIR_THR_K         0.8f    /**< collective > 0.8 x hover ...              */
#define FC_AIR_MS            300u    /**< ... this long: airborne                   */
#define FC_LAND_OL_K         0.85f   /**< open-loop LAND: 0.85 x hover ...          */
#define FC_LAND_OL_RAMP_MS   1000u   /**< ... reached in 1 s ...                    */
#define FC_LAND_OL_MS        20000u  /**< ... for at most 20 s, then touchdown      */
#define FC_GF_BRAKE_A        2.5f    /**< m/s^2: manual modes slow down below the ceiling */
#define FC_INDOOR_HSPEED     2.0f    /**< m/s: maxHSpeed cap of the indoor models (HANDOVER_FW §8) */

/* ---- battery (§6): signals from the BattTask ---- */
#define FC_BATT_STALE_MS     1000u   /**< no BattTask heartbeat: battery unknown   */

/* internal opcodes (AppTask -> core), outside the v2 range */
#define FC_OP_MOTOR_STOP     0xF0u   /**< stop a running motor test               */
typedef struct {
    uint8_t op, seq, len;            /**< len = payload bytes after seq */
    uint8_t p[18];
} fc_cmd_t;

typedef struct { uint8_t op, seq, result, reason; } fc_res_t;
typedef struct { uint8_t level; uint16_t code; int32_t arg; } fc_event_t;

/** battery signal (BattTask -> core): sent when the level or presence changes */
typedef struct {
    uint8_t  present;                /**< battery connected                       */
    uint8_t  level;                  /**< PV2_BATT_OK … PV2_BATT_EMERGENCY         */
    uint16_t cell_mv;                /**< sagged cell voltage (EVENT arg)          */
} fc_batt_sig_t;

typedef struct {
    uint16_t sensors;                /**< CAPS bits detected at boot             */
    uint8_t  n_motors;               /**< 4, 6 or 8                              */
    uint8_t  motors_built;           /**< 0: MOTOR_TEST -> UNSUPPORTED           */
    /** outputs (ImuTask context); may be NULL on the host */
    void   (*motors_arm)(uint8_t armed);
    void   (*motor_test)(uint8_t motor, float duty);   /**< motor 0 = all off */
    /** every event also goes here (flight-control context), e.g. to the log; may be NULL */
    void   (*on_event)(const fc_event_t *e);
    uint8_t  batt_monitor;           /**< a BattTask measures the battery          */
    /** 1 while a USB host is connected: ARM, TAKEOFF and MOTOR_TEST are refused
     *  (USB_CONNECTED, ADR 0009); may be NULL (no USB) */
    uint8_t (*usb_host)(void);
} fc_config_t;

/** published state (AppTask builds FLIGHT / MOTORS from it) */
typedef struct {
    uint8_t  armed, mode, autom, failsafe, input, in_air;
    uint8_t  rc_stale;               /**< RC older than FC_RC_STALE_MS: neutral sticks in use */
    uint8_t  fs_long;                /**< the long app-link action fired           */
    uint16_t flight_time_s;
    uint8_t  motor_out_pct[8];
    uint8_t  test_motor;             /**< motor under test (1..n), 0 none         */
    uint8_t  landed;                 /**< touched down after a flight (until disarm) */
    uint8_t  geofence;               /**< a limit is active                        */
    uint8_t  batt_level;             /**< last level signalled by the BattTask     */
    uint8_t  batt_stale;             /**< BattTask silent: battery state unknown   */
    int32_t  alt_rel_cm;             /**< above the take-off point                  */
    int16_t  vz_cms;
    uint32_t silence_ms;             /**< app link silence seen by the failsafe    */
    uint8_t  land_idle;              /**< touched down: idle until disarm or stick at the bottom */
    float    hover_sd;               /**< 1-sigma of the hover estimate             */
    float    hover_nis;              /**< last NIS of the hover estimate            */
    uint8_t  air_by_thrust;          /**< in air detected from the collective (no altitude estimate) */
    float    hover;                  /**< learned hover collective 0..1            */
    float    stick[4];               /**< setpoint inputs -1..1: thr, yaw, pitch, roll */
} fc_state_t;

/** vertical state from the estimator (flight-control context) */
typedef struct {
    uint8_t valid;                   /**< an altitude source is running (baro and/or ToF) */
    float   h, vz;                   /**< m, m/s, + up (alt_est)                  */
    uint8_t tof_valid, tof_ground;   /**< ToF used / ToF below its minimum range  */
    float   tof_h;                   /**< m above the floor                        */
    float   cos_tilt;                /**< R33 of the attitude                      */
    uint8_t flow_q;                  /**< 0..100, 0xFF n/a                         */
    uint8_t flow_usable;             /* flow valid: above the PMW3901 minimum
                                        height (80 mm; landed it is ~11 mm)      */
    float   a_up;                    /**< m/s^2, + up, gravity and accelerometer bias removed, ~10 Hz low-pass */
    float   vel_b[2];                /**< m/s, horizontal velocity from the flow, body x forward, y left (valid with flow_usable) */
    float   yaw;                     /**< rad, heading of the attitude (NWU, + counter-clockwise) */
    /* hand launch (ADR 0011) */
    float   f_g;                     /**< specific force magnitude of the last acc window, g      */
    float   a_w[3];                  /**< world acceleration, gravity removed, m/s^2 (NWU)        */
    float   w;                       /**< |body rate|, rad/s                                      */
    uint8_t gyro_sat;                /**< a gyro axis saturated since the last call               */
    float   v_cell;                  /**< sagged cell voltage, V (0 = unknown)                    */
    float   w_b[3];                  /**< body rates, rad/s, bias removed (flip, ADR 0012)        */
} fc_vert_t;

/** setpoints for the attitude / rate loops (phase 3) and the motor mixer */
typedef struct {
    uint8_t armed, mode, autom;
    uint8_t alt_ctrl;                /**< 1: thrust from the altitude loop (h_sp, vz_sp valid)            */
    float   thrust;                  /**< collective 0..1                          */
    float   roll, pitch;             /**< rad (ANGLE-type modes)                   */
    float   roll_rate, pitch_rate;   /**< rad/s (ACRO), FLU: + = right wing down / nose down */
    float   yaw_rate;                /**< rad/s, FLU: + = counter-clockwise from above (stick right = negative) */
    float   h_sp, vz_sp;             /**< m, m/s (when alt_ctrl)                   */
    uint8_t in_air;                  /**< airborne: the rate loop may integrate      */
    uint8_t motors_off;              /**< armed with the motors stopped (hand launch, ADR 0011): duty 0 */
    uint8_t pos_hold;                /* 1: roll / pitch from the horizontal position loop (pos_ctrl.c):
                                        POS_HOLD, or an automatic action (TAKEOFF, LAND, RTH, HOLD,
                                        failsafe), with a usable flow and the altitude loop on
                                        (pos_hold implies alt_ctrl). 0: roll / pitch from the sticks
                                        (centred = level), e.g. near the floor below the flow range */
} fc_setpoint_t;

void    fc_init(const fc_config_t *cfg);
/** sensors detected at run time (a late expansion board): atomic store */
void    fc_set_sensors(uint16_t sensors);

/* ---- flight-control context ---- */
/** v: NULL = no vertical estimate (in air only via g_fc_dbg_in_air) */
void    fc_service(uint32_t now_ms, uint8_t est_running, const fc_vert_t *v);
void    fc_setpoint(fc_setpoint_t *out);         /**< last setpoints, same context */
/** 1 during a flip: the estimator must skip its accelerometer and magnetometer updates (same context) */
uint8_t fc_vector_hold(void);

/* ---- BattTask ---- */
int     fc_batt_signal(const fc_batt_sig_t *s);   /**< 0 = queued, -1 = full   */
void    fc_batt_heartbeat(uint32_t now_ms);        /**< every BattTask step      */

/* ---- AppTask ---- */
int     fc_post(const fc_cmd_t *c);              /**< 0 = queued, -1 = full   */
int     fc_take_result(fc_res_t *r);              /**< 1 if a result was taken */
int     fc_take_event(fc_event_t *e);
void    fc_snapshot(fc_state_t *out);
/** ground information computed by the AppTask (atomic byte stores) */
void    fc_set_ground_info(uint8_t checks_ready, uint8_t battery_fail, uint8_t calibrating);
uint8_t fc_is_core_op(uint8_t op);                /**< ops executed by the core */

/* ---- bench test hooks (debugger Live Expressions) ----
 * g_fc_dbg_in_air: 1 while armed forces "in the air" (props off!), so that
 * the in-air failsafes can be tested on the bench without altitude sensors
 * (HANDOVER §6 test, §16.12). Clearing it simulates the touchdown. With an
 * altitude source, in air / on ground is detected from the vertical estimate
 * and the board can be "flown" by hand (lift it: lift-off; put it down during
 * LAND: touchdown). */
extern volatile uint8_t g_fc_dbg_in_air;

#endif /* FC_CORE_H */
