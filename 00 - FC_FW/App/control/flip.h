/**
 * @file  flip.h
 * @brief Autonomous flip (ADR 0012): the phases of one full rotation about the roll or the pitch axis.
 *
 *   CLIMB    collective 1.0, attitude level, FL_CLIMB_MS                                   (about +1 m/s)
 *   ROTATE   body rate flipRate on the axis, other axes 0, collective FL_ROT_COLLECTIVE (the airmode raises it as
 *            the torque needs); the angle is the gyro integrated on the axis. Ends at 360 deg minus the braking
 *            lead w^2 / (2 FL_ALPHA) minus FL_ROT_MARGIN; or at once after FL_ROT_MAX_MS, a stick moved, the link
 *            lost, the gyro saturated (-> RECOVER, reported as aborted)
 *   RECOVER  attitude level, collective: all the thrust while falling, the hover thrust / cos(tilt) otherwise,
 *            the hover thrust beyond 60 deg of tilt (the attitude first). Ends level and slow (tilt < 5 deg,
 *            |w| < 1 rad/s, after FL_REC_MIN_MS) -> DONE, or after FL_REC_MAX_MS -> ABORT (the caller lands)
 *   DONE     the caller restores the previous mode and the starting height
 *
 * Entry gate in fl_gate(): everything the ADR asks before anything moves. Pure logic, flight-control context
 * (fc_core, 250 Hz). Host-testable.
 */
#ifndef FLIP_H
#define FLIP_H

#include <stdint.h>

#define FL_CLIMB_MS         180u
#define FL_ROT_COLLECTIVE   0.25f
#define FL_ALPHA            500.0f   /**< rad/s^2: braking acceleration assumed for the lead (0.5 x 1100 rad/s^2) */
#define FL_ROT_MARGIN       0.1745f  /**< rad: the recovery takes the last 10 deg                               */
#define FL_ROT_MAX_MS       600u
#define FL_REC_MIN_MS       100u
#define FL_REC_MAX_MS       1000u
#define FL_CEILING_ROOM_M   0.6f
#define FL_STICK_ABORT      0.5f     /**< stick fraction that aborts the rotation                              */

enum { FL_IDLE = 0, FL_CLIMB = 1, FL_ROTATE = 2, FL_RECOVER = 3, FL_DONE = 4, FL_ABORT = 5 };
/** axes of the FLIP command */
enum { FL_ROLL_RIGHT = 0, FL_ROLL_LEFT = 1, FL_PITCH_FWD = 2, FL_PITCH_BACK = 3 };
/** reasons (EVENT 14 arg low byte; the phase is the high byte) */
enum { FL_AB_NONE = 0, FL_AB_TIMEOUT = 1, FL_AB_STICK = 2, FL_AB_LINK = 3, FL_AB_GYRO_SAT = 4, FL_AB_NOT_LEVEL = 5,
       FL_AB_ESTIMATE = 6 };

/** what the gate needs; the flight core fills it */
typedef struct {
    uint8_t in_air, armed, auto_none, failsafe, beginner, acro;
    uint8_t alt_valid;          /**< an altitude estimate                                              */
    float   height;             /**< m above the floor (ToF) or above the take-off point (baro)        */
    float   ceiling_room;       /**< m to the ceiling / geofence                                       */
    float   tw;                 /**< thrust-to-weight available now (ln_tw_available)                  */
    float   cos_tilt, w;        /**< |body rate| rad/s                                                 */
    float   v_h;                /**< m/s horizontal speed (0 if not measured)                          */
    float   vz;
    uint8_t batt_ok, link_ok, gyro_sat;
    float   min_h, min_tw;      /**< parameters flipMinH, flipMinTw                                    */
} fl_gate_in_t;

#define FL_REFUSED 0xFFu      /**< fl_gate(): refused without a specific reason (REJECTED / NONE)        */
/** PV2_R_* reason to refuse, PV2_R_NONE (0) if the flip may start */
uint8_t fl_gate(const fl_gate_in_t *g);

typedef struct {
    float   w[3];               /**< body rates, rad/s                                                 */
    float   cos_tilt, vz, hover;
    float   stick;              /**< largest |roll / pitch stick| (0..1), pilot sticks only            */
    uint8_t link_ok, gyro_sat, est_ok;
} fl_in_t;

typedef struct {
    uint8_t phase;
    uint8_t rate_mode;          /**< 1: body rates (rate[]) ; 0: attitude level                        */
    float   rate[3];            /**< rad/s                                                             */
    float   collective;
    uint8_t abort_reason;       /**< FL_AB_* (also set when the rotation was cut short)                */
    uint8_t vector_hold;        /**< 1: the estimator must skip its accelerometer / magnetometer updates */
} fl_out_t;

typedef struct {
    uint8_t  phase, axis, cut;  /**< cut: the rotation was ended early (aborted, then recovered)       */
    uint32_t t_phase;
    float    angle;             /**< rad turned about the axis                                         */
    float    rate;              /**< rad/s, flipRate                                                   */
} fl_t;

void fl_start(fl_t *f, uint8_t axis, float rate_rad_s);
void fl_cancel(fl_t *f);
void fl_step(fl_t *f, const fl_in_t *in, uint32_t dt_ms, fl_out_t *out);

#endif /* FLIP_H */
