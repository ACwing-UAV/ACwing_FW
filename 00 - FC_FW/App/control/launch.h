/**
 * @file  launch.h
 * @brief Hand launch (ADR 0011): the state machine from "armed in the hand, motors stopped" to "recovered".
 *
 *   READY      armed, motors stopped. The release is armed only after a presentation: held still and level
 *              (|w| < 1 rad/s, |f - 1 g| < 0.1 g, tilt < 30 deg) for LN_PRESENT_MS.
 *   (push)     after the presentation, a specific force above LN_PUSH_G (the toss) is remembered
 *   RELEASED   free fall (f < LN_FREEFALL_G) for LN_FREEFALL_MS after a push (or without one when drop is allowed),
 *              gyro not saturated. The motors stay off until the drone is LN_SEPARATION_M from the release point
 *              (dead reckoning of the world acceleration: the velocity from the presentation, where it was still,
 *              integrated through the push; the position from the first free-fall sample), or until a toss reaches
 *              its apex at least LN_APEX_MIN_M above the release point (a gentle toss never rises 10 cm: started at
 *              the apex, the propellers are as far from the hand as they get). A toss that does not rise
 *              LN_APEX_MIN_M would fall back onto the hand: aborted with the motors off, unless launchDrop. Only a
 *              drop has the time fallback LN_SEP_MAX_MS.
 *   RECOVER    motors on: full thrust while falling, then the hover thrust; attitude level, yaw rate 0
 *   DONE       tilt < 15 deg, |w| < 1 rad/s, v_z >= 0 (after LN_RECOVER_MIN_MS): the caller holds the height
 *   ABORT      READY for LN_READY_MAX_MS, the link lost, not recovered in LN_RECOVER_MAX_MS, the estimate lost
 *
 * Pure logic, no I/O, flight-control context (fc_core calls it at its 250 Hz). Host-testable.
 */
#ifndef LAUNCH_H
#define LAUNCH_H

#include <stdint.h>

#define LN_PRESENT_MS       500u
#define LN_PUSH_G           1.3f
#define LN_FREEFALL_G       0.3f
#define LN_FREEFALL_MS      60u
#define LN_SEPARATION_M     0.10f
#define LN_SEP_MAX_MS       300u     /**< drop only: falling, it must recover anyway                   */
#define LN_APEX_MIN_M       0.04f    /**< a toss starts the motors at its apex if it rose this much     */
#define LN_QUIET_MS         200u     /**< 0.7..1.3 g this long: velocity zeroed, push forgotten         */
#define LN_READY_MAX_MS     10000u
#define LN_LINK_LOST_MS     1000u
#define LN_RECOVER_MIN_MS   100u
#define LN_RECOVER_MAX_MS   1500u
#define LN_TW_MIN           1.5f     /**< thrust-to-weight available now, toss */
#define LN_TW_MIN_DROP      1.7f     /**< with a drop (launchDrop) */

enum { LN_IDLE = 0, LN_READY = 1, LN_RELEASED = 2, LN_RECOVER = 3, LN_DONE = 4, LN_ABORT = 5 };
/** abort reasons (EVENT 12 arg) */
enum { LN_AB_NONE = 0, LN_AB_TIMEOUT = 1, LN_AB_LINK = 2, LN_AB_NOT_RECOVERED = 3, LN_AB_ESTIMATE = 4,
       LN_AB_WEAK_TOSS = 5 /**< the toss rose less than LN_APEX_MIN_M: motors kept off (no launchDrop) */ };

typedef struct {
    float   f_g;            /**< specific force magnitude, g                                  */
    float   a_w[3];         /**< world acceleration without gravity, m/s^2 (NWU)              */
    float   w;              /**< |body rate|, rad/s                                           */
    float   cos_tilt;
    uint8_t gyro_sat;       /**< a gyro axis saturated since the last step                    */
    float   vz;             /**< m/s, altitude estimate                                       */
    uint8_t est_ok;         /**< attitude estimate running                                    */
    uint8_t link_ok;        /**< the controlling input is fresh                               */
    float   hover;          /**< hover thrust estimate (0..1)                                  */
} ln_in_t;

typedef struct {
    uint8_t phase;          /**< LN_*                                                         */
    uint8_t motors_off;     /**< 1: armed, duty 0                                             */
    float   thrust;         /**< collective while recovering                                  */
    uint8_t released;       /**< 1 on the step of the motor start (EVENT 11)                   */
    uint16_t start_ms;      /**< ms from the free-fall start to the motor start (EVENT 11 arg) */
    uint8_t abort_reason;   /**< LN_AB_* when phase = LN_ABORT                                */
} ln_out_t;

typedef struct {
    uint8_t  phase, drop, presented, pushed;
    uint32_t t_phase;       /**< ms in the phase                                              */
    uint32_t still_ms, ff_ms, link_bad_ms, quiet_ms;
    float    v[3];          /**< dead-reckoned velocity since the presentation, m/s           */
    float    p[3];          /**< position since the first free-fall sample, m                 */
    uint32_t sep_ms;        /**< since the free-fall start                                    */
} ln_t;

/** thrust-to-weight available now: (1 / hover) (V_cell / 4.2)^2; 0 if unknown */
float ln_tw_available(float hover, float v_cell);

void ln_start(ln_t *l, uint8_t drop_allowed);
void ln_cancel(ln_t *l);
void ln_step(ln_t *l, const ln_in_t *in, uint32_t dt_ms, ln_out_t *out);

#endif /* LAUNCH_H */
