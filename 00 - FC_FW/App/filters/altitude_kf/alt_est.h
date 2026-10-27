/**
 * @file  alt_est.h
 * @brief Vertical estimator: height, climb rate, baro bias, accelerometer bias.
 *
 *   x = [ h  v  b_baro  b_acc ]      h  m above the reference (+ up)
 *                                     v  m/s (+ up)
 *   predict : vertical acceleration from the IEKF attitude and the
 *             accelerometer (gravity removed), 240 Hz
 *   baro    : z = h + b_baro          (altitude from pressure, 50 Hz)
 *   ToF     : z = h - ground          (range x cos(tilt) - lens offset, 30 Hz)
 *
 * Without ToF the baro bias is not observable: it is frozen (no process
 * noise), h then follows the baro. With ToF (ACwing Flow) the baro bias is
 * tracked, the height is the ToF height above the floor, and the baro carries
 * the estimate through ToF gaps (out of range, > 30 deg tilt).
 * A ToF reading that jumps (step onto a table, an object under the drone) is
 * gated; if it keeps disagreeing for 0.5 s the ground level is re-anchored.
 * Below the ToF minimum range (~40 mm: landed, the lens is 11 mm above the
 * floor) the reading means "on the ground" (tof_ground).
 *
 * alt_est_zero() sets the reference (FLIGHT.altRel = 0 at the take-off point).
 * Host-testable: no HAL, no RTOS.
 */
#ifndef ALT_EST_H
#define ALT_EST_H

#include <stdint.h>

typedef struct {
    float sigma_acc;        /**< m/s^2, vertical acceleration noise             */
    float sigma_acc_bias;   /**< m/s^2/sqrt(s), random walk                     */
    float sigma_baro_bias;  /**< m/sqrt(s), random walk (while ToF is valid)    */
    float sigma_baro;       /**< m, baro altitude noise                          */
    float sigma_tof;        /**< m, ToF noise at 0 m (+1 % of the range)         */
    float tof_min_m, tof_max_m;
    float tof_max_tilt_cos; /**< cos of the largest usable tilt                 */
    float tof_offset_m;     /**< lens height above the floor when landed        */
} alt_est_cfg_t;

typedef struct {
    alt_est_cfg_t c;
    float    x[4];
    float    P[4][4];
    /** baro */
    float    p_ref;          /**< Pa at the reference                            */
    float    p_sum; uint16_t p_n;
    uint8_t  baro_ok;
    float    baro_alt;       /**< last baro altitude (m, p_ref based)            */
    float    pressure;       /**< last Pa                                        */
    /** ToF */
    float    ground;         /**< floor level in the h frame                     */
    float    tof_h;          /**< last height above the floor (m)                */
    uint8_t  tof_valid;      /**< last ToF sample used                           */
    uint8_t  tof_ground;     /**< below the minimum range: on the floor          */
    uint8_t  tof_rejects;
    float    tof_age_s;      /**< since the last valid ToF sample                */
    uint8_t  have_tof;       /**< a ToF sample was used at least once            */
    uint8_t  valid;          /**< an altitude source is initialised              */
} alt_est_t;

void alt_est_default_cfg(alt_est_cfg_t *c);
void alt_est_init(alt_est_t *e, const alt_est_cfg_t *c);
void alt_est_predict(alt_est_t *e, float a_up, float dt);
void alt_est_baro(alt_est_t *e, float pa);
/** range_m: raw ToF range; status_ok: range status valid; cos_tilt: R33 */
void alt_est_tof(alt_est_t *e, float range_m, uint8_t status_ok, float cos_tilt);
void alt_est_zero(alt_est_t *e);

/** vertical acceleration (m/s^2, + up, gravity removed) from the attitude and the
 * specific force in body axes (g units, 1 g at rest = nom_g) */
float alt_est_accel_up(const float q[4], const float f_body_g[3], float nom_g);

static inline float alt_est_h(const alt_est_t *e)  { return e->x[0]; }
static inline float alt_est_vz(const alt_est_t *e) { return e->x[1]; }

#endif /* ALT_EST_H */
