/**
 * @file  hover_track.h
 * @brief Hover thrust for the flight core (ADR 0010 §2-3): seed, online estimate, memory between flights.
 *
 * Owns everything about the hover thrust, so the flight core only asks for the value:
 *   - the seed at arm: the estimate of an earlier flight of this power cycle (>= HT_KEEP_MS), else the parameter;
 *   - the 50 Hz update of hover_est, with the command delayed by the motor lag;
 *   - the plausibility warning, once per flight.
 * The flight core decides when the model holds (hover_gate_t) because only it knows the flight state.
 *
 * Runs in the flight-control context only. Host-testable.
 */
#ifndef HOVER_TRACK_H
#define HOVER_TRACK_H

#include <stdint.h>
#include "hover_est.h"

#define HT_UPDATE_MS   20u      /**< estimate update, 50 Hz                    */
#define HT_STEP_MS     4u       /**< period of hover_track_step() (fc_service) */
#define HT_LAG_MS      36u      /**< command -> effect delay (motor lag)       */
#define HT_KEEP_MS     20000u   /**< a flight this long seeds the next one     */
#define HT_MIN_UPDATES 50u      /**< before the estimate is trusted            */
#define HT_SEED_DEFAULT 0.51f   /**< parameter out of range: T400 indoor       */
#define HT_MAX_VZ      0.5f     /**< m/s: faster, drag biases the model (climb test) */
#define HT_MIN_COS     0.866f   /**< tilt < 30 deg                              */

/** inputs of one step. `airborne` is the flight core's part (in air, past the spool-up, off the floor, no geofence
 *  or idle); the conditions of the model itself (vertical speed, tilt, saturation) are checked here */
typedef struct {
    uint8_t airborne;
    uint8_t saturated;  /**< the collective is at a limit of the vertical loop */
    float   a_up;       /**< m/s^2, + up, gravity and accelerometer bias removed */
    float   thrust;     /**< collective commanded at this step, normalised */
    float   cos_tilt;   /**< R33 */
    float   vz;         /**< m/s */
} hover_gate_t;

void    hover_track_init(void);                      /**< power-on: no memory                       */
void    hover_track_arm(float param_seed);           /**< seed for this flight                       */
void    hover_track_disarm(uint32_t flight_ms);      /**< keep the estimate if the flight was long    */
/** every HT_STEP_MS; returns 1 once per flight when the estimate leaves the plausible band */
uint8_t hover_track_step(uint32_t now_ms, uint8_t armed, const hover_gate_t *g);
float   hover_track_value(void);                     /**< hover thrust in use                         */
float   hover_track_sd(void);                        /**< 1-sigma                                     */
float   hover_track_nis(void);                       /**< last NIS                                    */

#endif /* HOVER_TRACK_H */
