/**
 * @file  checks.h
 * @brief Pre-flight checks (CHECKS frame, HANDOVER_FW.md §9). Host-testable.
 *
 * Only the items that apply are evaluated; the others are sent as N/A (the
 * app greys them). ready = no item in FAIL.
 */
#ifndef CHECKS_H
#define CHECKS_H

#include <stdint.h>
#include "proto_v2.h"

typedef struct {
    uint16_t sensors;          /**< CAPS bits                                     */
    uint8_t  imu_ok;           /**< IMU initialised                               */
    uint8_t  est_running;      /**< attitude estimate converged and running        */
    float    att_unc_deg;      /**< 1 sigma                                       */
    float    mag_interf_pct;
    uint32_t cal_done_mask;    /**< bit (type-1): calibration done and saved      */
    uint8_t  rc_fresh;         /**< RC from the app in the last 0.5 s             */
    int16_t  rc_thr;
    uint8_t  hold_mode;        /**< ALT_HOLD / POS_HOLD: centre stick = hold        */
    float    gf_max_alt;       /**< param gfMaxAlt                                */
    float    gf_max_dist;      /**< param gfMaxDist                               */
    uint8_t  alt_valid;        /**< vertical estimate running (baro / ToF)         */
    uint8_t  flow_q;           /**< 0..100, 0xFF n/a                              */
    uint8_t  flow_ok;          /**< bridge alive, PMW3901 OK                       */
    uint8_t  in_air;
    /* battery (item 1) */
    uint8_t  batt_monitor;     /**< battery measured (BattTask runs)               */
    uint8_t  batt_stale;       /**< BattTask silent                                */
    uint8_t  batt_present;
    uint8_t  batt_level;       /**< latched level from the flight core             */
    float    batt_cell_v;      /**< average cell, sagged and filtered              */
    float    batt_warn, batt_crit;   /**< params battWarn / battCrit                */
    uint8_t  usb_host;         /**< a USB host is connected (item 12, FAIL)         */
    float    hover;            /**< hover thrust in use, normalised (item 13)       */
    uint8_t  rc_ever, rc_valid;/**< RC receiver seen / link up (item 6)             */
    uint8_t  rc_lq;            /**< link quality %, 0xFF = n/a (link up = 100)      */
    /* GNSS (items 4, 5) */
    uint8_t  gnss_fix;         /**< NAV fix code 0 / 2 / 3 of a fresh solution (0 if stale) */
    uint8_t  gnss_sats;
    uint8_t  home_set;         /**< home recorded (none before the position estimator)     */
    uint8_t  gnss_nav;         /**< the GNSS is a navigation source (0 until the position
                                    estimator fuses it): only then an uncalibrated compass is a FAIL */
} checks_in_t;

/** item 4: WARN below this many satellites */
#define CHECKS_GNSS_MIN_SATS 8u

/** item 13: WARN when hovering needs more than this fraction of the thrust left at the present voltage */
#define CHECKS_THRUST_MARGIN 0.80f

/** fills items (PV2_CHECKS_MAX), returns the count; *ready = no FAIL;
 * *battery_fail = the battery item is in FAIL (ARM -> BATTERY_LOW) */
uint8_t checks_build(const checks_in_t *in, pv2_check_t *items, uint8_t *ready, uint8_t *battery_fail);

#endif /* CHECKS_H */
