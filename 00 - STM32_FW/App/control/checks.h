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
    uint16_t sensors;          /* CAPS bits                                     */
    uint8_t  imu_ok;           /* IMU initialised                               */
    uint8_t  est_running;      /* attitude estimate converged and running        */
    float    att_unc_deg;      /* 1 sigma                                       */
    float    mag_interf_pct;
    uint32_t cal_done_mask;    /* bit (type-1): calibration done and saved      */
    uint8_t  rc_fresh;         /* RC from the app in the last 0.5 s             */
    int16_t  rc_thr;
    uint8_t  hold_mode;        /* ALT_HOLD / POS_HOLD: centre stick = hold        */
    float    gf_max_alt;       /* param gfMaxAlt                                */
    float    gf_max_dist;      /* param gfMaxDist                               */
    uint8_t  alt_valid;        /* vertical estimate running (baro / ToF)         */
    uint8_t  flow_q;           /* 0..100, 0xFF n/a                              */
} checks_in_t;

/* fills items (PV2_CHECKS_MAX), returns the count; *ready = no FAIL;
 * *battery_fail = the battery item is in FAIL (ARM -> BATTERY_LOW) */
uint8_t checks_build(const checks_in_t *in, pv2_check_t *items, uint8_t *ready, uint8_t *battery_fail);

#endif /* CHECKS_H */
