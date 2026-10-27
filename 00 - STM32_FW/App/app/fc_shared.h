/**
 * @file  fc_shared.h
 * @brief Data exchanged between the flight context (ImuTask) and the AppTask.
 *
 *   ImuTask -> AppTask  estimator / vertical state   (seqlock, never blocks)
 *   ImuTask -> AppTask  calibration samples          (SPSC, only on request)
 *   AppTask -> ImuTask  calibration to apply         (SPSC)
 *
 * The flight core has its own queues (fc_core.h). Host-testable.
 */
#ifndef FC_SHARED_H
#define FC_SHARED_H

#include <stdint.h>
#include "calib_mgr.h"

typedef struct {
    uint8_t  imu_ok, running, acc_gated, mag_gated;
    float    q[4];               /* body -> world NWU                       */
    float    att_unc_deg;        /* 1 sigma, largest axis                   */
    float    mag_interf_pct;
    float    vib_mg;             /* RMS of |a| - 1 g over ~1 s              */
    uint16_t acc_clip;           /* samples near full scale since arm       */
    float    loop_hz, jitter_us, cpu_pct, imu_temp_c;
    /* vertical / navigation */
    uint8_t  alt_valid, tof_valid, tof_ground;
    float    h, vz, tof_h;
    uint16_t tof_mm;             /* raw, 0xFFFF n/a                         */
    uint8_t  flow_q;             /* 0..100, 0xFF n/a                        */
    uint8_t  flow_ok;            /* bridge alive, PMW3901 OK                */
    uint8_t  flow_usable;        /* above the PMW3901 minimum height (80 mm) */
    float    vel_body[2];        /* m/s from flow x height (x fwd, y left); 0 when not usable */
    uint32_t baro_pa;            /* 0 n/a                                   */
    uint16_t sensors;            /* IMU / MAG_FC / BARO found by the IMU init */
} fsh_est_t;

void fsh_est_write(const fsh_est_t *s);     /* ImuTask */
void fsh_est_read(fsh_est_t *out);          /* AppTask: keeps *out on a race */

/* calibration samples (ImuTask -> AppTask) */
extern volatile uint8_t g_fsh_cal_want;     /* AppTask: 1 while samples are needed */
int  fsh_cal_push(const cal_sample_t *s);
int  fsh_cal_pop(cal_sample_t *s);

/* calibration to apply (AppTask -> ImuTask) */
typedef struct { uint8_t type; cal_set_t set; } fsh_apply_t;
int  fsh_apply_post(uint8_t type, const cal_set_t *set);
int  fsh_apply_take(fsh_apply_t *a);

void fsh_reset(void);

#endif /* FC_SHARED_H */
