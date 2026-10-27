/**
 * @file  est_pipeline.h
 * @brief Multi-rate estimation pipeline, WITHOUT any HAL or RTOS dependency.
 *
 * It receives the FIFO events in the order the sensor wrote them
 * (gyro, acc, mag) and decides by itself what to do and when:
 *
 *   gyro  (ODR)        : accumulate dtheta; every gyro_per_step samples -> step
 *   step  (960 Hz)     : IEKF propagation, accwin_rotate, ctrl_rate_step()
 *   every steps_per_acc: rotated acc mean -> acc update, ctrl_attitude_step(),
 *   steps (240 Hz)       telemetry callback
 *   mag (~50-100 Hz)   : mag update (sample already de-duplicated by the driver)
 *
 * Before starting it runs the static calibration (calib.c): sigmaAcc/sigmaMag
 * are estimated on the windowed acc mean, i.e. exactly on the quantity that
 * later enters the update.
 *
 * Being pure, the same pipeline runs in the firmware and in the PC test.
 */
#ifndef EST_PIPELINE_H
#define EST_PIPELINE_H

#include <stdint.h>
#include <stddef.h>
#include "iekf.h"
#include "accwin.h"
#include "calib.h"
#include "imu_lsm6dsv16x.h"

#define EST_F_RUNNING    0x01u
#define EST_F_MAG_NEW    0x02u
#define EST_F_ACC_GATED  0x04u
#define EST_F_MAG_GATED  0x08u
#define EST_F_STILL      0x20u   /* stillness detector active */

typedef struct {
    uint8_t       gyro_per_step;     /* 2 with ODR 1920 Hz, 1 with 960 Hz      */
    uint8_t       steps_per_acc;     /* 4 -> acc update at 240 Hz             */
    uint16_t      calib_samples;     /* mag samples while standing still     */
    float         calib_gyro_still;  /* rad/s                                   */
    float         sigma_vec_min;
    iekf_config_t ekf;               /* sigma_acc/sigma_mag overwritten by calib */
    float         mag_map[9];        /* LIS2MDL sensor -> body (LSM6DSV16X)  */
    float         mag_hard_iron[3];  /* uT, body frame                          */
    float         mag_scale[3];      /* per-axis soft-iron scale (COMPASS calibration);
                                        0 = 1 (configs that do not set it)       */
    /* Age of the mag sample on arrival: the LIS2MDL converts at f_lis, the sensor
     * hub re-reads it at f_sh > f_lis and duplicates are discarded, so the age is
     * ~uniform in [0, 1/f_sh]: mean 0.5/f_sh, std (1/f_sh)/sqrt(12).
     * The mean is compensated by rotating the measurement forward with the gyro;
     * the random part enters as a variance proportional to |omega|^2. */
    float         mag_latency_s;
    float         mag_latency_std_s;
    /* Stillness detector: if |a| ~ nominal and |omega| small for still_time_s, an
     * external acceleration is physically implausible -> the accelerometer NIS
     * inflation is reset and the filter trusts it again at once (fast recovery
     * from a true attitude error). still_time_s = 0: disabled. */
    float         still_acc_tol;     /* relative deviation of |a|, e.g. 0.02 */
    float         still_gyro_thr;    /* rad/s, e.g. 3 deg/s                  */
    float         still_time_s;      /* e.g. 0.3 s                           */
} est_config_t;

/* ---- trace of the filter calls (parity test with MATLAB) ----
 * Every iekf_* call within an acc-update window is recorded with its EXACT
 * arguments. MATLAB replays it identically and compares state and covariance:
 * this checks the filter independently of accwin, mag latency, etc.
 * Encoding (little-endian, float32):
 *   'P' dtheta[3] h          propagation                     17 bytes
 *   'A' acc[3] trusted(u8)   accelerometer update            14 bytes
 *   'M' mag[3] extra_var     magnetometer update             17 bytes */
#define EST_TRACE_MAX   140u
#define EST_OP_PROP     0x50u   /* 'P' */
#define EST_OP_ACC      0x41u   /* 'A' */
#define EST_OP_MAG      0x4Du   /* 'M' */

typedef struct {
    uint8_t  buf[EST_TRACE_MAX];
    uint16_t len;
    uint8_t  nops;
    uint8_t  overflow;
} est_trace_t;

typedef struct est_s est_t;
typedef void (*est_cb_t)(const est_t *e, void *user);

struct est_s {
    est_config_t cfg;
    iekf_t       f;
    calib_t      cal;
    accwin_t     win;
    uint8_t      running;

    /* step accumulators */
    float    dth[3], gsum[3];
    uint8_t  k;
    uint32_t step;

    /* latest quantities (telemetry / calibration) */
    float    acc_mean[3], gyro_mean[3], mag[3];
    float    omega[3];                /* last step gyro, bias removed          */
    float    gwin_sum[3];
    uint16_t gwin_n;
    uint8_t  have_acc_mean;
    uint8_t  flags;                  /* EST_F_*, cleared after each callback */
    float    t;                      /* s, estimation time since RUN start  */
    float    still_t;                /* s, how long the board has been still */
    calib_result_t calres;

    uint8_t     trace_enable;
    est_trace_t trace;               /* cleared after each on_state */
    uint32_t    win_count;           /* acc windows since RUN start */

    est_cb_t on_state;               /* 240 Hz, after the acc update */
    est_cb_t on_init;                /* once, at end of calibration  */
    void    *user;
};

void est_init(est_t *e, const est_config_t *cfg, est_cb_t on_state, est_cb_t on_init, void *user);
/* dt_sample: IMU sample period measured by the MCU (s) */
void est_process(est_t *e, const imu_event_t *ev, size_t n, float dt_sample);

#endif /* EST_PIPELINE_H */
