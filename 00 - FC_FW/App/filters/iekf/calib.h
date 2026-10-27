/**
 * @file  calib.h
 * @brief Initial static window: references, noise and initial attitude.
 *
 * C equivalent of section 2 of runRealData.m + directionNoiseStd.m, with ONE
 * deliberate difference: the world frame is not "body at t=0" but a levelled
 * NWU frame (x = horizontal magnetic north, y = west, z = up), built with the
 * TRIAD method. Reason: the tilt-compensated magnetometer update assumes a
 * vertical world z; with world = body(t0) and the board not perfectly level
 * that assumption is violated. As a bonus, the MATLAB triad shows a true heading.
 */
#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>
#include "iekf.h"

#ifndef CALIB_MAX_SAMPLES
#define CALIB_MAX_SAMPLES 256
#endif

typedef struct {
    uint32_t n;
    float acc[CALIB_MAX_SAMPLES][3];
    float mag[CALIB_MAX_SAMPLES][3];
    float gyro_sum[3];
    float gyro_norm_max;
} calib_t;

typedef struct {
    iekf_seed_t seed;
    float sigma_acc;        /**< already lower-bounded by sigma_min     */
    float sigma_mag;
    float sigma_acc_raw;    /**< measured values, for diagnostics        */
    float sigma_mag_raw;
} calib_result_t;

void     calib_reset(calib_t *c);
/** Returns 0 if the sample is compatible with "still", -1 if moving */
int      calib_add(calib_t *c, const float acc[3], const float gyro[3], const float mag[3],
                   float gyro_still_thr);
uint32_t calib_count(const calib_t *c);
int      calib_finish(const calib_t *c, float sigma_min, calib_result_t *out);
float    direction_noise_std(const float (*v)[3], uint32_t n);

#endif /* CALIB_H */
