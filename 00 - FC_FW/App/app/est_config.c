/**
 * @file  est_config.c
 * @brief Production estimator configuration (see est_config.h).
 */
#include "est_config.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

void app_est_config(est_config_t *ec)
{
    memset(ec, 0, sizeof *ec);
    ec->gyro_per_step    = (APP_IMU_ODR == IMU_ODR_1920HZ) ? 2u : 1u;
    ec->steps_per_acc    = APP_STEPS_PER_ACC;
    ec->calib_samples    = APP_CALIB_SAMPLES;
    ec->calib_gyro_still = APP_CALIB_GYRO_STILL;
    ec->sigma_vec_min    = IEKF_SIGMA_VEC_MIN;
    iekf_default_config(&ec->ekf);
    ec->ekf.sigma_gyro = IEKF_SIGMA_GYRO;   ec->ekf.sigma_bias = IEKF_SIGMA_BIAS;
    ec->ekf.acc_tol    = IEKF_ACC_TOL;      ec->ekf.mag_tol    = IEKF_MAG_TOL;
    ec->ekf.p0_att     = IEKF_P0_ATT;       ec->ekf.p0_bias    = IEKF_P0_BIAS;
    ec->ekf.estimate_bias = IEKF_ESTIMATE_BIAS;
    ec->ekf.adaptive      = IEKF_ADAPTIVE;
    ec->ekf.tilt_comp_mag = IEKF_TILT_COMP_MAG;
    ec->ekf.acc_nis_gate  = IEKF_ACC_NIS_GATE;
    ec->ekf.mag_nis_gate  = IEKF_MAG_NIS_GATE;
    ec->ekf.infl_decay    = expf(-((float)APP_STEPS_PER_ACC / (float)APP_STEP_HZ) / IEKF_INFL_TAU_S);
    ec->ekf.dip_tol       = IEKF_DIP_TOL;
    ec->ekf.decouple      = IEKF_DECOUPLE;
    {
        const float hacc = (float)APP_STEPS_PER_ACC / (float)APP_STEP_HZ;   /* s between acc updates */
        ec->ekf.infl_max        = IEKF_INFL_MAX;
        ec->ekf.lockout_updates = (uint16_t)(IEKF_LOCKOUT_S / hacc + 0.5f);
        ec->ekf.lockout_p_bump  = IEKF_LOCKOUT_BUMP;
        ec->ekf.grace_updates   = (uint16_t)(IEKF_GRACE_S / hacc + 0.5f);
        ec->ekf.persist_decay   = expf(-hacc / IEKF_PERSIST_TAU_S);
        ec->ekf.persist_ratio   = IEKF_PERSIST_RATIO;
        ec->ekf.persist_min     = IEKF_PERSIST_MIN;
    }
    ec->still_acc_tol     = APP_STILL_ACC_TOL;
    ec->still_gyro_thr    = APP_STILL_GYRO_THR;
    ec->still_time_s      = APP_STILL_TIME_S;
    ec->ekf.reset_jacobian = IEKF_RESET_JACOBIAN;
    ec->ekf.tangent_innov  = IEKF_TANGENT_INNOV;
    {
        const float tsh = 1.0f / IMU_SH_HZ(APP_MAG_FAST);
        ec->mag_latency_s     = 0.5f * tsh;
        ec->mag_latency_std_s = tsh * 0.288675f;      /* 1/sqrt(12) */
    }
#if APP_COMPARE_ORIGINAL
    ec->ekf.acc_nis_gate   = 0.0f;   ec->ekf.mag_nis_gate = 0.0f;
    ec->ekf.dip_tol        = 0.0f;   ec->ekf.decouple     = 0;
    ec->ekf.reset_jacobian = 0;      ec->still_time_s     = 0.0f;
    ec->ekf.lockout_updates = 0;
    ec->ekf.tangent_innov  = 0;      /* the original iekfAttitude.m uses y - yhat */
    ec->mag_latency_std_s  = 0.0f;   /* the MEAN-latency compensation stays: it is an input */
#endif
}
