/**
 * @file  iekf.h
 * @brief Attitude IEKF on SO(3), C port of iekfAttitude.m.
 *
 * Same model, same parameters, same equations:
 *   left-invariant error   R_true = R_hat * exp(delta)
 *   error state            x = [delta; beta],  beta = b_true - b_hat
 *   F = [exp(-w*h), -h*I; 0, I],   H = [skew(y_hat), 0]
 * 6x6 covariance with CMSIS-DSP (arm_mat_*), 3x3 geometry hand-written.
 *
 * Multi-rate use (phase 1, see README.md):
 *   960 Hz : iekf_propagate_dtheta(&f, dtheta, dt)   dtheta = sum of omega*dt
 *                                                   over the gyro samples of the step
 *   240 Hz : iekf_update_acc(&f, rotated_acc_mean)  (see accwin.h)
 *   ~100 Hz: iekf_update_mag(&f, mag)               only on new data
 *
 * State: 3 (attitude) + 3 (gyro bias) by default; cfg.estimate_bias = 0
 * reduces the filter exactly to the 3-state version.
 */
#ifndef IEKF_H
#define IEKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IEKF_NX 6
#define IEKF_P_ATT_MIN   1e-12f   /* rad^2      : floor on the diagonal of P */
#define IEKF_P_BIAS_MIN  1e-16f   /* (rad/s)^2                               */

typedef struct {
    float   sigma_gyro;     /* rad/s,   gyro white noise                    */
    float   sigma_bias;     /* rad/s^2, bias random walk                    */
    float   sigma_acc;      /* per-component std of the normalised vector   */
    float   sigma_mag;
    float   acc_tol;        /* relative tolerance on the magnitude (gating) */
    float   mag_tol;
    float   p0_att;         /* initial attitude variance [rad^2]            */
    float   p0_bias;        /* initial bias variance     [(rad/s)^2]        */
    uint8_t estimate_bias;
    uint8_t adaptive;
    uint8_t tilt_comp_mag;
    /* --- robustness to external accelerations / magnetic disturbances --- */
    float   acc_nis_gate;    /* NIS gate (chi^2, 2 dof) above which R_acc is
                                inflated by NIS/gate. 0 = off. 9.21 = 99%        */
    float   mag_nis_gate;    /* same for the magnetometer. NOT RECOMMENDED: a high
                                mag NIS is often a true yaw error, and de-weighting
                                it slows recovery. Prefer dip_tol.                */
    float   dip_tol;         /* rad: the mag is de-weighted when the angle between
                                the MEASURED field and gravity departs from the
                                calibration value (magnetic disturbance). Independent
                                of the estimate. 0 = off                          */
    float   infl_decay;      /* per-update decay of the inflation (memory):
                                exp(-dt_update/tau), e.g. tau = 0.25 s           */
    uint8_t decouple;        /* 1: acc corrects only roll/pitch, mag only yaw    */
    uint8_t reset_jacobian;  /* 1: transport P after the retraction (default)    */
    /* --- protection against "lockout" of the NIS inflation ---
     * If the estimate is truly wrong, the innovation is large because of the
     * estimate, not the measurement: NIS would inflate R forever and the filter
     * would stop correcting (very slow convergence). Hence:
     *  - the inflation is capped at infl_max (the correction never vanishes);
     *  - if the acc stays de-weighted for more than lockout_updates consecutive
     *    updates AND the innovation, seen in the WORLD frame, is persistent
     *    (its mean does not cancel out), it is an estimation error rather than
     *    an acceleration: P_aa += lockout_p_bump*I and the acc is trusted again
     *    for grace_updates updates.
     *    Why the world frame: with a correct gyro, a true attitude error stays
     *    fixed in the world frame, while the acceleration of a hand shaking the
     *    board oscillates with zero mean.                                       */
    float    infl_max;          /* e.g. 100                                    */
    uint16_t lockout_updates;   /* e.g. 120 (0.5 s at 240 Hz); 0 = off         */
    float    lockout_p_bump;    /* rad^2, e.g. (10 deg)^2                      */
    uint16_t grace_updates;     /* e.g. 60 (0.25 s)                            */
    float    persist_decay;     /* moving mean of the innovation: exp(-dt/tau), tau ~ 1 s */
    float    persist_ratio;     /* |mean|^2 / mean(|e|^2) above which it is persistent, e.g. 0.6 */
    float    persist_min;       /* minimum |mean| (sine of the error), e.g. 0.02 (~1 deg) */
} iekf_config_t;

/* Initial conditions, typically from calib_finish() */
typedef struct {
    float R0[9];
    float b0[3];
    float g_ref[3];         /* accelerometer direction at rest, world frame */
    float m_ref[3];         /* magnetic field direction, world frame        */
    float nom_acc;          /* nominal magnitudes for the adaptive gating   */
    float nom_mag;
} iekf_seed_t;

typedef struct {
    iekf_config_t cfg;
    float R[9];
    float b[3];
    float P[IEKF_NX * IEKF_NX];
    float g_ref[3], m_ref[3];
    float nom_acc, nom_mag;
    float last_acc_dev, last_mag_dev;   /* relative magnitude deviation    */
    float acc_infl, mag_infl;           /* current inflation of R (>= 1)   */
    float last_acc_nis, last_mag_nis;
    float last_acc_dir[3];              /* last measured acc direction (for dip) */
    float dip_ref;                      /* g_ref-m_ref angle, rad                */
    float last_dip_dev;
    uint8_t acc_trusted;                /* set from outside (stillness detector):
                                           no NIS inflation on the next acc update */
    uint16_t acc_gated_run;             /* consecutive acc updates with inflation > 2 */
    uint16_t acc_grace;                 /* remaining acc updates without inflation   */
    uint32_t lockouts;                  /* how many times the protection fired       */
    float    innov_w_mean[3];           /* moving mean of acc innovation, world frame */
    float    innov_w_pow;               /* moving mean of |innovation|^2             */
    float    last_acc_innov_b[3];       /* acc innovation (body) before the update   */
    float E_last[9];                    /* rotation of the last propagation step     */
    /* static workspace: no large matrices on the RTOS task stack */
    float wF[36], wA[36], wB[36];
    float wH[18], wHt[18], wPHt[18], wK[18];
} iekf_t;

void iekf_default_config(iekf_config_t *cfg);
void iekf_init(iekf_t *f, const iekf_config_t *cfg, const iekf_seed_t *seed);
/* Propagation with a measured angle increment (bias NOT yet removed):
 * phi = dtheta - b*dt. Use this form when a step spans several samples. */
void iekf_propagate_dtheta(iekf_t *f, const float dtheta[3], float dt);
/* Single-sample case: dtheta = gyro*dt (compatible with the previous port) */
void iekf_propagate(iekf_t *f, const float gyro[3], float dt);
/* Return 1 if the measurement was "gated" (de-weighted), 0 otherwise */
int  iekf_update_acc(iekf_t *f, const float acc[3]);
int  iekf_update_mag(iekf_t *f, const float mag[3]);
/* As above, with an additional per-component variance (e.g. uncertainty due to
 * the age of the sample during a fast rotation). */
int  iekf_update_mag_ex(iekf_t *f, const float mag[3], float extra_var);
void iekf_get_quat(const iekf_t *f, float q[4]);

#ifdef __cplusplus
}
#endif
#endif /* IEKF_H */
