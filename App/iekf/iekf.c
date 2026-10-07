/**
 * @file  iekf.c
 * @brief Attitude IEKF on SO(3) - port of iekfAttitude.m (see iekf.h).
 */
#include "iekf.h"
#include "so3.h"
#include "arm_math.h"
#include <string.h>
#include <math.h>

#define NX IEKF_NX

static void condition_covariance(iekf_t *f);

/* ---- CMSIS-DSP wrappers (dst always distinct from the sources) ---------- */
static void mmul(float *C, const float *A, uint16_t ar, uint16_t ac,
                 const float *B, uint16_t bc)
{
    arm_matrix_instance_f32 mA, mB, mC;
    arm_mat_init_f32(&mA, ar, ac, (float32_t *)A);
    arm_mat_init_f32(&mB, ac, bc, (float32_t *)B);
    arm_mat_init_f32(&mC, ar, bc, C);
    (void)arm_mat_mult_f32(&mA, &mB, &mC);
}

static void mtrans(float *At, const float *A, uint16_t r, uint16_t c)
{
    arm_matrix_instance_f32 mA, mT;
    arm_mat_init_f32(&mA, r, c, (float32_t *)A);
    arm_mat_init_f32(&mT, c, r, At);
    (void)arm_mat_trans_f32(&mA, &mT);
}

static void symmetrize(float *P)
{
    for (int r = 0; r < NX; ++r)
        for (int c = r + 1; c < NX; ++c) {
            const float m = 0.5f * (P[r*NX + c] + P[c*NX + r]);
            P[r*NX + c] = m;
            P[c*NX + r] = m;
        }
}

/* ------------------------------------------------------------------------ */
void iekf_default_config(iekf_config_t *c)
{
    c->sigma_gyro    = 0.5f * 0.017453293f;
    c->sigma_bias    = 1e-4f;
    c->sigma_acc     = 0.05f;
    c->sigma_mag     = 0.05f;
    c->acc_tol       = 0.08f;
    c->mag_tol       = 0.08f;
    c->p0_att        = 1e-2f;
    c->p0_bias       = 1e-4f;
    c->estimate_bias = 1;
    c->adaptive      = 1;
    c->tilt_comp_mag = 0;
    c->acc_nis_gate  = 0.0f;
    c->mag_nis_gate  = 0.0f;
    c->infl_decay    = 0.0f;
    c->decouple      = 0;
    c->dip_tol       = 0.0f;
    c->reset_jacobian = 1;
    c->infl_max        = 1e4f;
    c->lockout_updates = 0;
    c->lockout_p_bump  = 0.0f;
    c->grace_updates   = 0;
    c->persist_decay   = 0.0f;
    c->persist_ratio   = 0.6f;
    c->persist_min     = 0.02f;
}

void iekf_init(iekf_t *f, const iekf_config_t *cfg, const iekf_seed_t *s)
{
    memset(f, 0, sizeof *f);
    f->cfg = *cfg;
    f->E_last[0] = f->E_last[4] = f->E_last[8] = 1.0f;
    memcpy(f->R, s->R0, sizeof f->R);
    so3_orthonormalize(f->R);
    if (cfg->estimate_bias) memcpy(f->b, s->b0, sizeof f->b);

    memcpy(f->g_ref, s->g_ref, sizeof f->g_ref);  (void)vec3_normalize(f->g_ref);
    memcpy(f->m_ref, s->m_ref, sizeof f->m_ref);  (void)vec3_normalize(f->m_ref);
    f->nom_acc = (s->nom_acc > 1e-6f) ? s->nom_acc : 1.0f;
    f->nom_mag = (s->nom_mag > 1e-6f) ? s->nom_mag : 1.0f;
    f->acc_infl = f->mag_infl = 1.0f;
    f->dip_ref = acosf(fmaxf(-1.0f, fminf(1.0f, vec3_dot(f->g_ref, f->m_ref))));
    mat3t_vec(f->R, f->g_ref, f->last_acc_dir);

    /* Without bias estimation the bias block stays identically zero:
     * the filter reduces exactly to the 3-state MATLAB version. */
    for (int i = 0; i < 3; ++i) {
        f->P[i*NX + i]             = cfg->p0_att;
        f->P[(i + 3)*NX + (i + 3)] = cfg->estimate_bias ? cfg->p0_bias : 0.0f;
    }
}

void iekf_propagate_dtheta(iekf_t *f, const float dtheta[3], float h)
{
    float phi[3], E[9], Rn[9];
    for (int i = 0; i < 3; ++i) phi[i] = dtheta[i] - f->b[i] * h;

    /* Exact integration on the group */
    so3_exp(phi, E);
    mat3_mul(f->R, E, Rn);
    memcpy(f->R, Rn, sizeof Rn);
    so3_orthonormalize(f->R);
    memcpy(f->E_last, E, sizeof E);

    /* F depends ONLY on the measured rate (group-affine property) */
    memset(f->wF, 0, sizeof f->wF);
    for (int i = 0; i < NX; ++i) f->wF[i*NX + i] = 1.0f;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) f->wF[r*NX + c] = E[c*3 + r];   /* exp(-phi) = E' */
        if (f->cfg.estimate_bias) f->wF[r*NX + 3 + r] = -h;
    }

    /* P = F P F' + Q */
    mmul(f->wA, f->wF, NX, NX, f->P, NX);
    mtrans(f->wB, f->wF, NX, NX);
    mmul(f->P, f->wA, NX, NX, f->wB, NX);

    const float qg = f->cfg.sigma_gyro * f->cfg.sigma_gyro * h;
    const float qb = f->cfg.estimate_bias ? f->cfg.sigma_bias * f->cfg.sigma_bias * h : 0.0f;
    for (int i = 0; i < 3; ++i) {
        f->P[i*NX + i]             += qg;
        f->P[(i + 3)*NX + (i + 3)] += qb;
    }
    condition_covariance(f);
}

void iekf_propagate(iekf_t *f, const float gyro[3], float h)
{
    const float dth[3] = { gyro[0]*h, gyro[1]*h, gyro[2]*h };
    iekf_propagate_dtheta(f, dth, h);
}

/* P_aa <- J P_aa J',  P_ab <- J P_ab,  P_ba <- P_ab',  J = I - 1/2 [dth]x */
static void reset_covariance(iekf_t *f, const float dth[3])
{
    float J[9], S[9], A[9], T[9], Jt[9];
    so3_skew(dth, S);
    for (int i = 0; i < 9; ++i) J[i] = -0.5f * S[i];
    J[0] += 1.0f; J[4] += 1.0f; J[8] += 1.0f;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) { A[3*r + c] = f->P[r*NX + c]; Jt[3*c + r] = J[3*r + c]; }
    mat3_mul(J, A, T);
    mat3_mul(T, Jt, A);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) f->P[r*NX + c] = A[3*r + c];
    if (f->cfg.estimate_bias) {
        for (int c = 0; c < 3; ++c) {          /* columns of the P_ab block */
            const float b0 = f->P[0*NX + 3 + c], b1 = f->P[1*NX + 3 + c], b2 = f->P[2*NX + 3 + c];
            for (int r = 0; r < 3; ++r) {
                const float v = J[3*r]*b0 + J[3*r + 1]*b1 + J[3*r + 2]*b2;
                f->P[r*NX + 3 + c]   = v;
                f->P[(3 + c)*NX + r] = v;       /* P_ba = P_ab' */
            }
        }
    }
}

/* Single-precision numerical hygiene: exact symmetry and a diagonal never
 * below a minimum floor (the Joseph form guarantees P >= 0 in exact arithmetic,
 * not in float32). The bias block stays zero when not estimated. */
static void condition_covariance(iekf_t *f)
{
    symmetrize(f->P);
    for (int i = 0; i < 3; ++i)
        if (f->P[i*NX + i] < IEKF_P_ATT_MIN) f->P[i*NX + i] = IEKF_P_ATT_MIN;
    if (f->cfg.estimate_bias)
        for (int i = 3; i < 6; ++i)
            if (f->P[i*NX + i] < IEKF_P_BIAS_MIN) f->P[i*NX + i] = IEKF_P_BIAS_MIN;
}

typedef enum { PROJ_NONE = 0, PROJ_TILT_ONLY = 1, PROJ_YAW_ONLY = 2 } proj_t;

/* Projects the columns of K (attitude and bias rows) w.r.t. the vertical axis
 * expressed in the body frame, a = R' * g_ref:
 *   TILT_ONLY: removes the component along a -> the measurement cannot touch yaw
 *   YAW_ONLY : keeps only the component along a -> the measurement touches only yaw
 * The gain is no longer optimal, but the Joseph form keeps P consistent with
 * ANY gain: the covariance stays correct. */
static void project_gain(iekf_t *f, proj_t proj)
{
    if (proj == PROJ_NONE) return;
    float a[3];
    mat3t_vec(f->R, f->g_ref, a);
    (void)vec3_normalize(a);
    for (int blk = 0; blk < 2; ++blk) {
        if (blk == 1 && !f->cfg.estimate_bias) break;
        for (int col = 0; col < 3; ++col) {
            float *k0 = &f->wK[(3*blk + 0)*3 + col];
            float *k1 = &f->wK[(3*blk + 1)*3 + col];
            float *k2 = &f->wK[(3*blk + 2)*3 + col];
            const float d = a[0]*(*k0) + a[1]*(*k1) + a[2]*(*k2);
            if (proj == PROJ_TILT_ONLY) {
                *k0 -= d*a[0]; *k1 -= d*a[1]; *k2 -= d*a[2];
            } else {
                *k0 = d*a[0];  *k1 = d*a[1];  *k2 = d*a[2];
            }
        }
    }
}

/* Common vector correction (MATLAB applyUpdate) with:
 *  - robust inflation of R driven by the normalised innovation (NIS):
 *    an external acceleration perpendicular to gravity barely changes |a|
 *    but rotates the measured direction a lot: NIS sees it, the magnitude does not;
 *  - memory of the inflation (*infl decays slowly towards 1), because real
 *    disturbances oscillate and cross zero;
 *  - optional gain projection (yaw / tilt decoupling).
 * Returns the NIS computed with the nominal R. */
static float vector_update(iekf_t *f, const float y[3], const float yhat[3], float r,
                           float nis_gate, float *infl, proj_t proj)
{
    float S0[9], S3[9], Si[9], S[9], innov[3], dx[NX];

    memset(f->wH, 0, sizeof f->wH);
    so3_skew(yhat, S);
    for (int rr = 0; rr < 3; ++rr)
        for (int c = 0; c < 3; ++c) f->wH[rr*NX + c] = S[rr*3 + c];

    mtrans(f->wHt, f->wH, 3, NX);                 /* H'           6x3 */
    mmul(f->wPHt, f->P, NX, NX, f->wHt, 3);       /* P H'         6x3 */
    mmul(S0, f->wH, 3, NX, f->wPHt, 3);           /* H P H'       3x3 */
    for (int i = 0; i < 3; ++i) innov[i] = y[i] - yhat[i];

    /* NIS with the nominal R */
    float nis = 0.0f;
    memcpy(S3, S0, sizeof S3);
    for (int i = 0; i < 3; ++i) S3[i*3 + i] += r;
    if (mat3_inv(S3, Si)) return 0.0f;
    float t[3];
    mat3_vec(Si, innov, t);
    nis = vec3_dot(innov, t);

    /* inflation: instantaneous NIS/gate, with memory */
    float k = 1.0f;
    if (nis_gate > 0.0f && infl) {
        const float inst = nis / nis_gate;
        float g = *infl * f->cfg.infl_decay;
        if (g < 1.0f) g = 1.0f;
        if (inst > g) g = inst;
        if (g > f->cfg.infl_max) g = f->cfg.infl_max;
        *infl = g;
        k = g;
    }
    const float re = r * k;
    if (k != 1.0f) {
        memcpy(S3, S0, sizeof S3);
        for (int i = 0; i < 3; ++i) S3[i*3 + i] += re;
        if (mat3_inv(S3, Si)) return nis;
    }
    mmul(f->wK, f->wPHt, NX, 3, Si, 3);           /* K = P H' S^-1 6x3 */
    project_gain(f, proj);

    for (int i = 0; i < NX; ++i)
        dx[i] = f->wK[i*3]*innov[0] + f->wK[i*3 + 1]*innov[1] + f->wK[i*3 + 2]*innov[2];

    /* Retraction onto SO(3) */
    float E[9], Rn[9];
    so3_exp(dx, E);
    mat3_mul(f->R, E, Rn);
    memcpy(f->R, Rn, sizeof Rn);
    so3_orthonormalize(f->R);
    if (f->cfg.estimate_bias) {                   /* dx(4:6) estimates b_true - b_hat */
        f->b[0] += dx[3]; f->b[1] += dx[4]; f->b[2] += dx[5];
    }

    /* Joseph form: P = (I-KH) P (I-KH)' + re K K'  (valid for any K) */
    mmul(f->wA, f->wK, NX, 3, f->wH, NX);         /* KH */
    for (int i = 0; i < NX*NX; ++i) f->wA[i] = -f->wA[i];
    for (int i = 0; i < NX; ++i) f->wA[i*NX + i] += 1.0f;
    mmul(f->wB, f->wA, NX, NX, f->P, NX);         /* (I-KH) P */
    mtrans(f->wF, f->wA, NX, NX);                 /* (I-KH)'  (wF reused) */
    mmul(f->P, f->wB, NX, NX, f->wF, NX);
    for (int rr = 0; rr < NX; ++rr)
        for (int c = 0; c < NX; ++c)
            f->P[rr*NX + c] += re * (f->wK[rr*3]*f->wK[c*3] + f->wK[rr*3 + 1]*f->wK[c*3 + 1]
                                   + f->wK[rr*3 + 2]*f->wK[c*3 + 2]);
    symmetrize(f->P);

    /* Error reset: after R <- R*exp(dtheta) the error is defined w.r.t. the NEW
     * estimate, delta' ~ delta - dtheta - 1/2 [dtheta]x delta, hence
     * P_aa <- J P_aa J', P_ab <- J P_ab with J = I - 1/2 [dtheta]x.
     * Negligible at steady state, relevant for large corrections (convergence). */
    if (f->cfg.reset_jacobian) reset_covariance(f, dx);
    condition_covariance(f);
    return nis;
}

static float adaptive_var(float r0, float n, float nom, float tol, int adaptive, float *dev_out)
{
    const float dev = fabsf(n - nom) / nom;
    *dev_out = dev;
    if (!adaptive) return r0;
    const float x = dev / tol;
    return r0 * (1.0f + x*x);
}

int iekf_update_acc(iekf_t *f, const float acc[3])
{
    const float na = vec3_norm(acc);
    if (na < 1e-9f) return 0;
    float y[3] = { acc[0]/na, acc[1]/na, acc[2]/na }, yhat[3];
    memcpy(f->last_acc_dir, y, sizeof y);
    mat3t_vec(f->R, f->g_ref, yhat);

    const float r = adaptive_var(f->cfg.sigma_acc * f->cfg.sigma_acc, na, f->nom_acc,
                                 f->cfg.acc_tol, f->cfg.adaptive, &f->last_acc_dev);
    for (int i = 0; i < 3; ++i) f->last_acc_innov_b[i] = y[i] - yhat[i];
    const int trusted = f->acc_trusted || f->acc_grace > 0;
    if (f->acc_grace > 0) f->acc_grace--;
    if (trusted) f->acc_infl = 1.0f;
    f->last_acc_nis = vector_update(f, y, yhat, r,
                                    trusted ? 0.0f : f->cfg.acc_nis_gate, &f->acc_infl,
                                    f->cfg.decouple ? PROJ_TILT_ONLY : PROJ_NONE);
    f->acc_trusted = 0;

    /* persistence of the innovation in the world frame (the innovation is the one
     * computed BEFORE the update, i.e. independent of the correction just applied) */
    {
        float ew[3];
        mat3_vec(f->R, f->last_acc_innov_b, ew);
        const float a = f->cfg.persist_decay;
        for (int i = 0; i < 3; ++i) f->innov_w_mean[i] = a*f->innov_w_mean[i] + (1.0f - a)*ew[i];
        f->innov_w_pow = a*f->innov_w_pow + (1.0f - a)*vec3_dot(ew, ew);
    }

    /* lockout protection */
    if (f->cfg.lockout_updates > 0) {
        const float m2 = vec3_dot(f->innov_w_mean, f->innov_w_mean);
        const int persistent = (f->innov_w_pow > 0.0f) &&
                               (m2 > f->cfg.persist_ratio * f->innov_w_pow) &&
                               (m2 > f->cfg.persist_min * f->cfg.persist_min);
        if (f->acc_infl > 2.0f) {
            if (++f->acc_gated_run >= f->cfg.lockout_updates && persistent) {
                for (int i = 0; i < 3; ++i) f->P[i*NX + i] += f->cfg.lockout_p_bump;
                f->acc_infl      = 1.0f;
                f->acc_gated_run = 0;
                f->acc_grace     = f->cfg.grace_updates;
                f->lockouts++;
            }
        } else {
            f->acc_gated_run = 0;
        }
    }
    return (f->last_acc_dev > f->cfg.acc_tol) || (f->acc_infl > 2.0f);
}

int iekf_update_mag(iekf_t *f, const float mag[3])
{
    return iekf_update_mag_ex(f, mag, 0.0f);
}

int iekf_update_mag_ex(iekf_t *f, const float mag[3], float extra_var)
{
    const float nm = vec3_norm(mag);
    if (nm < 1e-9f) return 0;
    float y[3] = { mag[0]/nm, mag[1]/nm, mag[2]/nm }, yhat[3];

    if (f->cfg.tilt_comp_mag) {
        /* Tilt-compensated compass: only the horizontal component matters.
         * Requires a world frame with vertical z -> guaranteed by calib.c. */
        float hw[3], bw[3];
        mat3_vec(f->R, y, hw);
        bw[0] = sqrtf(hw[0]*hw[0] + hw[1]*hw[1]);
        bw[1] = 0.0f;
        bw[2] = hw[2];
        mat3t_vec(f->R, bw, yhat);
    } else {
        mat3t_vec(f->R, f->m_ref, yhat);
    }

    float r = adaptive_var(f->cfg.sigma_mag * f->cfg.sigma_mag, nm, f->nom_mag,
                           f->cfg.mag_tol, f->cfg.adaptive, &f->last_mag_dev);
    /* Magnetic dip check: angle between the measured field and gravity,
     * independent of the attitude estimate (a yaw error does not change it). */
    f->last_dip_dev = 0.0f;
    if (f->cfg.dip_tol > 0.0f) {
        const float dip = acosf(fmaxf(-1.0f, fminf(1.0f, vec3_dot(y, f->last_acc_dir))));
        f->last_dip_dev = fabsf(dip - f->dip_ref);
        const float x = f->last_dip_dev / f->cfg.dip_tol;
        r *= 1.0f + x*x;
    }
    r += extra_var;
    f->last_mag_nis = vector_update(f, y, yhat, r, f->cfg.mag_nis_gate, &f->mag_infl,
                                    f->cfg.decouple ? PROJ_YAW_ONLY : PROJ_NONE);
    return (f->last_mag_dev > f->cfg.mag_tol) || (f->mag_infl > 2.0f)
        || (f->cfg.dip_tol > 0.0f && f->last_dip_dev > f->cfg.dip_tol);
}

void iekf_get_quat(const iekf_t *f, float q[4])
{
    so3_to_quat(f->R, q);
}
