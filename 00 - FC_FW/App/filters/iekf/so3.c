/**
 * @file  so3.c
 * @brief Single-precision SO(3) implementation for Cortex-M4F.
 *
 * Deliberate differences w.r.t. expSO3.m (double):
 *  - small-angle threshold at 0.05 rad with a 4th-order Taylor series: in float,
 *    (1-cos t)/t^2 computed directly loses all digits for t ~ 1e-3, which is
 *    exactly the order of omega*dt at 100 Hz and above;
 *  - so3_orthonormalize(): in single precision the product R*exp(.) accumulates
 *    orthogonality error at every step and must be re-projected onto SO(3).
 */
#include "so3.h"
#include <math.h>

void so3_skew(const float v[3], float S[9])
{
    S[0] = 0.0f;  S[1] = -v[2]; S[2] =  v[1];
    S[3] =  v[2]; S[4] = 0.0f;  S[5] = -v[0];
    S[6] = -v[1]; S[7] =  v[0]; S[8] = 0.0f;
}

void so3_exp(const float phi[3], float R[9])
{
    const float t2 = phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2];
    float a, b;
    if (t2 < 2.5e-3f) {
        a = 1.0f - t2*(1.0f/6.0f)  + t2*t2*(1.0f/120.0f);   /* sin(t)/t       */
        b = 0.5f - t2*(1.0f/24.0f) + t2*t2*(1.0f/720.0f);   /* (1-cos t)/t^2  */
    } else {
        const float t = sqrtf(t2);
        a = sinf(t) / t;
        b = (1.0f - cosf(t)) / t2;
    }
    /* Rodrigues: R = I + a*[phi]x + b*[phi]x^2,  [phi]x^2 = phi*phi' - t2*I */
    R[0] = 1.0f + b*(phi[0]*phi[0] - t2);
    R[1] = -a*phi[2] + b*phi[0]*phi[1];
    R[2] =  a*phi[1] + b*phi[0]*phi[2];
    R[3] =  a*phi[2] + b*phi[1]*phi[0];
    R[4] = 1.0f + b*(phi[1]*phi[1] - t2);
    R[5] = -a*phi[0] + b*phi[1]*phi[2];
    R[6] = -a*phi[1] + b*phi[2]*phi[0];
    R[7] =  a*phi[0] + b*phi[2]*phi[1];
    R[8] = 1.0f + b*(phi[2]*phi[2] - t2);
}

float vec3_dot(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
float vec3_norm(const float v[3])                  { return sqrtf(vec3_dot(v, v)); }

void vec3_cross(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

int vec3_normalize(float v[3])
{
    const float n = vec3_norm(v);
    if (n < 1e-12f) return -1;
    v[0] /= n; v[1] /= n; v[2] /= n;
    return 0;
}

/* Gram-Schmidt on the columns (= body axes expressed in the world frame) */
void so3_orthonormalize(float R[9])
{
    float c0[3] = { R[0], R[3], R[6] };
    float c1[3] = { R[1], R[4], R[7] };
    float c2[3];
    if (vec3_normalize(c0)) return;
    const float d = vec3_dot(c0, c1);
    c1[0] -= d*c0[0]; c1[1] -= d*c0[1]; c1[2] -= d*c0[2];
    if (vec3_normalize(c1)) return;
    vec3_cross(c0, c1, c2);
    R[0] = c0[0]; R[1] = c1[0]; R[2] = c2[0];
    R[3] = c0[1]; R[4] = c1[1]; R[5] = c2[1];
    R[6] = c0[2]; R[7] = c1[2]; R[8] = c2[2];
}

void so3_to_quat(const float R[9], float q[4])
{
    const float tr = R[0] + R[4] + R[8];
    float S;
    if (tr > 0.0f) {
        S = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f*S;            q[1] = (R[7] - R[5]) / S;
        q[2] = (R[2] - R[6]) / S;  q[3] = (R[3] - R[1]) / S;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        S = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        q[0] = (R[7] - R[5]) / S;  q[1] = 0.25f*S;
        q[2] = (R[1] + R[3]) / S;  q[3] = (R[2] + R[6]) / S;
    } else if (R[4] > R[8]) {
        S = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        q[0] = (R[2] - R[6]) / S;  q[1] = (R[1] + R[3]) / S;
        q[2] = 0.25f*S;            q[3] = (R[5] + R[7]) / S;
    } else {
        S = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        q[0] = (R[3] - R[1]) / S;  q[1] = (R[2] + R[6]) / S;
        q[2] = (R[5] + R[7]) / S;  q[3] = 0.25f*S;
    }
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (q[0] < 0.0f) n = -n;             /* hemisphere w >= 0: no jumps in plots      */
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

void mat3_mul(const float A[9], const float B[9], float C[9])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            C[3*r + c] = A[3*r]*B[c] + A[3*r + 1]*B[3 + c] + A[3*r + 2]*B[6 + c];
}

void mat3_vec(const float A[9], const float x[3], float y[3])
{
    y[0] = A[0]*x[0] + A[1]*x[1] + A[2]*x[2];
    y[1] = A[3]*x[0] + A[4]*x[1] + A[5]*x[2];
    y[2] = A[6]*x[0] + A[7]*x[1] + A[8]*x[2];
}

void mat3t_vec(const float A[9], const float x[3], float y[3])
{
    y[0] = A[0]*x[0] + A[3]*x[1] + A[6]*x[2];
    y[1] = A[1]*x[0] + A[4]*x[1] + A[7]*x[2];
    y[2] = A[2]*x[0] + A[5]*x[1] + A[8]*x[2];
}

int mat3_inv(const float A[9], float B[9])
{
    const float c00 = A[4]*A[8] - A[5]*A[7];
    const float c01 = A[5]*A[6] - A[3]*A[8];
    const float c02 = A[3]*A[7] - A[4]*A[6];
    const float det = A[0]*c00 + A[1]*c01 + A[2]*c02;
    if (fabsf(det) < 1e-30f) return -1;
    const float id = 1.0f / det;
    B[0] = c00*id; B[1] = (A[2]*A[7] - A[1]*A[8])*id; B[2] = (A[1]*A[5] - A[2]*A[4])*id;
    B[3] = c01*id; B[4] = (A[0]*A[8] - A[2]*A[6])*id; B[5] = (A[2]*A[3] - A[0]*A[5])*id;
    B[6] = c02*id; B[7] = (A[1]*A[6] - A[0]*A[7])*id; B[8] = (A[0]*A[4] - A[1]*A[3])*id;
    return 0;
}
