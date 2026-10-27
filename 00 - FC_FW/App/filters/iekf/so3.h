/**
 * @file  so3.h
 * @brief Single-precision SO(3) Lie-group utilities (port of expSO3.m /
 *        skewSym.m + firmware helpers).
 *
 * Convention: 3x3 ROW-MAJOR matrices (as CMSIS-DSP), R rotates body -> world.
 * Scalar-first Hamilton quaternions [w x y z], same convention as
 * rotm2quatLocal in runSyntheticBenchmark.m.
 */
#ifndef SO3_H
#define SO3_H

#ifdef __cplusplus
extern "C" {
#endif

void  so3_skew(const float v[3], float S[9]);
void  so3_exp(const float phi[3], float R[9]);
void  so3_orthonormalize(float R[9]);
void  so3_to_quat(const float R[9], float q[4]);

void  mat3_mul(const float A[9], const float B[9], float C[9]);   /**< C = A*B   */
void  mat3_vec(const float A[9], const float x[3], float y[3]);   /**< y = A*x   */
void  mat3t_vec(const float A[9], const float x[3], float y[3]);  /**< y = A'*x  */
int   mat3_inv(const float A[9], float Ai[9]);                    /**< 0 = OK    */

float vec3_norm(const float v[3]);
float vec3_dot(const float a[3], const float b[3]);
void  vec3_cross(const float a[3], const float b[3], float c[3]);
int   vec3_normalize(float v[3]);                                 /**< 0 = OK    */

#ifdef __cplusplus
}
#endif
#endif /* SO3_H */
