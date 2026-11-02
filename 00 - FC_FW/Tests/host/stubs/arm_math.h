/* Host stand-in for the three CMSIS-DSP matrix functions iekf.c uses
 * (the firmware links the ARM library; the host has no Cortex-M build of it).
 * Same semantics: row-major, dst distinct from the sources. */
#ifndef ARM_MATH_HOST_STUB_H
#define ARM_MATH_HOST_STUB_H

#include <stdint.h>

typedef float float32_t;
typedef enum { ARM_MATH_SUCCESS = 0, ARM_MATH_SIZE_MISMATCH = -3 } arm_status;
typedef struct { uint16_t numRows, numCols; float32_t *pData; } arm_matrix_instance_f32;

static inline void arm_mat_init_f32(arm_matrix_instance_f32 *m, uint16_t r, uint16_t c, float32_t *p)
{
    m->numRows = r; m->numCols = c; m->pData = p;
}

static inline arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B,
                                          arm_matrix_instance_f32 *C)
{
    if (A->numCols != B->numRows || C->numRows != A->numRows || C->numCols != B->numCols) return ARM_MATH_SIZE_MISMATCH;
    for (uint16_t r = 0; r < A->numRows; ++r)
        for (uint16_t c = 0; c < B->numCols; ++c) {
            float32_t s = 0.0f;
            for (uint16_t k = 0; k < A->numCols; ++k) s += A->pData[r * A->numCols + k] * B->pData[k * B->numCols + c];
            C->pData[r * C->numCols + c] = s;
        }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *T)
{
    if (T->numRows != A->numCols || T->numCols != A->numRows) return ARM_MATH_SIZE_MISMATCH;
    for (uint16_t r = 0; r < A->numRows; ++r)
        for (uint16_t c = 0; c < A->numCols; ++c) T->pData[c * T->numCols + r] = A->pData[r * A->numCols + c];
    return ARM_MATH_SUCCESS;
}

#endif
