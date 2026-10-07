/**
 * @file  accwin.h
 * @brief Windowed accelerometer mean, compensated for rotation.
 *
 * A plain mean of N samples taken while the body rotates is "smeared" over
 * different orientations and lags by half a window (at 360 deg/s with a 4 ms
 * window that is ~0.7 deg). Here every sample is brought into the body frame at
 * the START of the window with the propagated rotation D, averaged, and the
 * result is brought back into the CURRENT body frame:
 *
 *     a_mean = D_N' * (1/N) * sum_i D_i * a_i ,   D = E_1 E_2 ... E_k
 *
 * The E are the rotation increments of the IEKF propagation (iekf.E_last),
 * so the cost is a few 3x3 multiplications per sample.
 */
#ifndef ACCWIN_H
#define ACCWIN_H

#include <stdint.h>

typedef struct {
    float    D[9];      /* rotation body(start) <- body(now)     */
    float    sum[3];
    uint16_t n;
} accwin_t;

void accwin_reset(accwin_t *w);
void accwin_add(accwin_t *w, const float acc[3]);           /* sample in the current body  */
void accwin_rotate(accwin_t *w, const float E[9]);          /* after every propagation     */
int  accwin_mean(const accwin_t *w, float out[3]);          /* in the current body, 0 = OK  */

#endif /* ACCWIN_H */
