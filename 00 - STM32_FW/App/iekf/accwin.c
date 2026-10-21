/**
 * @file  accwin.c
 */
#include "accwin.h"
#include "so3.h"
#include <string.h>

void accwin_reset(accwin_t *w)
{
    memset(w, 0, sizeof *w);
    w->D[0] = w->D[4] = w->D[8] = 1.0f;
}

void accwin_add(accwin_t *w, const float acc[3])
{
    float v[3];
    mat3_vec(w->D, acc, v);
    w->sum[0] += v[0]; w->sum[1] += v[1]; w->sum[2] += v[2];
    w->n++;
}

void accwin_rotate(accwin_t *w, const float E[9])
{
    float Dn[9];
    mat3_mul(w->D, E, Dn);
    memcpy(w->D, Dn, sizeof Dn);
}

int accwin_mean(const accwin_t *w, float out[3])
{
    if (w->n == 0) return -1;
    const float k = 1.0f / (float)w->n;
    const float m[3] = { w->sum[0]*k, w->sum[1]*k, w->sum[2]*k };
    mat3t_vec(w->D, m, out);
    return 0;
}
