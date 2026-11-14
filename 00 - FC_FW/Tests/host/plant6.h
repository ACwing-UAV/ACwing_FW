/**
 * 6-DOF plant of the 1S micro quad for the host tests (vehicle-model.md, "Design numbers"): rigid body with the
 * inertia J, four rotors with a first-order lag tau and the thrust quadratic in the applied voltage, linear drag.
 * Motor order and torque signs as mixer.h (M1 RR, M2 FR, M3 RL, M4 FL). Used by test_launch.c and test_flip.c.
 */
#ifndef PLANT6_H
#define PLANT6_H

#include <math.h>
#include <string.h>

#ifndef G0
#define G0 9.80665f
#endif

typedef struct {
    float q[4], w[3], p[3], v[3];          /* body->world, body rates, world position / velocity */
    float T[4];                            /* rotor thrust, N                                    */
    float J[3], arm, kappa, tmax, tau, mass, vcell;
} plant_t;

static inline void qmul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
static inline void rot(const float q[4], const float b[3], float o[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2*(y*y + z*z))*b[0] + 2*(x*y - w*z)*b[1] + 2*(x*z + w*y)*b[2];
    o[1] = 2*(x*y + w*z)*b[0] + (1 - 2*(x*x + z*z))*b[1] + 2*(y*z - w*x)*b[2];
    o[2] = 2*(x*z - w*y)*b[0] + 2*(y*z + w*x)*b[1] + (1 - 2*(x*x + y*y))*b[2];
}

static inline void plant_init(plant_t *pl, float mass, float vcell)
{
    memset(pl, 0, sizeof *pl);
    pl->q[0] = 1.0f;
    pl->J[0] = pl->J[1] = 3.5e-5f; pl->J[2] = 6e-5f;
    pl->arm = 0.028f; pl->kappa = 0.012f; pl->tmax = 0.343f; pl->tau = 0.035f; pl->mass = mass; pl->vcell = vcell;
}

/* duty per motor -> rotor thrust (quadratic in the applied voltage, lag), rigid body */
static inline void plant_step(plant_t *pl, const float duty[4], float dt)
{
    const float k = pl->vcell / 4.2f;
    for (int i = 0; i < 4; ++i) {
        const float u = duty[i] * k;
        const float tss = pl->tmax * u * u;
        pl->T[i] += (tss - pl->T[i]) * dt / pl->tau;
    }
    const float *T = pl->T;
    /* M1 RR, M2 FR, M3 RL, M4 FL (mixer.h) */
    const float tau[3] = { pl->arm * ((T[2] + T[3]) - (T[0] + T[1])),
                           pl->arm * ((T[0] + T[2]) - (T[1] + T[3])),
                           pl->kappa * ((T[0] + T[3]) - (T[1] + T[2])) };
    const float *J = pl->J, *w = pl->w;
    const float Jw[3] = { J[0]*w[0], J[1]*w[1], J[2]*w[2] };
    const float c[3] = { w[1]*Jw[2] - w[2]*Jw[1], w[2]*Jw[0] - w[0]*Jw[2], w[0]*Jw[1] - w[1]*Jw[0] };
    for (int i = 0; i < 3; ++i) pl->w[i] += (tau[i] - c[i]) / J[i] * dt;
    const float h = 0.5f * dt;
    const float dq[4] = { 1.0f, pl->w[0] * h, pl->w[1] * h, pl->w[2] * h };
    float qn[4]; qmul(pl->q, dq, qn);
    const float n = sqrtf(qn[0]*qn[0] + qn[1]*qn[1] + qn[2]*qn[2] + qn[3]*qn[3]);
    for (int i = 0; i < 4; ++i) pl->q[i] = qn[i] / n;
    const float fb[3] = { 0.0f, 0.0f, (T[0] + T[1] + T[2] + T[3]) / pl->mass };
    float fw[3]; rot(pl->q, fb, fw);
    for (int i = 0; i < 3; ++i) {
        const float a = fw[i] - (i == 2 ? G0 : 0.0f) - 0.3f * pl->v[i];
        pl->v[i] += a * dt; pl->p[i] += pl->v[i] * dt;
    }
}

#endif /* PLANT6_H */
