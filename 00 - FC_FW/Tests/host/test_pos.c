/**
 * Host tests of the horizontal position hold (control/pos_ctrl.c) in closed loop with a horizontal plant:
 * the tilt loop as a first-order lag (tau 0.17 s, as the tilt loop of ctrl_test), acceleration g tan(tilt) about
 * the heading, linear rotor drag, a constant wind, the yaw turning, and the flow velocity as app_main.c makes it
 * (noise, 0.3 low-pass at 100 Hz).
 */
#include "pos_ctrl.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define G0 9.80665f
#define DT 0.004f                     /* fc_service period */

typedef struct {
    float p[2], v[2];                 /* world NW, m, m/s                        */
    float roll, pitch, yaw, yaw_rate; /* actual attitude                          */
    float wind[2];                    /* world, m/s^2 disturbance                 */
    float mu;                         /* 1/s rotor drag                           */
    float flow[2];                    /* body, filtered                           */
    float noise;                      /* m/s 1-sigma of a flow sample             */
} plant_t;

static float s_tau = 0.17f, s_flow_k = 1.0f;   /* tilt-loop lag, flow scale error (robustness) */

static float rnd(void) { return ((float)rand() / (float)RAND_MAX - 0.5f) * 3.46f; }

static void plant_step(plant_t *pl, float roll_sp, float pitch_sp, float dt)
{
    const float k = dt / s_tau;
    pl->roll += (roll_sp - pl->roll) * k;
    pl->pitch += (pitch_sp - pl->pitch) * k;
    pl->yaw += pl->yaw_rate * dt;
    /* acceleration in the heading frame: forward from pitch (+ nose down), left from -roll */
    const float ah[2] = { G0 * tanf(pl->pitch), -G0 * tanf(pl->roll) };
    const float c = cosf(pl->yaw), s = sinf(pl->yaw);
    const float aw[2] = { c * ah[0] - s * ah[1], s * ah[0] + c * ah[1] };
    for (int i = 0; i < 2; ++i) {
        pl->v[i] += (aw[i] - pl->mu * pl->v[i] + pl->wind[i]) * dt;
        pl->p[i] += pl->v[i] * dt;
    }
    /* flow: body velocity + noise, low-pass as app_main.c (100 Hz samples ~ every 2.5 steps: once per step here) */
    const float vb[2] = { c * pl->v[0] + s * pl->v[1], -s * pl->v[0] + c * pl->v[1] };
    for (int i = 0; i < 2; ++i) pl->flow[i] += 0.3f * (s_flow_k * vb[i] + pl->noise * rnd() - pl->flow[i]);
}

typedef struct { float fwd, left; } stick_t;

/* runs t seconds; returns the largest tilt asked (rad) */
static float run(pos_ctrl_t *c, plant_t *pl, float t, stick_t st, float vmax, float tilt_max)
{
    float worst = 0.0f;
    for (int k = 0; k < (int)(t / DT + 0.5f); ++k) {
        const pc_in_t in = { { pl->flow[0], pl->flow[1] }, pl->yaw, st.fwd, st.left, vmax, tilt_max, DT };
        float r, p;
        pc_step(c, &in, &r, &p);
        const float tilt = sqrtf(r * r + p * p);
        if (tilt > worst) worst = tilt;
        plant_step(pl, r, p, DT);
    }
    return worst;
}

static void setup(pos_ctrl_t *c, plant_t *pl)
{
    pc_gains_t g; pc_gains_default(&g); pc_init(c, &g);
    memset(pl, 0, sizeof *pl);
    pl->mu = 0.3f; pl->noise = 0.05f;
}

static float dist(const float a[2], const float b[2]) { return hypotf(a[0] - b[0], a[1] - b[1]); }

int main(void)
{
    srand(3);
    const float tmax = 30.0f * 0.0174533f;
    pos_ctrl_t c; plant_t pl;
    const stick_t none = { 0, 0 };

    /* 1. released at 1 m/s forward: brakes, holds the point where it stopped, no pull back */
    setup(&c, &pl);
    pl.v[0] = 1.0f; pl.flow[0] = 1.0f;
    (void)run(&c, &pl, 1.5f, none, 2.0f, tmax);
    CHECK(c.state == PC_HOLD, "braked and holding (state %u)", c.state);
    const float stop[2] = { pl.p[0], pl.p[1] };
    printf("  release at 1 m/s: stopped after %.2f m\n", stop[0]);
    CHECK(stop[0] > 0.05f && stop[0] < 0.6f, "stopping distance %.2f m", stop[0]);
    (void)run(&c, &pl, 8.0f, none, 2.0f, tmax);
    CHECK(dist(pl.p, stop) < 0.10f && hypotf(pl.v[0], pl.v[1]) < 0.1f, "holds within 10 cm of the stop (%.3f m)", dist(pl.p, stop));

    /* 2. constant wind 0.8 m/s^2 sideways: the integrator holds the point */
    setup(&c, &pl);
    pl.wind[1] = 0.8f;
    float worst = 0.0f;
    for (int k = 0; k < 20; ++k) {
        (void)run(&c, &pl, 0.5f, none, 2.0f, tmax);
        if (k >= 4) { const float o[2] = { 0, 0 }; const float d = dist(pl.p, o); if (d > worst) worst = d; }
    }
    const float o0[2] = { 0, 0 };
    printf("  wind 0.8 m/s^2: worst drift %.3f m after 2 s, final %.3f m\n", worst, dist(pl.p, o0));
    CHECK(dist(pl.p, o0) < 0.05f, "wind: back on the point (%.3f m)", dist(pl.p, o0));
    CHECK(worst < 0.35f, "wind: drift bounded (%.3f m)", worst);

    /* 3. stick half forward: velocity = half vmax, along the heading */
    setup(&c, &pl);
    pl.yaw = 1.0f;
    const stick_t half = { 0.5f, 0.0f };
    (void)run(&c, &pl, 4.0f, half, 2.0f, tmax);
    const float vh = cosf(pl.yaw) * pl.v[0] + sinf(pl.yaw) * pl.v[1];
    const float vl = -sinf(pl.yaw) * pl.v[0] + cosf(pl.yaw) * pl.v[1];
    CHECK(fabsf(vh - 1.0f) < 0.1f && fabsf(vl) < 0.05f, "half stick: 1 m/s forward (%.2f, side %.2f)", vh, vl);
    CHECK(c.state == PC_MOVE, "moving");
    /* stick left (roll stick right = -left): to the right */
    setup(&c, &pl);
    const stick_t right = { 0.0f, -0.5f };
    (void)run(&c, &pl, 4.0f, right, 2.0f, tmax);
    CHECK(pl.v[1] < -0.9f, "roll stick right: moves to the right, -y (%.2f)", pl.v[1]);

    /* 4. full stick: the tilt never exceeds maxTilt */
    setup(&c, &pl);
    const stick_t full = { 1.0f, 1.0f };
    const float wt = run(&c, &pl, 3.0f, full, 8.0f, tmax);
    CHECK(wt <= tmax + 1e-4f, "tilt within maxTilt (%.1f deg)", wt * 57.2958f);

    /* 5. yawing at 1 rad/s while holding: the point stays (dead reckoning in the world frame) */
    setup(&c, &pl);
    (void)run(&c, &pl, 1.0f, none, 2.0f, tmax);
    const float p0[2] = { pl.p[0], pl.p[1] };
    pl.yaw_rate = 1.0f;
    pl.wind[0] = 0.4f;
    (void)run(&c, &pl, 10.0f, none, 2.0f, tmax);
    CHECK(dist(pl.p, p0) < 0.15f, "yawing with wind: held within 15 cm (%.3f m)", dist(pl.p, p0));

    /* 6. margins: the tilt loop twice as slow and the flow scale 25 % off (assumed numbers, uncalibrated flow) */
    s_tau = 0.34f; s_flow_k = 1.25f;
    setup(&c, &pl);
    pl.v[0] = 1.0f; pl.flow[0] = 1.25f; pl.wind[1] = 0.5f;
    (void)run(&c, &pl, 7.0f, none, 2.0f, tmax);                 /* settled */
    const float q0[2] = { pl.p[0], pl.p[1] };
    float vpk = 0.0f;
    for (int k = 0; k < 24; ++k) { (void)run(&c, &pl, 0.25f, none, 2.0f, tmax); const float vv = hypotf(pl.v[0], pl.v[1]); if (vv > vpk) vpk = vv; }
    printf("  slow tilt loop, flow +25 %%: moved %.3f m in 6 s once settled, peak speed %.3f m/s\n", dist(pl.p, q0), vpk);
    CHECK(dist(pl.p, q0) < 0.05f && vpk < 0.1f, "margins: stable and holding (%.3f m, %.3f m/s)", dist(pl.p, q0), vpk);
    s_tau = 0.17f; s_flow_k = 1.0f;

    /* 7. reset: the next hold starts from where the drone is */
    pc_reset(&c);
    CHECK(c.state == PC_BRAKE && c.i[0] == 0.0f && c.p[0] == 0.0f, "reset");

    printf("pos: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
