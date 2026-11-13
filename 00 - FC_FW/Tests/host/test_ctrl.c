/**
 * Host tests of the phase-3 loops (att_ctrl.c) and the quad-X mixer (mixer.c), in closed loop with a rotational
 * model of the 1S micro quad (vehicle-model.md): rigid body with inertia J, four rotors with a first-order lag
 * tau_m and thrust quadratic in the applied voltage, gyro noise. Attitude loop at 240 Hz, rate loop at 960 Hz,
 * the plant integrated at 3840 Hz.
 */
#include "att_ctrl.h"
#include "mixer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define D2R 0.017453292519943295f
#define R2D 57.29577951308232f

/* ------------------------------------------------------------------ plant */
typedef struct {
    float q[4], w[3], T[4];
    float J[3], arm, kappa, tmax, tau_m, v_cell;
    float dist[3];                      /* N m, constant disturbance torque */
    float noise;                        /* rad/s, gyro noise (uniform +-) */
} plant_t;

static float frand(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

static void plant_init(plant_t *p, float yaw0)
{
    memset(p, 0, sizeof *p);
    p->q[0] = cosf(yaw0 / 2); p->q[3] = sinf(yaw0 / 2);
    p->J[0] = p->J[1] = 3.5e-5f; p->J[2] = 6e-5f;
    p->arm = 0.028f; p->kappa = 0.012f; p->tmax = 0.343f; p->tau_m = 0.035f; p->v_cell = 4.2f;
    p->noise = 0.02f;
    for (int i = 0; i < 4; ++i) p->T[i] = 0.5f * p->tmax;
}

static void plant_step(plant_t *p, const float duty[4], float dt)
{
    for (int i = 0; i < 4; ++i) {
        const float u = duty[i] * p->v_cell / MIXER_VREF;
        const float tss = p->tmax * u * u;
        p->T[i] += (tss - p->T[i]) * dt / p->tau_m;
    }
    const float *T = p->T;
    const float tau[3] = {
        p->arm * ((T[2] + T[3]) - (T[0] + T[1])) + p->dist[0],     /* left (M3, M4) up -> roll right  */
        p->arm * ((T[0] + T[2]) - (T[1] + T[3])) + p->dist[1],     /* rear (M1, M3) up -> nose down   */
        p->kappa * ((T[0] + T[3]) - (T[1] + T[2])) + p->dist[2] };  /* CW props (M1, M4) up -> CCW yaw */
    const float *w = p->w, *J = p->J;
    const float Jw[3] = { J[0] * w[0], J[1] * w[1], J[2] * w[2] };
    const float wxJw[3] = { w[1] * Jw[2] - w[2] * Jw[1], w[2] * Jw[0] - w[0] * Jw[2], w[0] * Jw[1] - w[1] * Jw[0] };
    for (int i = 0; i < 3; ++i) p->w[i] += (tau[i] - wxJw[i]) / J[i] * dt;
    /* q <- q (x) exp(w dt / 2) (first order, then normalised) */
    const float a = 0.5f * dt, *q = p->q;
    const float dq[4] = {
        -a * (q[1] * w[0] + q[2] * w[1] + q[3] * w[2]),
         a * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]),
         a * (q[0] * w[1] + q[3] * w[0] - q[1] * w[2]),
         a * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0]) };
    float n = 0.0f;
    for (int i = 0; i < 4; ++i) { p->q[i] += dq[i]; n += p->q[i] * p->q[i]; }
    n = 1.0f / sqrtf(n);
    for (int i = 0; i < 4; ++i) p->q[i] *= n;
}

static void euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    float s = 2 * (w * y - z * x);
    s = s > 1 ? 1 : (s < -1 ? -1 : s);
    *pitch = asinf(s);
    *yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/* ------------------------------------------------------------------ closed loop */
typedef struct {
    plant_t     p;
    att_ctrl_t  c;
    mixer_cfg_t m;
    mixer_out_t mo;
    float       collective;
    uint8_t     integrate;
    float       duty[4];
} sim_t;

static void sim_init(sim_t *s, float yaw0)
{
    memset(s, 0, sizeof *s);
    plant_init(&s->p, yaw0);
    att_gains_t g;
    att_gains_default(&g);
    att_ctrl_init(&s->c, &g);
    s->m.idle = 0.05f; s->m.theta = 1.0f;
    s->collective = 0.5f;
    s->integrate = 1;
    for (int i = 0; i < 4; ++i) s->duty[i] = sqrtf(0.5f);
}

/* runs `sec` seconds; sp for the attitude loop */
static void sim_run(sim_t *s, const att_sp_t *sp, float sec, void (*probe)(sim_t *, float, void *), void *u)
{
    const int n = (int)(sec * 960.0f + 0.5f);
    static float t;
    for (int k = 0; k < n; ++k) {
        if (k % 4 == 0) att_ctrl_attitude(&s->c, s->p.q, sp);
        float gyro[3], tq[3];
        for (int i = 0; i < 3; ++i) gyro[i] = s->p.w[i] + s->p.noise * frand();
        att_ctrl_rate(&s->c, gyro, s->integrate, tq);
        mixer_quadx(s->collective, tq, s->p.v_cell, &s->m, &s->mo);
        memcpy(s->duty, s->mo.duty, sizeof s->duty);
        for (int j = 0; j < 4; ++j) plant_step(&s->p, s->duty, 1.0f / 3840.0f);
        t += 1.0f / 960.0f;
        if (probe) probe(s, (float)k / 960.0f, u);
    }
}

/* step metrics on one Euler angle */
typedef struct { int axis; float target, peak, settle, last_out; } step_t;
static void step_probe(sim_t *s, float t, void *u)
{
    step_t *m = u;
    float a[3];
    euler(s->p.q, &a[0], &a[1], &a[2]);
    const float v = a[m->axis];
    if (fabsf(v) > fabsf(m->peak)) m->peak = v;
    if (fabsf(v - m->target) > 1.0f * D2R) m->last_out = t;
    m->settle = m->last_out;
}

static void test_angle_steps(float v_cell, const char *tag)
{
    for (int axis = 0; axis < 2; ++axis) {
        sim_t s; sim_init(&s, 0.0f); s.p.v_cell = v_cell;
        att_sp_t sp; memset(&sp, 0, sizeof sp);
        sim_run(&s, &sp, 0.5f, NULL, NULL);                        /* settle at level */
        if (axis == 0) sp.roll = 20.0f * D2R; else sp.pitch = 20.0f * D2R;
        step_t m = { axis, 20.0f * D2R, 0, 0, 0 };
        sim_run(&s, &sp, 2.0f, step_probe, &m);
        float a[3]; euler(s.p.q, &a[0], &a[1], &a[2]);
        printf("  %s %s step 20 deg: settle (1 deg) %.3f s, peak %.2f deg, final %.2f deg\n",
               tag, axis ? "pitch" : "roll ", m.settle, m.peak * R2D, a[axis] * R2D);
        CHECK(fabsf(a[axis] - m.target) < 1.0f * D2R, "%s axis %d final %.2f deg", tag, axis, a[axis] * R2D);
        CHECK(m.settle < 0.8f, "%s axis %d settles in %.3f s", tag, axis, m.settle);
        CHECK(m.peak * R2D < 23.0f, "%s axis %d overshoot (peak %.2f deg)", tag, axis, m.peak * R2D);
        CHECK(fabsf(a[1 - axis]) < 1.5f * D2R, "%s axis %d: the other tilt stays level (%.2f deg)", tag, axis, a[1 - axis] * R2D);
    }
}

static void test_heading_independent(void)
{
    /* heading 90 deg (facing west in NWU): pitch 15 deg nose down is about the body y axis, not the world one */
    sim_t s; sim_init(&s, 90.0f * D2R);
    att_sp_t sp; memset(&sp, 0, sizeof sp);
    sp.pitch = 15.0f * D2R;
    sim_run(&s, &sp, 1.5f, NULL, NULL);
    float r, p, y; euler(s.p.q, &r, &p, &y);
    printf("  heading 90 deg, pitch 15: roll %.2f pitch %.2f yaw %.2f deg\n", r * R2D, p * R2D, y * R2D);
    CHECK(fabsf(p - 15.0f * D2R) < 1.0f * D2R && fabsf(r) < 1.0f * D2R, "pitch about the body axis at any heading");
    CHECK(fabsf(y - 90.0f * D2R) < 3.0f * D2R, "heading kept (%.2f deg)", y * R2D);
}

static void test_acro(void)
{
    sim_t s; sim_init(&s, 0.0f);
    att_sp_t sp; memset(&sp, 0, sizeof sp);
    sp.acro = 1; sp.roll_rate = 3.0f;
    sim_run(&s, &sp, 0.3f, NULL, NULL);
    const float w_after = s.p.w[0];
    printf("  ACRO roll rate 3 rad/s: %.3f rad/s after 0.3 s\n", w_after);
    CHECK(fabsf(w_after - 3.0f) < 0.3f, "ACRO rate tracked (%.3f)", w_after);
    sp.roll_rate = 0.0f;
    sim_run(&s, &sp, 0.3f, NULL, NULL);
    float r0, p0, y0; euler(s.p.q, &r0, &p0, &y0);
    sim_run(&s, &sp, 0.7f, NULL, NULL);
    float r1, p1, y1; euler(s.p.q, &r1, &p1, &y1);
    printf("  ACRO released at %.1f deg roll, %.1f deg 0.7 s later\n", r0 * R2D, r1 * R2D);
    CHECK(fabsf(r0) > 30.0f * D2R && fabsf(r1 - r0) < 3.0f * D2R, "ACRO: no self-levelling (%.1f -> %.1f deg)", r0 * R2D, r1 * R2D);
}

static void test_yaw(void)
{
    sim_t s; sim_init(&s, 0.0f);
    att_sp_t sp; memset(&sp, 0, sizeof sp);
    sp.yaw_rate = 1.5f;                                             /* FLU: counter-clockwise */
    sim_run(&s, &sp, 0.8f, NULL, NULL);
    float r, p, y; euler(s.p.q, &r, &p, &y);
    printf("  yaw rate 1.5 rad/s: %.3f rad/s after 0.8 s, yaw %.1f deg\n", s.p.w[2], y * R2D);
    CHECK(fabsf(s.p.w[2] - 1.5f) < 0.15f, "yaw rate tracked (%.3f)", s.p.w[2]);
    CHECK(y > 30.0f * D2R, "yaw positive = counter-clockwise (%.1f deg)", y * R2D);
    CHECK(fabsf(r) < 1.5f * D2R && fabsf(p) < 1.5f * D2R, "level while yawing");
}

static void test_disturbance(void)
{
    /* centre of gravity off to the left: constant roll torque. The integrator removes the error. */
    float err_i, err_noi;
    for (int with_i = 1; with_i >= 0; --with_i) {
        sim_t s; sim_init(&s, 0.0f);
        s.p.dist[0] = -0.003f; s.integrate = (uint8_t)with_i;
        att_sp_t sp; memset(&sp, 0, sizeof sp);
        sim_run(&s, &sp, 4.0f, NULL, NULL);
        float r, p, y; euler(s.p.q, &r, &p, &y);
        if (with_i) err_i = r; else err_noi = r;
    }
    printf("  roll disturbance 3 mN m: error %.2f deg with I, %.2f deg without\n", err_i * R2D, err_noi * R2D);
    CHECK(fabsf(err_i) < 0.5f * D2R, "I removes a constant torque (%.2f deg)", err_i * R2D);
    CHECK(fabsf(err_noi) > 2.0f * fabsf(err_i) && fabsf(err_noi) > 0.5f * D2R, "and without I the error stays (%.2f deg)", err_noi * R2D);
}

static void test_battery_comp(void)
{
    /* the same hover thrust at 4.2 and 3.6 V per cell: the duty scales by 4.2 / 3.6 */
    mixer_cfg_t m = { 0.05f, 1.0f };
    const float z[3] = { 0, 0, 0 };
    mixer_out_t a, b;
    mixer_quadx(0.5f, z, 4.2f, &m, &a);
    mixer_quadx(0.5f, z, 3.6f, &m, &b);
    CHECK(fabsf(b.duty[0] / a.duty[0] - 4.2f / 3.6f) < 1e-4f, "battery compensation 4.2/3.6 (%.4f)", b.duty[0] / a.duty[0]);
    mixer_quadx(0.5f, z, 0.0f, &m, &b);
    CHECK(fabsf(b.duty[0] - a.duty[0]) < 1e-6f && b.batt_k == 1.0f, "battery unknown: no compensation");
    test_angle_steps(3.6f, "3.6 V");
}

static void test_mixer(void)
{
    mixer_cfg_t m = { 0.05f, 1.0f };
    mixer_out_t o;
    /* signs: + roll raises the left motors (M3, M4), + pitch the rear ones (M1, M3), + yaw the CW ones (M1, M4) */
    float tq[3] = { 0.1f, 0, 0 };
    mixer_quadx(0.5f, tq, 0, &m, &o);
    CHECK(o.t[2] > o.t[0] && o.t[3] > o.t[1] && fabsf(o.t[2] - o.t[0] - 0.2f) < 1e-5f, "roll + : left motors up");
    tq[0] = 0; tq[1] = 0.1f;
    mixer_quadx(0.5f, tq, 0, &m, &o);
    CHECK(o.t[0] > o.t[1] && o.t[2] > o.t[3], "pitch + : rear motors up");
    tq[1] = 0; tq[2] = 0.1f;
    mixer_quadx(0.5f, tq, 0, &m, &o);
    CHECK(o.t[0] > o.t[1] && o.t[3] > o.t[2], "yaw + : CW motors (M1, M4) up");
    /* full collective: the roll difference is kept, the collective comes down */
    tq[0] = 0.2f; tq[1] = 0; tq[2] = 0;
    mixer_quadx(0.95f, tq, 0, &m, &o);
    CHECK(fabsf((o.t[2] - o.t[0]) - 0.4f) < 1e-5f && fabsf(o.t[2] - 1.0f) < 1e-5f && o.sat_col, "airmode high: roll kept, collective %.3f", o.collective);
    /* zero collective: the motors stay at idle or above, the roll difference is kept */
    mixer_quadx(0.0f, tq, 0, &m, &o);
    CHECK(fabsf(o.t[0] - 0.05f) < 1e-5f && fabsf((o.t[2] - o.t[0]) - 0.4f) < 1e-5f, "airmode low: idle and roll kept");
    /* roll and pitch beyond the span: scaled, yaw dropped */
    tq[0] = 0.4f; tq[1] = 0.4f; tq[2] = 0.3f;
    mixer_quadx(0.5f, tq, 0, &m, &o);
    float mn = 1, mx = 0;
    for (int i = 0; i < 4; ++i) { if (o.t[i] < mn) mn = o.t[i]; if (o.t[i] > mx) mx = o.t[i]; }
    CHECK(o.sat_rp && o.sat_yaw && fabsf(mx - mn - 0.95f) < 1e-4f && mn >= 0.05f - 1e-5f && mx <= 1.0f + 1e-5f,
          "roll/pitch first: span %.3f, yaw dropped", mx - mn);
    /* yaw gets what is left */
    tq[0] = 0.3f; tq[1] = 0; tq[2] = 0.4f;
    mixer_quadx(0.5f, tq, 0, &m, &o);
    mn = 1; mx = 0;
    for (int i = 0; i < 4; ++i) { if (o.t[i] < mn) mn = o.t[i]; if (o.t[i] > mx) mx = o.t[i]; }
    CHECK(!o.sat_rp && o.sat_yaw && mx - mn <= 0.95f + 1e-4f && fabsf((o.t[2] + o.t[3]) - (o.t[0] + o.t[1]) - 1.2f) < 1e-4f,
          "yaw limited, roll intact (span %.3f)", mx - mn);
    /* thrust curve */
    CHECK(fabsf(mixer_curve_inv(0.25f, 1.0f) - 0.5f) < 1e-6f, "quadratic curve: t 0.25 -> duty 0.5");
    CHECK(fabsf(mixer_curve_inv(0.25f, 0.0f) - 0.25f) < 1e-6f, "linear curve");
    const float u = mixer_curve_inv(0.4f, 0.5f);
    CHECK(fabsf(0.5f * u * u + 0.5f * u - 0.4f) < 1e-5f, "theta 0.5: f(f^-1(t)) = t");
}

static void test_filters(void)
{
    biquad_t lp, nt;
    biquad_lowpass(&lp, 90.0f, 960.0f);
    biquad_notch(&nt, 200.0f, 3.0f, 960.0f);
    float amp_lp = 0, amp_nt = 0, amp_dc = 0;
    for (int k = 0; k < 4000; ++k) {
        const float x = sinf(2 * 3.14159265f * 400.0f * k / 960.0f);
        const float y = biquad_step(&lp, x);
        const float z = biquad_step(&nt, sinf(2 * 3.14159265f * 200.0f * k / 960.0f));
        if (k > 2000) { if (fabsf(y) > amp_lp) amp_lp = fabsf(y); if (fabsf(z) > amp_nt) amp_nt = fabsf(z); }
    }
    biquad_reset(&lp, 1.0f);
    for (int k = 0; k < 50; ++k) amp_dc = biquad_step(&lp, 1.0f);
    printf("  gyro LPF 90 Hz at 400 Hz: %.3f; notch 200 Hz: %.3f\n", amp_lp, amp_nt);
    CHECK(amp_lp < 0.1f, "LPF attenuates 400 Hz (%.3f)", amp_lp);
    CHECK(amp_nt < 0.05f, "notch removes 200 Hz (%.3f)", amp_nt);
    CHECK(fabsf(amp_dc - 1.0f) < 1e-4f, "reset: steady at the input");
}

int main(void)
{
    srand(1);
    test_mixer();
    test_filters();
    test_angle_steps(4.2f, "4.2 V");
    test_heading_independent();
    test_acro();
    test_yaw();
    test_disturbance();
    test_battery_comp();
    printf("ctrl: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
