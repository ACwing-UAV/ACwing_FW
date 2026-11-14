/**
 * Host tests of the hover-thrust estimate (ADR 0010 §2, Validation 1): hover_est + hover_track with the vertical
 * loop (alt_ctrl) on a 1-D vehicle. Thrust in normalised units through a motor lag (40 ms); the mixer battery
 * compensation uses the cell voltage filtered 0.5 s, the motors see the true one; accelerometer noise 2 m/s^2
 * (1-sigma), low-passed at 10 Hz as in app_main.c. Steps of 4 ms, as fc_service.
 */
#include "hover_est.h"
#include "hover_track.h"
#include "alt_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static float gauss(void)
{
    float s = 0.0f;
    for (int i = 0; i < 12; ++i) s += (float)rand() / (float)RAND_MAX;
    return s - 6.0f;
}

typedef struct {
    float h, vz, thrust_n;            /* thrust_n: normalised thrust actually produced (after the motor lag) */
    float mass, mass0, th_true0;      /* th_true = th_true0 * mass / mass0                                     */
    float v_true, v_filt;             /* cell voltage: true and the mixer's filtered copy                       */
    float a_lp;
    alt_ctrl_t ac;
    float h_sp, vz_cmd; int climb;    /* climb: follow vz_cmd instead of h_sp                                   */
} veh_t;

static float th_true(const veh_t *v) { return v->th_true0 * v->mass / v->mass0; }

static void veh_init(veh_t *v)
{
    memset(v, 0, sizeof *v);
    v->h = 1.0f; v->mass = v->mass0 = 0.075f; v->th_true0 = 0.51f;
    v->v_true = v->v_filt = 4.2f; v->thrust_n = 0.51f; v->h_sp = 1.0f;
    alt_ctrl_init(&v->ac, 0.51f);
}

/* one 4 ms step: loop, mixer + battery, plant, estimate */
static void veh_step(veh_t *v, uint32_t now)
{
    const float dt = 0.004f, g = 9.80665f;
    v->ac.hover = hover_track_value();
    const float vz_sp = v->climb ? v->vz_cmd : alt_ctrl_vz_sp(&v->ac, v->h_sp, v->h, 2.0f, 0.7f);
    const float t_cmd = alt_ctrl_step(&v->ac, vz_sp, v->vz, 1.0f, dt);
    /* the mixer compensates with the filtered voltage, the motors see the true one: thrust ~ (duty V)^2 */
    const float k = v->v_true / v->v_filt;
    const float t_phys = t_cmd * k * k;
    v->thrust_n += (t_phys - v->thrust_n) * dt / 0.040f;
    const float a = g * (v->thrust_n / th_true(v) - 1.0f) - 0.3f * v->vz;
    v->vz += a * dt; v->h += v->vz * dt;
    v->v_filt += (v->v_true - v->v_filt) * dt / 0.5f;
    v->a_lp += 0.2f * ((a + 2.0f * gauss()) - v->a_lp);         /* 250 Hz, ~10 Hz low-pass */
    hover_gate_t gt = { 1u, 0u, v->a_lp, t_cmd, 1.0f, v->vz };
    (void)hover_track_step(now, 1u, &gt);
}

static uint32_t s_now;
static void run(veh_t *v, float sec) { for (int k = 0; k < (int)(sec / 0.004f + 0.5f); ++k) { s_now += 4u; veh_step(v, s_now); } }

/* time until |th - target| < tol and stays there (within the window) */
static float settle_time(veh_t *v, float sec, float tol)
{
    float last_out = 0.0f;
    for (int k = 0; k < (int)(sec / 0.004f + 0.5f); ++k) {
        s_now += 4u; veh_step(v, s_now);
        if (fabsf(hover_track_value() - th_true(v)) > tol * th_true(v)) last_out = (float)(k + 1) * 0.004f;
    }
    return last_out;
}

static void test_seed(float factor)
{
    veh_t v; veh_init(&v);
    hover_track_init();
    hover_track_arm(0.51f * factor);
    const float t = settle_time(&v, 6.0f, 0.03f);
    printf("  seed %+3.0f %%: within 3 %% after %.2f s, estimate %.4f (true 0.51)\n", (factor - 1) * 100, t, hover_track_value());
    CHECK(t < 2.0f, "seed %+.0f %%: converges within 3 %% in < 2 s (%.2f s)", (factor - 1) * 100, t);
}

static void test_battery_sag(void)
{
    veh_t v; veh_init(&v);
    hover_track_init(); hover_track_arm(0.51f);
    run(&v, 5.0f);
    float worst = 0.0f;
    for (int s = 0; s < 300; ++s) {                       /* 4.2 -> 3.5 V in 300 s */
        v.v_true = 4.2f - 0.7f * (float)s / 300.0f;
        run(&v, 1.0f);
        const float e = fabsf(hover_track_value() - 0.51f) / 0.51f;
        if (e > worst) worst = e;
    }
    printf("  battery 4.2 -> 3.5 V in 300 s: worst estimate error %.2f %%\n", worst * 100);
    CHECK(worst < 0.03f, "battery sag: no drift (worst %.2f %%)", worst * 100);
}

static void test_mass_step(void)
{
    veh_t v; veh_init(&v);
    hover_track_init(); hover_track_arm(0.51f);
    run(&v, 10.0f);
    v.mass += 0.015f;                                     /* tray change / payload */
    const float t = settle_time(&v, 8.0f, 0.03f);
    printf("  mass +15 g (true %.3f): within 3 %% after %.2f s, estimate %.4f\n", th_true(&v), t, hover_track_value());
    CHECK(t < 3.0f, "mass step tracked in < 3 s (%.2f s)", t);
}

static void test_climb(void)
{
    veh_t v; veh_init(&v);
    hover_track_init(); hover_track_arm(0.51f);
    run(&v, 10.0f);
    v.climb = 1; v.vz_cmd = 1.0f;
    float worst = 0.0f;
    for (int k = 0; k < 5 * 250; ++k) {
        s_now += 4u; veh_step(&v, s_now);
        if (k > 250) { const float e = fabsf(hover_track_value() - 0.51f) / 0.51f; if (e > worst) worst = e; }
    }
    printf("  climb at 1 m/s for 5 s: worst estimate error %.2f %% (no update above %.1f m/s)\n", worst * 100, HT_MAX_VZ);
    CHECK(worst < 0.02f, "climb: bias < 2 %% (%.2f %%)", worst * 100);
    /* the gate itself: tilt beyond 30 deg, a saturated collective, not airborne -> no update */
    const float before = hover_track_value();
    for (int k = 0; k < 100; ++k) {
        s_now += 20u;
        hover_gate_t a = { 1u, 0u, 5.0f, 0.51f, 0.80f, 0.0f }, b = { 1u, 1u, 5.0f, 0.51f, 1.0f, 0.0f }, c = { 0u, 0u, 5.0f, 0.51f, 1.0f, 0.0f };
        (void)hover_track_step(s_now, 1u, k % 3 == 0 ? &a : (k % 3 == 1 ? &b : &c));
    }
    CHECK(fabsf(hover_track_value() - before) < 1e-6f, "tilted, saturated or on the ground: no update");
}

static void test_gate_and_memory(void)
{
    /* an impact: a few samples of 30 m/s^2 are rejected, the estimate does not jump */
    veh_t v; veh_init(&v);
    hover_track_init(); hover_track_arm(0.51f);
    run(&v, 5.0f);
    const float before = hover_track_value();
    for (int k = 0; k < 5; ++k) { s_now += 20u; hover_gate_t g = { 1u, 0u, 30.0f, 0.51f, 1.0f, 0.0f }; (void)hover_track_step(s_now, 1u, &g); }
    CHECK(fabsf(hover_track_value() - before) < 0.01f, "impact rejected (%.4f -> %.4f)", before, hover_track_value());
    /* a flight of 25 s seeds the next arm; a short one does not */
    run(&v, 25.0f);
    const float learned = hover_track_value();
    hover_track_disarm(25000u);
    hover_track_arm(0.60f);
    CHECK(fabsf(hover_track_value() - learned) < 1e-6f, "the next flight starts from the learned value, not the parameter");
    hover_track_init();
    hover_track_arm(0.60f);
    CHECK(fabsf(hover_track_value() - 0.60f) < 1e-6f, "power-on: the parameter");
    hover_track_disarm(5000u);
    hover_track_arm(0.58f);
    CHECK(fabsf(hover_track_value() - 0.58f) < 1e-6f, "short flight: not kept");
    /* plausibility warning: once, only after the estimate has settled */
    hover_track_arm(0.84f);
    int warns = 0;
    for (int k = 0; k < 200; ++k) { s_now += 20u; hover_gate_t g = { 1u, 0u, 0.0f, 0.84f, 1.0f, 0.0f }; warns += hover_track_step(s_now, 1u, &g); }
    CHECK(warns == 1, "abnormal hover thrust: one warning (%d)", warns);
}

int main(void)
{
    srand(2);
    test_seed(1.3f);
    test_seed(0.7f);
    test_battery_sag();
    test_mass_step();
    test_climb();
    test_gate_and_memory();
    printf("hover: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
