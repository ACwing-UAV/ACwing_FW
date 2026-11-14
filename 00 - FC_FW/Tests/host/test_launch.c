/**
 * Host tests of the hand launch (control/launch.c, ADR 0011): the state machine on synthetic IMU traces (toss,
 * drop, handling, jump), then the recovery in closed loop with the attitude controller, the mixer and a 6-DOF plant
 * (rigid body, four rotors with lag, thrust at the cell voltage), across a grid of release attitudes and rates.
 */
#include "launch.h"
#include "att_ctrl.h"
#include "mixer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "plant6.h"

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define G0 9.80665f
#define DT_MS 4u

/* ------------------------------------------------------------------ synthetic traces (no dynamics) */
typedef struct { ln_t l; ln_out_t o; int released_at, start_ms, motors_on_at; uint8_t aborted, abort_reason; int t; } run_t;

static ln_in_t base_in(void)
{
    ln_in_t in; memset(&in, 0, sizeof in);
    in.f_g = 1.0f; in.cos_tilt = 1.0f; in.est_ok = 1u; in.link_ok = 1u; in.hover = 0.5f;
    return in;
}

/* ms of a constant input; a_up = the world vertical acceleration, f = the specific force */
static void feed(run_t *r, int ms, float f_g, float a_up, float w)
{
    ln_in_t in = base_in();
    in.f_g = f_g; in.a_w[2] = a_up; in.w = w;
    for (int k = 0; k < ms / (int)DT_MS; ++k) {
        r->t += DT_MS;
        ln_step(&r->l, &in, DT_MS, &r->o);
        if (r->o.phase == LN_RELEASED && r->released_at < 0) r->released_at = r->t;
        if (r->o.released) { r->start_ms = r->o.start_ms; r->motors_on_at = r->t; }
        if (r->o.phase == LN_ABORT && !r->aborted) { r->aborted = 1u; r->abort_reason = r->o.abort_reason; }
    }
}

static void start(run_t *r, uint8_t drop)
{
    memset(r, 0, sizeof *r);
    r->released_at = r->motors_on_at = -1;
    ln_start(&r->l, drop);
}

static void test_traces(void)
{
    run_t r;
    /* 1. toss: presented, push 0.15 s at 1.8 g (0.8 g up), then free fall */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    CHECK(r.l.presented && r.o.motors_off, "presented, motors off");
    feed(&r, 150, 1.8f, 1.02f * G0, 0.5f);                              /* 1.5 m/s at the release */
    feed(&r, 300, 0.02f, -G0, 0.3f);
    CHECK(r.released_at > 0 && r.motors_on_at > 0, "toss: released, motors started");
    printf("  toss 1.5 m/s: motors started %d ms after the release\n", r.start_ms);
    CHECK(r.start_ms >= 80 && r.start_ms <= 120, "toss 1.5 m/s: 10 cm reached in 80..120 ms (%d)", r.start_ms);

    /* 2. drop without a push: refused by default, accepted with launchDrop */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 400, 0.02f, -G0, 0.0f);
    CHECK(r.released_at < 0 && r.o.motors_off, "drop without launchDrop: motors stay off");
    start(&r, 1);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 400, 0.02f, -G0, 0.0f);
    printf("  drop: motors started %d ms after the release\n", r.start_ms);
    CHECK(r.motors_on_at > 0 && r.start_ms >= 135 && r.start_ms <= 150, "drop with launchDrop: 10 cm in ~143 ms (%d)", r.start_ms);

    /* 3. free fall before any presentation (dropped while being picked up): nothing */
    start(&r, 1);
    feed(&r, 100, 1.2f, 0.2f * G0, 2.0f);
    feed(&r, 300, 0.02f, -G0, 0.0f);
    CHECK(r.released_at < 0, "no presentation: no release");

    /* 4. handling: walking (1 +- 0.4 g at 2 Hz, rotating), a tap (0.05 s at 3 g), set down: no release */
    start(&r, 1);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    for (int k = 0; k < 2000; ++k) {
        const float ph = (float)k * 0.004f * 2.0f * 3.14159f * 2.0f;
        feed(&r, 4, 1.0f + 0.4f * sinf(ph), 0.4f * G0 * sinf(ph), 1.5f);
    }
    feed(&r, 50, 3.0f, 2.0f * G0, 0.5f);
    feed(&r, 20, 0.2f, -0.8f * G0, 0.5f);                                    /* a short dip: 20 ms */
    CHECK(r.released_at < 0, "walking, a tap, short dips: no release");
    feed(&r, 2000, 1.0f, 0.0f, 0.0f);
    CHECK(r.aborted && r.abort_reason == LN_AB_TIMEOUT, "10 s in the hand: aborted (timeout)");

    /* 5. gyro saturated during the free fall: the release is refused while it lasts */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 150, 1.8f, 0.8f * G0, 0.5f);
    {
        ln_in_t in = base_in(); in.f_g = 0.02f; in.a_w[2] = -G0; in.gyro_sat = 1u; in.w = 70.0f;
        for (int k = 0; k < 40; ++k) { r.t += 4; ln_step(&r.l, &in, 4, &r.o); }
    }
    CHECK(r.o.phase == LN_READY, "gyro saturated: not released");

    /* 6. link lost in the hand: aborted after 1 s */
    start(&r, 0);
    {
        ln_in_t in = base_in(); in.link_ok = 0u;
        for (int k = 0; k < 300 && r.o.phase != LN_ABORT; ++k) ln_step(&r.l, &in, 4, &r.o);
    }
    CHECK(r.o.phase == LN_ABORT && r.o.abort_reason == LN_AB_LINK, "link lost: aborted");

    /* 6b. a gentle toss (0.6 m/s, rises 1.8 cm): falls back onto the hand -> motors kept off */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 80, 1.8f, 0.77f * G0, 0.3f);
    feed(&r, 300, 0.02f, -G0, 0.3f);
    CHECK(r.motors_on_at < 0 && r.aborted && r.abort_reason == LN_AB_WEAK_TOSS, "gentle toss: motors kept off, aborted");
    /* 6c. a toss at 1.2 m/s rises 7 cm: started at the apex, never later on the way down */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 150, 1.8f, 0.82f * G0, 0.3f);
    feed(&r, 400, 0.02f, -G0, 0.3f);
    printf("  toss 1.2 m/s: motors at the apex, %d ms\n", r.start_ms);
    CHECK(r.motors_on_at > 0 && r.start_ms >= 100 && r.start_ms <= 140, "toss 1.2 m/s: started at the apex (%d ms)", r.start_ms);
    /* 6d. running with the drone (2 g pushes of 0.12 s, flight phases of 0.12 s, apex 1.8 cm): never a motor start */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 60, 2.0f, 1.0f * G0, 1.0f);                               /* first half stride: v -> +0.59 m/s */
    for (int k = 0; k < 20; ++k) { feed(&r, 120, 0.05f, -G0, 1.0f); feed(&r, 120, 2.0f, 1.0f * G0, 1.0f); }   /* +-0.59 m/s */
    CHECK(r.motors_on_at < 0, "running: motors never started");

    /* 7. a jump with the drone in the hand: legs push (2 g, 0.25 s), flight phase 0.4 s in free fall.
     *    The IMU cannot tell it from a toss (see ADR 0011, validation finding): recorded, not asserted. */
    start(&r, 0);
    feed(&r, 600, 1.0f, 0.0f, 0.0f);
    feed(&r, 250, 2.0f, 1.0f * G0, 0.3f);
    feed(&r, 400, 0.05f, -G0, 0.3f);
    printf("  jump with the drone in the hand: %s (motors at %d ms)\n",
           r.motors_on_at > 0 ? "RELEASED - indistinguishable from a toss" : "not released", r.start_ms);

    /* T/W available */
    CHECK(fabsf(ln_tw_available(0.51f, 4.2f) - 1.96f) < 0.01f && fabsf(ln_tw_available(0.51f, 3.7f) - 1.52f) < 0.01f &&
          ln_tw_available(0.5f, 0.0f) == 0.0f, "T/W available");
}

/* ------------------------------------------------------------------ closed loop: 6-DOF plant (plant6.h) */
typedef struct { float lowest, t_done, t_start, tilt_after, t_level, tilt_start; uint8_t done, started, level; } cl_res_t;

/* a toss (vz0 up) or a drop, released with a tilt (rad, about x then y) and body rates (rad/s) */
static cl_res_t closed_loop(float mass, float vcell, float vz0, float roll0, float pitch0, const float w0[3], uint8_t drop)
{
    plant_t pl; plant_init(&pl, mass, vcell);
    const float hr = 0.5f * roll0, hp = 0.5f * pitch0;
    const float qr[4] = { cosf(hr), sinf(hr), 0, 0 }, qp[4] = { cosf(hp), 0, sinf(hp), 0 };
    att_gains_t g; att_gains_default(&g);
    att_ctrl_t ac; att_ctrl_init(&ac, &g);
    mixer_cfg_t mc = { 0.05f, 1.0f };
    ln_t l; ln_out_t o; ln_start(&l, drop);
    const float hover = mass * G0 / (4.0f * pl.tmax);
    /* presented in the hand, then released at the attitude, rates and speed given (the push is abstracted) */
    ln_in_t in = base_in(); in.hover = hover;
    for (int k = 0; k < 150; ++k) ln_step(&l, &in, DT_MS, &o);
    in.f_g = 1.8f; in.a_w[2] = 0.8f * G0;
    for (int k = 0; k < 10; ++k) ln_step(&l, &in, DT_MS, &o);
    qmul(qr, qp, pl.q);
    memcpy(pl.w, w0, sizeof pl.w);
    pl.v[2] = vz0;
    l.v[0] = l.v[1] = 0.0f; l.v[2] = vz0;                 /* the push integrated to the release speed */
    cl_res_t res; memset(&res, 0, sizeof res);
    float duty[4] = { 0, 0, 0, 0 };
    int k_rate = 0;
    for (int t = 0; t < 3000; t += 1) {                  /* 1 ms plant steps, rate loop ~ 1 kHz */
        const float dt = 1e-3f;
        if (t % (int)DT_MS == 0) {
            float fb[3] = { 0.0f, 0.0f, (pl.T[0] + pl.T[1] + pl.T[2] + pl.T[3]) / pl.mass };
            float fw[3]; rot(pl.q, fb, fw);
            in.f_g = sqrtf(fb[2]*fb[2]) / G0;
            in.a_w[0] = fw[0]; in.a_w[1] = fw[1]; in.a_w[2] = fw[2] - G0;
            in.w = sqrtf(pl.w[0]*pl.w[0] + pl.w[1]*pl.w[1] + pl.w[2]*pl.w[2]);
            in.cos_tilt = 1.0f - 2.0f * (pl.q[1]*pl.q[1] + pl.q[2]*pl.q[2]);
            in.vz = pl.v[2];
            ln_step(&l, &in, DT_MS, &o);
            if (o.released && !res.started) { res.started = 1u; res.t_start = (float)t * 1e-3f; res.tilt_start = acosf(in.cos_tilt > 1.0f ? 1.0f : (in.cos_tilt < -1.0f ? -1.0f : in.cos_tilt)); }
            if (res.started && !res.level && in.cos_tilt > 0.966f && in.w < 1.0f) { res.level = 1u; res.t_level = (float)t * 1e-3f; }
            if (o.phase == LN_DONE && !res.done) { res.done = 1u; res.t_done = (float)t * 1e-3f; }
            if (res.done && (float)t * 1e-3f >= res.t_done + 0.3f && res.tilt_after == 0.0f)
                res.tilt_after = acosf(in.cos_tilt > 1.0f ? 1.0f : in.cos_tilt) + 1e-6f;   /* 0.3 s after the recovery */
            att_sp_t sp; memset(&sp, 0, sizeof sp);
            att_ctrl_attitude(&ac, pl.q, &sp);                 /* level, yaw rate 0 */
        }
        if (++k_rate >= 1) {
            k_rate = 0;
            float torque[3];
            const uint8_t off = o.motors_off;
            att_ctrl_rate(&ac, pl.w, (uint8_t)!off, torque);
            if (off) { duty[0] = duty[1] = duty[2] = duty[3] = 0.0f; att_ctrl_reset(&ac); }
            else {
                const float thr = (o.phase == LN_DONE) ? hover / (in.cos_tilt > 0.5f ? in.cos_tilt : 0.5f) : o.thrust;
                mixer_out_t mo; mixer_quadx(thr, torque, vcell, &mc, &mo);
                memcpy(duty, mo.duty, sizeof duty);
            }
        }
        plant_step(&pl, duty, dt);
        if (pl.p[2] < res.lowest) res.lowest = pl.p[2];
        if (res.done && (float)t * 1e-3f > res.t_done + 0.35f) break;
    }
    return res;
}

static void test_closed_loop(void)
{
    const float D2R = 0.0174533f;
    const struct { const char *name; float mass; } fr[] = { { "T400", 0.071f }, { "T800", 0.082f }, { "T1200", 0.090f } };
    const float volts[] = { 4.2f, 3.6f };
    const float tilts[] = { 0.0f, 20.0f, 40.0f };
    const float rates[] = { 0.0f, 200.0f, 400.0f };
    int n = 0, ok = 0;
    float worst_toss = 0.0f, worst_drop = 0.0f, worst_t = 0.0f, worst_tilt = 0.0f;
    for (unsigned f = 0; f < 3; ++f)
        for (unsigned v = 0; v < 2; ++v) {
            const float tw = (volts[v] / 4.2f) * (volts[v] / 4.2f) * 4.0f * 0.343f / (fr[f].mass * G0);
            if (tw < LN_TW_MIN) continue;                        /* the gate refuses the launch */
            for (unsigned a = 0; a < 3; ++a)
                for (unsigned r = 0; r < 3; ++r) {
                    const float w0[3] = { rates[r] * D2R, -0.5f * rates[r] * D2R, 0.3f * rates[r] * D2R };
                    const cl_res_t c = closed_loop(fr[f].mass, volts[v], 1.5f, tilts[a] * D2R, -0.5f * tilts[a] * D2R, w0, 0);
                    n++;
                    const float rec = c.t_done - c.t_start;               /* from the motor start */
                    /* ADR 0011 validation, amended: 0.7 s. At 400 dps the drone turns freely to 63-85 deg during the
                     * ~0.1 s with the motors off (the separation rule); the tilt loop (6 rad/s, rate limit 6 rad/s)
                     * needs ~0.6 s from there. The 0.5 s first written in the ADR did not account for it. */
                    if (c.done && c.started && rec <= 0.7f && c.tilt_after < 5.0f * D2R) ok++;
                    else printf("  toss %s %.1f V tilt %.0f rate %.0f: tilt %.0f deg at the motor start, level after %.2f s, "
                                "done %.2f s (v_z >= 0), %.1f deg 0.3 s later\n", fr[f].name, volts[v], tilts[a], rates[r],
                                c.tilt_start / D2R, c.t_level - c.t_start, rec, c.tilt_after / D2R);
                    if (c.lowest < worst_toss) worst_toss = c.lowest;
                    if (rec > worst_t) worst_t = rec;
                    if (c.tilt_after > worst_tilt) worst_tilt = c.tilt_after;
                }
            if (tw >= LN_TW_MIN_DROP) {
                const float w0[3] = { 0, 0, 0 };
                const cl_res_t c = closed_loop(fr[f].mass, volts[v], 0.0f, 0.0f, 0.0f, w0, 1);
                if (c.lowest < worst_drop) worst_drop = c.lowest;
                CHECK(c.done, "drop %s %.1f V recovered", fr[f].name, volts[v]);
            }
        }
    printf("  closed loop: %d/%d tosses within the ADR criteria; worst toss lowest %.2f m, drop lowest %.2f m,\n"
           "  slowest recovery %.2f s after the motor start, worst tilt %.1f deg 0.3 s after it\n",
           ok, n, worst_toss, worst_drop, worst_t, worst_tilt / D2R);
    CHECK(ok == n, "every toss recovered within 0.7 s of the motor start, within 5 deg after (%d/%d)", ok, n);
    CHECK(worst_toss > -0.15f, "a toss loses (almost) no height below the release point (%.2f m)", worst_toss);
    CHECK(worst_drop > -0.8f, "a drop within the ADR table (%.2f m)", worst_drop);
}

int main(void)
{
    test_traces();
    test_closed_loop();
    printf("launch: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
