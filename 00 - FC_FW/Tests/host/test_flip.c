/**
 * Host tests of the autonomous flip (control/flip.c, ADR 0012): the entry gate, then the manoeuvre in closed loop
 * with the attitude and rate controllers, the mixer and the 6-DOF plant (plant6.h), on the four axes, the three
 * trays and three cell voltages; the abort paths.
 */
#include "flip.h"
#include "att_ctrl.h"
#include "mixer.h"
#include "proto_v2.h"
#include "plant6.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define D2R 0.0174533f

/* ------------------------------------------------------------------ gate */
static fl_gate_in_t ok_gate(void)
{
    fl_gate_in_t g; memset(&g, 0, sizeof g);
    g.in_air = g.armed = g.auto_none = 1u; g.alt_valid = 1u; g.height = 3.0f; g.ceiling_room = 2.0f; g.tw = 1.9f;
    g.cos_tilt = 1.0f; g.batt_ok = g.link_ok = 1u; g.min_h = 2.0f; g.min_tw = 1.6f;
    return g;
}

static void test_gate(void)
{
    fl_gate_in_t g = ok_gate();
    CHECK(fl_gate(&g) == PV2_R_NONE, "gate: all fine");
    g = ok_gate(); g.in_air = 0;          CHECK(fl_gate(&g) == PV2_R_NOT_ARMED, "on the ground: NOT_ARMED");
    g = ok_gate(); g.auto_none = 0;       CHECK(fl_gate(&g) == FL_REFUSED, "during an automatic action: refused");
    g = ok_gate(); g.acro = 1;            CHECK(fl_gate(&g) == FL_REFUSED, "ACRO: refused");
    g = ok_gate(); g.beginner = 1;        CHECK(fl_gate(&g) == FL_REFUSED, "beginner: refused");
    g = ok_gate(); g.tw = 1.5f;           CHECK(fl_gate(&g) == PV2_R_THRUST_LOW, "T/W 1.5: THRUST_LOW");
    g = ok_gate(); g.height = 1.5f;       CHECK(fl_gate(&g) == PV2_R_OUT_OF_RANGE, "1.5 m: OUT_OF_RANGE");
    g = ok_gate(); g.ceiling_room = 0.4f; CHECK(fl_gate(&g) == PV2_R_OUT_OF_RANGE, "0.4 m to the ceiling: OUT_OF_RANGE");
    g = ok_gate(); g.cos_tilt = 0.97f;    CHECK(fl_gate(&g) == PV2_R_NOT_STILL, "14 deg of tilt: NOT_STILL");
    g = ok_gate(); g.v_h = 1.2f;          CHECK(fl_gate(&g) == PV2_R_NOT_STILL, "moving 1.2 m/s: NOT_STILL");
    g = ok_gate(); g.batt_ok = 0;         CHECK(fl_gate(&g) == PV2_R_BATTERY_LOW, "battery low: BATTERY_LOW");
    g = ok_gate(); g.alt_valid = 0;       CHECK(fl_gate(&g) == PV2_R_SENSOR_MISSING, "no altitude: SENSOR_MISSING");
}

/* ------------------------------------------------------------------ closed loop */
typedef struct { float lowest, highest, t_done, tilt_after, turned; uint8_t done, aborted, cut; } fl_res_t;

typedef enum { EV_NONE, EV_STICK, EV_SAT } ev_t;

static fl_res_t run_flip(float mass, float vcell, uint8_t axis, ev_t ev)
{
    plant_t pl; plant_init(&pl, mass, vcell);
    const float k = vcell / 4.2f;
    const float hover = mass * G0 / (4.0f * pl.tmax * k * k);        /* normalised at 4.2 V */
    const float th_hover = mass * G0 / 4.0f;
    for (int i = 0; i < 4; ++i) pl.T[i] = th_hover;                   /* hovering */
    att_gains_t g; att_gains_default(&g);
    att_ctrl_t ac; att_ctrl_init(&ac, &g);
    mixer_cfg_t mc = { 0.05f, 1.0f };
    fl_t f; fl_out_t o; memset(&o, 0, sizeof o);
    fl_start(&f, axis, 1400.0f * D2R);
    const float hover_n = mass * G0 / (4.0f * pl.tmax);               /* the flight core's estimate (at 4.2 V) */
    fl_res_t res; memset(&res, 0, sizeof res);
    float duty[4];
    float coll = hover_n;
    for (int t = 0; t < 4000; ++t) {
        if (t % 4 == 0) {
            fl_in_t in; memset(&in, 0, sizeof in);
            memcpy(in.w, pl.w, sizeof in.w);
            in.cos_tilt = 1.0f - 2.0f * (pl.q[1]*pl.q[1] + pl.q[2]*pl.q[2]);
            in.vz = pl.v[2]; in.hover = hover_n; in.link_ok = 1u; in.est_ok = 1u;
            if (f.phase == FL_ROTATE && f.t_phase > 100u) { if (ev == EV_STICK) in.stick = 0.8f; if (ev == EV_SAT) in.gyro_sat = 1u; }
            if (f.phase != FL_DONE && f.phase != FL_ABORT) fl_step(&f, &in, 4u, &o);
            if (o.phase == FL_DONE && !res.done) { res.done = 1u; res.t_done = (float)t * 1e-3f; res.cut = o.abort_reason; }
            if (o.phase == FL_ABORT) res.aborted = 1u;
            att_sp_t sp; memset(&sp, 0, sizeof sp);
            if (!res.done && o.rate_mode) { sp.acro = 1u; sp.roll_rate = o.rate[0]; sp.pitch_rate = o.rate[1]; sp.yaw_rate = o.rate[2]; }
            att_ctrl_attitude(&ac, pl.q, &sp);
            /* after DONE: hover on the estimate, as the vertical loop would start from */
            coll = res.done ? hover_n / (in.cos_tilt > 0.5f ? in.cos_tilt : 0.5f) : o.collective;
            if (res.done && (float)t * 1e-3f >= res.t_done + 0.3f && res.tilt_after == 0.0f)
                res.tilt_after = acosf(in.cos_tilt > 1.0f ? 1.0f : in.cos_tilt) + 1e-6f;
        }
        float torque[3];
        att_ctrl_rate(&ac, pl.w, 1u, torque);
        mixer_out_t mo; mixer_quadx(coll, torque, vcell, &mc, &mo);
        memcpy(duty, mo.duty, sizeof duty);
        plant_step(&pl, duty, 1e-3f);
        {   /* the rotation actually done by the plant, about the flip axis */
            const int ai = (axis <= FL_ROLL_LEFT) ? 0 : 1;
            const float sg = (axis == FL_ROLL_RIGHT || axis == FL_PITCH_FWD) ? 1.0f : -1.0f;
            res.turned += sg * pl.w[ai] * 1e-3f;
        }
        if (pl.p[2] < res.lowest) res.lowest = pl.p[2];
        if (pl.p[2] > res.highest) res.highest = pl.p[2];
        if (res.tilt_after > 0.0f) break;
    }
    (void)hover;
    return res;
}

static void test_closed_loop(void)
{
    /* "edge": the mass that puts T/W at 1.62 on a full battery, just above the gate */
    const struct { const char *name; float mass; } fr[] = { { "T400", 0.071f }, { "T800", 0.082f }, { "T1200", 0.090f },
                                                            { "edge", 0.0863f } };
    const float volts[] = { 4.2f, 3.9f, 3.6f };
    const char *axn[] = { "roll right", "roll left", "pitch fwd", "pitch back" };
    int n = 0, ok = 0;
    float worst_low = 0.0f, worst_t = 0.0f, worst_tilt = 0.0f;
    printf("  flip, closed loop (lowest / highest point, duration, tilt 0.3 s after):\n");
    for (unsigned a = 0; a < 4; ++a)
        for (unsigned v = 0; v < 3; ++v) {
            const float tw = (volts[v] / 4.2f) * (volts[v] / 4.2f) * 4.0f * 0.343f / (fr[a].mass * G0);
            if (tw < 1.6f) continue;                                  /* refused by the gate (flipMinTw) */
            for (uint8_t ax = 0; ax < 4; ++ax) {
                const fl_res_t r = run_flip(fr[a].mass, volts[v], ax, EV_NONE);
                n++;
                const uint8_t pass = (uint8_t)(r.done && !r.aborted && !r.cut && r.t_done < 1.5f && r.tilt_after < 5.0f * D2R &&
                                               fabsf(r.turned - 2.0f * 3.14159f) < 15.0f * D2R);
                if (pass) ok++;
                if (ax == 0 || !pass)
                    printf("   %-5s %.1f V T/W %.2f %-10s: turned %.0f deg, %+.2f / %+.2f m, %.2f s, %.1f deg%s\n", fr[a].name,
                           volts[v], tw, axn[ax], r.turned / D2R, r.lowest, r.highest, r.t_done, r.tilt_after / D2R,
                           pass ? "" : "  <- FAIL");
                if (r.lowest < worst_low) worst_low = r.lowest;
                if (r.t_done > worst_t) worst_t = r.t_done;
                if (r.tilt_after > worst_tilt) worst_tilt = r.tilt_after;
            }
        }
    printf("  %d/%d flips: worst lowest %.2f m, slowest %.2f s, worst tilt %.1f deg\n", ok, n, worst_low, worst_t, worst_tilt / D2R);
    CHECK(ok == n, "every flip within the gate: one full turn (+-15 deg), level within 5 deg, < 1.5 s (%d/%d)", ok, n);
    CHECK(worst_low > -0.4f, "lowest point above -0.4 m (1-D estimate -0.27 m at T/W 1.6, + 0.1 m) (%.2f)", worst_low);

    /* aborts: a stick or the gyro saturated during the rotation -> recovered, reported */
    fl_res_t r = run_flip(0.071f, 4.2f, FL_ROLL_RIGHT, EV_STICK);
    CHECK(r.done && r.cut == FL_AB_STICK && r.tilt_after < 5.0f * D2R, "stick during the rotation: cut, recovered (%.1f deg)", r.tilt_after / D2R);
    r = run_flip(0.071f, 4.2f, FL_PITCH_FWD, EV_SAT);
    CHECK(r.done && r.cut == FL_AB_GYRO_SAT && r.tilt_after < 5.0f * D2R, "gyro saturated: cut, recovered");
}

static void test_phases(void)
{
    fl_t f; fl_out_t o; fl_in_t in; memset(&in, 0, sizeof in);
    in.cos_tilt = 1.0f; in.hover = 0.5f; in.link_ok = in.est_ok = 1u;
    fl_start(&f, FL_ROLL_LEFT, 24.0f);
    fl_step(&f, &in, 4u, &o);
    CHECK(o.phase == FL_CLIMB && o.collective == 1.0f && !o.rate_mode && o.vector_hold, "climb: full thrust, level, vector hold");
    for (int k = 0; k < 50; ++k) fl_step(&f, &in, 4u, &o);
    CHECK(o.phase == FL_ROTATE && o.rate_mode && o.rate[0] < -20.0f && o.collective == FL_ROT_COLLECTIVE, "roll left: negative roll rate");
    in.est_ok = 0u; fl_step(&f, &in, 4u, &o);
    CHECK(o.phase == FL_ABORT && o.abort_reason == FL_AB_ESTIMATE && !o.vector_hold, "estimate lost: abort");
    /* never level: aborted after 1 s of recovery (the caller lands) */
    fl_start(&f, FL_PITCH_BACK, 24.0f);
    memset(&o, 0, sizeof o);
    in.est_ok = 1u; in.cos_tilt = 0.5f; in.stick = 0.9f;
    for (int k = 0; k < 400 && o.phase != FL_ABORT; ++k) fl_step(&f, &in, 4u, &o);
    CHECK(o.phase == FL_ABORT && o.abort_reason == FL_AB_NOT_LEVEL, "not level within 1 s: abort");
}

int main(void)
{
    test_gate();
    test_phases();
    test_closed_loop();
    printf("flip: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
