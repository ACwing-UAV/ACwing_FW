/*
 * Cold-start convergence of the attitude IEKF (HANDOVER: ACwing IEKF after the
 * c4dynamics notebook review, items 1, 2 and 6).
 *
 * The production pipeline (est_pipeline.c with app_est_config(): every
 * app_config.h value, stillness detector, NIS inflation, lockout protection,
 * tilt-compensated compass, mag latency) runs on simulated LSM6DSV16X + LIS2MDL
 * data. After the static calibration the attitude is re-initialised with an
 * error of 30, 90, 150 or 179 degrees about several axes, with an HONEST P0
 * (iekf_reset_attitude), on a static and on a moving (rotating) recording.
 * Reported: time to converge below 5 deg (and stay there), lockouts, final
 * error; with and without the tangential innovation.
 *
 * Also run with the production p0_att (2 deg, overconfident) to show what a
 * re-init without the honest P0 does.
 */
#include "est_pipeline.h"
#include "est_config.h"
#include "so3.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define FS        960.0
#define DT        (1.0 / FS)
#define MAG_HZ    100.0
#define D2R       0.017453292519943295
#define CONV_DEG  5.0

/* ---------------------------------------------------------------- noise */
static uint64_t s_rng = 88172645463325252ull;
static double urand(void) { s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17; return (double)(s_rng >> 11) * (1.0 / 9007199254740992.0); }
static double grand(void) { double u1 = urand(), u2 = urand(); if (u1 < 1e-300) u1 = 1e-300; return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); }

/* ---------------------------------------------------------------- double SO(3) */
static void dexp(const double w[3], double R[9])
{
    const double th = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    double a = 1.0, b = 0.5;
    if (th > 1e-9) { a = sin(th) / th; b = (1.0 - cos(th)) / (th * th); }
    const double K[9] = { 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0 };
    double K2[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
        double s = 0; for (int k = 0; k < 3; ++k) s += K[3*r+k] * K[3*k+c]; K2[3*r+c] = s;
    }
    for (int i = 0; i < 9; ++i) R[i] = (i % 4 == 0 ? 1.0 : 0.0) + a * K[i] + b * K2[i];
}
static void dmul(const double A[9], const double B[9], double C[9])
{
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
        double s = 0; for (int k = 0; k < 3; ++k) s += A[3*r+k] * B[3*k+c]; C[3*r+c] = s;
    }
}
static void dtvec(const double R[9], const double v[3], double o[3])   /* R' v */
{
    for (int i = 0; i < 3; ++i) o[i] = R[i] * v[0] + R[3+i] * v[1] + R[6+i] * v[2];
}
/* angle of R_true' R_est, degrees */
static double att_err_deg(const double Rt[9], const float Re[9])
{
    double E[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
        double s = 0; for (int k = 0; k < 3; ++k) s += Rt[3*k+r] * (double)Re[3*k+c]; E[3*r+c] = s;
    }
    const double vx = E[7] - E[5], vy = E[2] - E[6], vz = E[3] - E[1];
    return atan2(0.5 * sqrt(vx*vx + vy*vy + vz*vz), 0.5 * (E[0] + E[4] + E[8] - 1.0)) / D2R;
}

/* ---------------------------------------------------------------- scenario */
typedef struct {
    double conv_s;       /* time below CONV_DEG for good (< 0 = never) */
    double final_deg, max_after_conv;
    unsigned lockouts;
} result_t;

static const double G_W[3] = { 0, 0, 1 };                 /* g, world NWU (up)          */
static double       s_m_w[3];                             /* uT, world: dip 58 deg       */
static const double BIAS[3] = { 0.5 * D2R, -0.3 * D2R, 0.4 * D2R };

static void omega_of(int moving, double t, double w[3])
{
    if (!moving) { w[0] = w[1] = w[2] = 0; return; }
    /* a hand moving the drone around: up to ~1 rad/s per axis */
    w[0] = 0.8 * sin(2 * M_PI * 0.23 * t);
    w[1] = 0.6 * sin(2 * M_PI * 0.17 * t + 1.0);
    w[2] = 1.0 * sin(2 * M_PI * 0.11 * t + 2.0);
}

static result_t run(int moving, double err_deg, const double axis_in[3], int tangent, int mode)
{
    s_rng = 88172645463325252ull;                          /* same noise for every run   */
    est_config_t ec;
    app_est_config(&ec);
    const float I3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    memcpy(ec.mag_map, I3, sizeof I3);
    ec.ekf.tangent_innov = (uint8_t)tangent;
    static est_t e;
    est_init(&e, &ec, NULL, NULL, NULL);

    /* truth: starts tilted, as a drone on a slope */
    double R[9];
    { const double w0[3] = { 0.20, -0.15, 1.0 }; dexp(w0, R); }
    double hist[16][9];                                    /* R of the last steps (mag age) */
    int nh = 0;
    const int lat = (int)lround(ec.mag_latency_s * FS);    /* mean age of a mag sample     */

    const double t_cal = 3.0, t_err = 4.0, t_end = t_err + 10.0;
    double next_mag = 0.0;
    int injected = 0;
    result_t res = { -1.0, 0, 0, 0 };
    double last_above = 0.0;

    for (long k = 0; k * DT < t_end; ++k) {
        const double t = k * DT;
        const int mov = moving && t >= t_err;              /* still during the calibration */
        double w[3];
        omega_of(mov, t - t_err, w);
        memcpy(hist[nh % 16], R, sizeof R); nh++;

        imu_event_t ev[3];
        int n = 0;
        ev[n].type = IMU_EV_GYRO;
        for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)(w[i] + BIAS[i] + 0.001 * grand());
        n++;
        double a[3];
        dtvec(R, G_W, a);
        ev[n].type = IMU_EV_ACC;
        for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)(a[i] + 0.002 * grand());
        n++;
        if (t >= next_mag) {
            next_mag += 1.0 / MAG_HZ;
            const double *Rm = hist[(nh - 1 - (lat < nh ? lat : nh - 1) + 16) % 16];
            double m[3];
            dtvec(Rm, s_m_w, m);
            ev[n].type = IMU_EV_MAG;
            for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)(m[i] + 0.15 * grand());
            n++;
        }
        est_process(&e, ev, (size_t)n, (float)DT);

        /* truth propagation */
        { double dw[3] = { w[0]*DT, w[1]*DT, w[2]*DT }, E[9], Rn[9]; dexp(dw, E); dmul(R, E, Rn); memcpy(R, Rn, sizeof R); }

        if (!injected && t >= t_err) {
            if (!e.running) { printf("  calibration did not finish by %.1f s\n", t_cal); return res; }
            /* re-init with an error of err_deg about axis (body frame), honest P0 */
            double ax[3] = { axis_in[0], axis_in[1], axis_in[2] };
            const double nn = sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2]);
            double phi[3], E[9], Rw[9];
            for (int i = 0; i < 3; ++i) phi[i] = ax[i] / nn * err_deg * D2R;
            dexp(phi, E); dmul(R, E, Rw);
            float Rf[9];
            for (int i = 0; i < 9; ++i) Rf[i] = (float)Rw[i];
            const float p0 = (mode != 1) ? (float)((err_deg * D2R) * (err_deg * D2R)) : ec.ekf.p0_att;
            iekf_reset_attitude(&e.f, Rf, p0);
            if (mode == 2) e.f.acc_grace = 0;         /* no grace: NIS inflation from the first update */
            e.f.lockouts = 0;
            injected = 1;
            last_above = t;
            continue;
        }
        if (injected) {
            const double d = att_err_deg(R, e.f.R);
            if (d >= CONV_DEG) last_above = t;
        }
    }
    res.final_deg = att_err_deg(R, e.f.R);
    res.lockouts = e.f.lockouts;
    res.conv_s = (res.final_deg < CONV_DEG) ? last_above - t_err : -1.0;
    return res;
}

/* ---------------------------------------------------------------- tests */
/* INTERNAL_FREQ_FINE -> ODR: 0.13 % per LSB, implausible values (> 10 %) ignored */
static void test_odr_trim(void)
{
    CHECK(imu_odr_trimmed(960.0f, 0) == 960.0f, "fine 0: nominal");
    /* the bench board measured 916 Hz: -4.6 % = fine -35 or -36 */
    CHECK(fabsf(imu_odr_trimmed(960.0f, -35) - 916.3f) < 0.1f, "fine -35: %.1f Hz", imu_odr_trimmed(960.0f, -35));
    CHECK(fabsf(imu_odr_trimmed(1920.0f, 20) - 1969.9f) < 0.1f, "fine +20 at 1920 Hz");
    CHECK(imu_odr_trimmed(960.0f, 77) == 960.0f && imu_odr_trimmed(960.0f, -128) == 960.0f, "beyond 10 %%: nominal");
    CHECK(fabsf(imu_odr_trimmed(960.0f, 76) - 960.0f * 1.0988f) < 0.1f, "9.88 %% kept");
}

int main(void)
{
    test_odr_trim();
    const double dip = 58.0 * D2R;
    s_m_w[0] = 48.0 * cos(dip); s_m_w[1] = 0.0; s_m_w[2] = -48.0 * sin(dip);

    static const double AX[6][3] = { {1,0,0}, {0,1,0}, {0,0,1}, {1,1,1}, {1,-1,0.5}, {-0.3,0.8,-1} };
    const double ERR[4] = { 30, 90, 150, 179 };
    const int NAX = 6;

    /* [tangent][moving][err] */
    int    ok[2][2][4] = { { { 0 } } };
    double tmax[2][2][4] = { { { 0 } } }, tmed[2][2][4] = { { { 0 } } }, fmax_[2][2][4] = { { { 0 } } };
    unsigned lock[2][2][4] = { { { 0 } } };

    for (int tg = 0; tg < 2; ++tg)
        for (int mv = 0; mv < 2; ++mv)
            for (int ie = 0; ie < 4; ++ie) {
                double ts[6];
                int n = 0;
                for (int ia = 0; ia < NAX; ++ia) {
                    const result_t r = run(mv, ERR[ie], AX[ia], tg, 0);
                    if (r.conv_s >= 0.0 && r.conv_s <= 10.0) { ok[tg][mv][ie]++; ts[n++] = r.conv_s; if (r.conv_s > tmax[tg][mv][ie]) tmax[tg][mv][ie] = r.conv_s; }
                    if (r.final_deg > fmax_[tg][mv][ie]) fmax_[tg][mv][ie] = r.final_deg;
                    lock[tg][mv][ie] += r.lockouts;
                }
                /* median */
                for (int i = 0; i < n; ++i) for (int j = i + 1; j < n; ++j) if (ts[j] < ts[i]) { double x = ts[i]; ts[i] = ts[j]; ts[j] = x; }
                tmed[tg][mv][ie] = n ? (n % 2 ? ts[n/2] : 0.5 * (ts[n/2 - 1] + ts[n/2])) : -1.0;
            }

    printf("cold start, production config (%d axes per case, honest P0, < %.0f deg for good within 10 s)\n", NAX, CONV_DEG);
    printf("  innovation  motion   error  converged  median s  max s  lockouts  worst final deg\n");
    for (int tg = 0; tg < 2; ++tg)
        for (int mv = 0; mv < 2; ++mv)
            for (int ie = 0; ie < 4; ++ie)
                printf("  %-10s  %-6s  %5.0f   %d/%d      %7.2f  %6.2f  %8u  %8.1f\n",
                       tg ? "tangential" : "full", mv ? "moving" : "static", ERR[ie], ok[tg][mv][ie], NAX,
                       tmed[tg][mv][ie], tmax[tg][mv][ie], lock[tg][mv][ie], fmax_[tg][mv][ie]);

    /* ---- regression (production: tangential) ---- */
    for (int mv = 0; mv < 2; ++mv) {
        CHECK(ok[1][mv][0] == NAX && tmax[1][mv][0] < 1.0, "%s 30 deg: all converge within 1 s (%d, %.2f s)", mv ? "moving" : "static", ok[1][mv][0], tmax[1][mv][0]);
        CHECK(ok[1][mv][1] == NAX && tmax[1][mv][1] < 2.0, "%s 90 deg: all converge within 2 s (%d, %.2f s)", mv ? "moving" : "static", ok[1][mv][1], tmax[1][mv][1]);
        CHECK(lock[1][mv][0] == 0 && lock[1][mv][1] == 0, "%s 30/90 deg: no lockout needed (%u, %u)", mv ? "moving" : "static", lock[1][mv][0], lock[1][mv][1]);
        CHECK(ok[1][mv][2] >= NAX - 1, "%s 150 deg: at least %d/%d converge (%d)", mv ? "moving" : "static", NAX - 1, NAX, ok[1][mv][2]);
        /* the tangential innovation must never be worse than the full one */
        for (int ie = 0; ie < 4; ++ie)
            CHECK(ok[1][mv][ie] >= ok[0][mv][ie], "%s %.0f deg: tangential converges at least as often (%d vs %d)",
                  mv ? "moving" : "static", ERR[ie], ok[1][mv][ie], ok[0][mv][ie]);
    }
    /* Variants, reported (and the tangential innovation must never be worse):
     *  1: overconfident P0 = production p0_att (2 deg), with the grace period;
     *  2: honest P0 but NO grace period: the accelerometer NIS inflation acts
     *     from the first update. This is the case item 1 of the handover is
     *     about: the radial part of the innovation, (1 - cos e)^2 / sigma^2,
     *     inflates R_acc whatever P is. */
    static const char *const VAR[3] = { "", "overconfident P0 (production p0_att), with grace",
                                         "honest P0, no grace period (NIS inflation from the first update)" };
    int vok[3][2][2][4] = { { { { 0 } } } };
    for (int md = 1; md <= 2; ++md) {
        printf("%s\n", VAR[md]);
        for (int tg = 0; tg < 2; ++tg)
            for (int mv = 0; mv < 2; ++mv) {
                double tm[4] = { 0 }; unsigned lk[4] = { 0 };
                for (int ie = 0; ie < 4; ++ie)
                    for (int ia = 0; ia < NAX; ++ia) {
                        const result_t r = run(mv, ERR[ie], AX[ia], tg, md);
                        if (r.conv_s >= 0.0 && r.conv_s <= 10.0) { vok[md][tg][mv][ie]++; if (r.conv_s > tm[ie]) tm[ie] = r.conv_s; }
                        lk[ie] += r.lockouts;
                    }
                for (int ie = 0; ie < 4; ++ie)
                    printf("  %-10s  %-6s  %5.0f   %d/%d               %6.2f  %8u\n", tg ? "tangential" : "full",
                           mv ? "moving" : "static", ERR[ie], vok[md][tg][mv][ie], NAX, tm[ie], lk[ie]);
            }
        for (int mv = 0; mv < 2; ++mv)
            for (int ie = 0; ie < 4; ++ie)
                CHECK(vok[md][1][mv][ie] >= vok[md][0][mv][ie], "variant %d, %s %.0f deg: tangential not worse (%d vs %d)",
                      md, mv ? "moving" : "static", ERR[ie], vok[md][1][mv][ie], vok[md][0][mv][ie]);
    }
    /* 179 deg is reported, not required: near 180 deg the correction direction is ill-defined */
    printf("iekf: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
