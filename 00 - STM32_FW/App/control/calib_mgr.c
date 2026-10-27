/**
 * @file  calib_mgr.c
 * @brief Calibration state machines (see calib_mgr.h, HANDOVER_FW.md §10).
 *        Behavioural reference: calStart/calCapture/stepCal in simDrone.ts.
 */
#include "calib_mgr.h"
#include "proto_v2.h"
#include <math.h>
#include <string.h>

#define D2R                 0.017453292519943295f
#define R2D                 57.29577951308232f
#define STILL_GYRO          (3.0f * D2R)     /* rad/s: "still" */
#define ACCEL_STILL_MS      2000u            /* still window per position */
#define LEVEL_MS            5000u
#define GYRO_MS             5000u
#define GYRO_MAX_RESTARTS   3u
#define COMPASS_MIN_SAMPLES 150u
#define COMPASS_KEEP        256u             /* samples kept for the quality figure */

static cal_config_t s_cfg;
static cal_set_t   *s_active;       /* set in effect (caller-owned)            */
static cal_set_t    s_prev;         /* active set before this calibration      */
static cal_set_t    s_res;          /* active set with this calibration's result */
static uint8_t      s_applied;      /* s_res was applied (DONE)                */

static uint8_t  s_type, s_state, s_step, s_progress, s_quality, s_reason;
static uint8_t  s_changed, s_failed_type;
static uint32_t s_t0;               /* start of the current window (ms)        */
static uint32_t s_now;

/* ---- per-type working data ---- */
static struct {                      /* ACCEL */
    uint8_t done_mask, current, still_pct, pos;
    float   sum[3]; uint32_t n; uint32_t t_still;
    float   cap[6][3];               /* captured raw mean per position */
    float   last_g;                  /* |gyro| of the last sample */
    float   last_a[3];
    float   fit_err_pct;
} A;
static struct {                      /* LEVEL */
    double sr, sp, sr2, sp2; uint32_t n;
    float  roll, pitch; uint16_t remaining;
} L;
static struct {                      /* GYRO */
    double s[3], s2[3]; uint32_t n; uint8_t restarts;
    float  bias[3], noise; uint16_t remaining;
} G;
static struct {                      /* COMPASS */
    double N[6][6], b[6]; uint32_t n;
    uint8_t cov[5];
    float   keep[COMPASS_KEEP][3]; uint16_t nkeep, kidx;
    uint8_t interf, qual;
} C;

/* ================================================================== */
void cal_set_defaults(cal_set_t *s, const float hi[3])
{
    memset(s, 0, sizeof *s);
    for (int i = 0; i < 3; ++i) {
        s->accel.scale[i] = 1.0f;
        s->compass.scale[i] = 1.0f;
        s->compass.hi[i] = hi ? hi[i] : 0.0f;
    }
}

void cal_copy_type(cal_set_t *d, const cal_set_t *s, uint8_t type)
{
    switch (type) {
    case PV2_CAL_ACCEL:   d->accel   = s->accel;   break;
    case PV2_CAL_LEVEL:   d->level   = s->level;   break;
    case PV2_CAL_GYRO:    d->gyro    = s->gyro;    break;
    case PV2_CAL_COMPASS: d->compass = s->compass; break;
    case PV2_CAL_MOTORS:  d->motors  = s->motors;  break;
    default: return;
    }
    const uint32_t bit = 1u << (type - 1u);
    d->done_mask = (d->done_mask & ~bit) | (s->done_mask & bit);
}

void cal_set_sensors(uint16_t sensors) { s_cfg.sensors = sensors; }

void cal_init(const cal_config_t *cfg, cal_set_t *active)
{
    s_cfg = *cfg;
    s_active = active;
    s_type = 0; s_state = PV2_CS_IDLE;
    s_changed = 0; s_failed_type = 0; s_applied = 0;
}

static void set_state(uint8_t st, uint8_t step)
{
    if (st != s_state || step != s_step) s_changed = 1;
    s_state = st; s_step = step;
}

static void apply(const cal_set_t *set)
{
    cal_copy_type(s_active, set, s_type);
    if (s_cfg.apply) s_cfg.apply(s_type, s_active);
}

static void revert(void)
{
    if (s_applied) { apply(&s_prev); s_applied = 0; }
}

static void fail(uint8_t reason)
{
    revert();
    s_reason = reason;
    set_state(PV2_CS_FAILED, s_step);
    s_failed_type = s_type;
}

static void done(uint8_t step)
{
    s_progress = 100;
    set_state(PV2_CS_DONE, step);
    apply(&s_res);
    s_applied = 1;
}

/* ================================================================== */
uint8_t cal_active(void)  { return s_state != PV2_CS_IDLE; }
uint8_t cal_running(void) { return s_state != PV2_CS_IDLE && s_state != PV2_CS_DONE; }
uint8_t cal_type(void)    { return s_type; }
const cal_set_t *cal_result(void) { return &s_res; }

uint8_t cal_wants_samples(void)
{
    return (uint8_t)(s_state == PV2_CS_RUNNING || s_state == PV2_CS_WAITING_USER);
}

uint8_t cal_take_changed(void) { const uint8_t c = s_changed; s_changed = 0; return c; }

uint8_t cal_take_failed(void)
{
    const uint8_t t = s_failed_type;
    s_failed_type = 0;
    return t;
}

uint8_t cal_start(uint8_t type, uint8_t option, uint8_t armed, uint32_t now, uint8_t *reason)
{
    *reason = PV2_R_NONE;
    if (type < PV2_CAL_ACCEL || type > PV2_CAL_STICKS) return PV2_ACK_INVALID;
    if (armed) { *reason = PV2_R_ARMED; return PV2_ACK_REJECTED; }
    if (cal_running()) { *reason = PV2_R_CALIBRATING; return PV2_ACK_BUSY; }
    switch (type) {
    case PV2_CAL_FLOW_TOF:
        if (!(s_cfg.sensors & PV2_S_FLOW)) { *reason = PV2_R_SENSOR_MISSING; return PV2_ACK_REJECTED; }
        break;
    case PV2_CAL_BATTERY:
        if (!(s_cfg.sensors & PV2_S_CURRENT)) { *reason = PV2_R_SENSOR_MISSING; return PV2_ACK_REJECTED; }
        break;
    case PV2_CAL_STICKS:
        if (!(s_cfg.sensors & PV2_S_ELRS)) { *reason = PV2_R_SENSOR_MISSING; return PV2_ACK_REJECTED; }
        break;
    case PV2_CAL_MOTORS:
        if (!s_cfg.motors_built) { *reason = PV2_R_SENSOR_MISSING; return PV2_ACK_REJECTED; }
        break;
    case PV2_CAL_COMPASS:
        if (!(s_cfg.sensors & PV2_S_MAG_FC) || option == 2u) { *reason = PV2_R_SENSOR_MISSING; return PV2_ACK_REJECTED; }
        break;
    default:
        break;
    }
    /* the sensors exist but this firmware has no procedure for them yet */
    if (type == PV2_CAL_FLOW_TOF || type == PV2_CAL_BATTERY || type == PV2_CAL_STICKS ||
        type == PV2_CAL_MOTORS) return PV2_ACK_UNSUPPORTED;

    /* a previous calibration left in DONE (not saved) is discarded */
    if (s_state == PV2_CS_DONE) revert();

    s_type = type; s_now = now; s_t0 = now;
    s_prev = *s_active;
    s_res  = *s_active;
    s_applied = 0;
    s_progress = 0; s_quality = 0xFF; s_reason = 0;
    s_state = PV2_CS_IDLE; s_step = 0;
    switch (type) {
    case PV2_CAL_ACCEL:
        memset(&A, 0, sizeof A);
        set_state(PV2_CS_WAITING_USER, 2);
        break;
    case PV2_CAL_LEVEL:
        memset(&L, 0, sizeof L);
        L.remaining = LEVEL_MS;
        L.roll  = s_active->level.roll_deg;      /* shown until the first sample */
        L.pitch = s_active->level.pitch_deg;
        set_state(PV2_CS_RUNNING, 2);
        break;
    case PV2_CAL_GYRO:
        memset(&G, 0, sizeof G);
        G.remaining = GYRO_MS;
        set_state(PV2_CS_RUNNING, 1);
        break;
    case PV2_CAL_COMPASS:
        memset(&C, 0, sizeof C);
        set_state(PV2_CS_RUNNING, 2);
        break;
    default:
        break;
    }
    s_changed = 1;
    return PV2_ACK_OK;
}

uint8_t cal_abort(uint8_t *reason)
{
    if (!cal_active()) { *reason = PV2_R_NOT_CALIBRATING; return PV2_ACK_REJECTED; }
    revert();
    s_state = PV2_CS_IDLE; s_type = 0;
    *reason = PV2_R_NONE;
    return PV2_ACK_OK;
}

uint8_t cal_save_check(uint8_t *reason)
{
    *reason = PV2_R_NONE;
    if (!cal_active() || (s_state != PV2_CS_DONE && s_type != PV2_CAL_MOTORS)) {
        *reason = PV2_R_NOT_CALIBRATING;
        return PV2_ACK_REJECTED;
    }
    s_res.done_mask |= 1u << (s_type - 1u);
    s_active->done_mask |= 1u << (s_type - 1u);
    return PV2_ACK_OK;
}

void cal_saved(void)
{
    s_applied = 0;                       /* the new values stay in effect */
    set_state(PV2_CS_SAVED, s_step);     /* one frame in SAVED, then IDLE (cal_frame) */
}

/* ================================================================== */
/* ACCEL                                                               */
/* ================================================================== */
/* positions, body FLU: 0 +Z up, 1 -Z, 2 nose down (-X), 3 nose up (+X), 4 left down (-Y), 5 right down (+Y) */
static int accel_position(const float a[3])
{
    int ax = 0;
    for (int i = 1; i < 3; ++i) if (fabsf(a[i]) > fabsf(a[ax])) ax = i;
    if (fabsf(a[ax]) < 0.8f) return -1;
    const int pos = (a[ax] > 0.0f);
    switch (ax) {
    case 2:  return pos ? 0 : 1;
    case 0:  return pos ? 3 : 2;
    default: return pos ? 5 : 4;
    }
}

static int first_missing(uint8_t mask)
{
    for (int i = 0; i < 6; ++i) if (!(mask & (1u << i))) return i;
    return 0;
}

static void accel_fit(void)
{
    /* per axis: + and - 1 g positions */
    static const int pplus[3] = { 3, 5, 0 }, pminus[3] = { 2, 4, 1 };
    float off[3], sc[3];
    for (int k = 0; k < 3; ++k) {
        const float p = A.cap[pplus[k]][k], m = A.cap[pminus[k]][k];
        off[k] = 0.5f * (p + m);
        sc[k]  = (p - m) > 1e-3f ? 2.0f / (p - m) : 0.0f;
    }
    double e2 = 0.0;
    for (int i = 0; i < 6; ++i) {
        float n2 = 0.0f;
        for (int k = 0; k < 3; ++k) { const float c = sc[k] * (A.cap[i][k] - off[k]); n2 += c * c; }
        const double e = sqrt((double)n2) - 1.0;
        e2 += e * e;
    }
    A.fit_err_pct = (float)(100.0 * sqrt(e2 / 6.0));
    for (int k = 0; k < 3; ++k) {
        if (sc[k] < 0.90f || sc[k] > 1.10f || fabsf(off[k]) > 0.25f) { fail(PV2_R_OUT_OF_RANGE); return; }
        s_res.accel.off[k] = off[k];
        s_res.accel.scale[k] = sc[k];
    }
    const float q = 100.0f - 20.0f * A.fit_err_pct;
    s_quality = (uint8_t)(q < 0.0f ? 0.0f : (q > 100.0f ? 100.0f : q));
    done(3);
}

static void accel_capture_pos(int pos)
{
    if (A.n) for (int k = 0; k < 3; ++k) A.cap[pos][k] = A.sum[k] / (float)A.n;
    else     for (int k = 0; k < 3; ++k) A.cap[pos][k] = A.last_a[k];
    A.done_mask |= (uint8_t)(1u << pos);
    A.sum[0] = A.sum[1] = A.sum[2] = 0.0f; A.n = 0; A.still_pct = 0;
    A.pos = 0xFF;
    s_changed = 1;
    uint8_t cnt = 0;
    for (int i = 0; i < 6; ++i) cnt += (A.done_mask >> i) & 1u;
    s_progress = (uint8_t)(100u * cnt / 6u);
    if (A.done_mask == 0x3Fu) { accel_fit(); return; }
    A.current = (uint8_t)first_missing(A.done_mask);
}

static void accel_feed(const cal_sample_t *s, uint32_t now)
{
    const float g = sqrtf(s->gyro[0]*s->gyro[0] + s->gyro[1]*s->gyro[1] + s->gyro[2]*s->gyro[2]);
    A.last_g = g;
    memcpy(A.last_a, s->acc, sizeof A.last_a);
    const int pos = accel_position(s->acc);
    if (g >= STILL_GYRO || pos < 0 || (A.done_mask & (1u << pos))) {
        A.sum[0] = A.sum[1] = A.sum[2] = 0.0f; A.n = 0; A.still_pct = 0; A.pos = 0xFF;
        return;
    }
    if (pos != A.pos) {                       /* a new still position: start its window */
        A.pos = (uint8_t)pos; A.current = (uint8_t)pos;
        A.sum[0] = A.sum[1] = A.sum[2] = 0.0f; A.n = 0; A.t_still = now;
        s_changed = 1;
    }
    for (int k = 0; k < 3; ++k) A.sum[k] += s->acc[k];
    A.n++;
    const uint32_t el = now - A.t_still;
    A.still_pct = (uint8_t)(el >= ACCEL_STILL_MS ? 100u : el * 100u / ACCEL_STILL_MS);
    if (el >= ACCEL_STILL_MS) accel_capture_pos(pos);
}

/* ================================================================== */
/* LEVEL / GYRO                                                        */
/* ================================================================== */
static void level_feed(const cal_sample_t *s, uint32_t now)
{
    if (!s->running) return;
    const float g = sqrtf(s->gyro[0]*s->gyro[0] + s->gyro[1]*s->gyro[1] + s->gyro[2]*s->gyro[2]);
    if (g >= STILL_GYRO) { fail(PV2_R_NOT_STILL); return; }
    const float w = s->q[0], x = s->q[1], y = s->q[2], z = s->q[3];
    const float roll = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * R2D;
    float sp = 2.0f*(w*y - z*x);
    sp = sp > 1.0f ? 1.0f : (sp < -1.0f ? -1.0f : sp);
    const float pitch = asinf(sp) * R2D;
    L.sr += roll; L.sp += pitch; L.sr2 += (double)roll*roll; L.sp2 += (double)pitch*pitch; L.n++;
    L.roll = (float)(L.sr / L.n); L.pitch = (float)(L.sp / L.n);
    const uint32_t el = now - s_t0;
    L.remaining = (uint16_t)(el >= LEVEL_MS ? 0u : LEVEL_MS - el);
    s_progress = (uint8_t)(el >= LEVEL_MS ? 100u : el * 100u / LEVEL_MS);
    if (el >= LEVEL_MS) {
        const double vr = L.sr2 / L.n - (double)L.roll * L.roll, vp = L.sp2 / L.n - (double)L.pitch * L.pitch;
        const float sd = (float)sqrt((vr > 0 ? vr : 0) + (vp > 0 ? vp : 0));
        const float q = 100.0f - 100.0f * sd;            /* 0.05 deg rms -> 95 */
        s_quality = (uint8_t)(q < 0.0f ? 0.0f : q);
        s_res.level.roll_deg  = -L.roll;
        s_res.level.pitch_deg = -L.pitch;
        done(3);
    }
}

static void gyro_feed(const cal_sample_t *s, uint32_t now)
{
    const float g = sqrtf(s->gyro[0]*s->gyro[0] + s->gyro[1]*s->gyro[1] + s->gyro[2]*s->gyro[2]);
    if (g >= STILL_GYRO) {                     /* movement restarts the window */
        if (G.n) {
            if (++G.restarts > GYRO_MAX_RESTARTS) { fail(PV2_R_NOT_STILL); return; }
            memset(G.s, 0, sizeof G.s); memset(G.s2, 0, sizeof G.s2); G.n = 0;
        }
        s_t0 = now; G.remaining = GYRO_MS; s_progress = 0;
        return;
    }
    for (int k = 0; k < 3; ++k) { G.s[k] += s->gyro[k]; G.s2[k] += (double)s->gyro[k]*s->gyro[k]; }
    G.n++;
    const uint32_t el = now - s_t0;
    G.remaining = (uint16_t)(el >= GYRO_MS ? 0u : GYRO_MS - el);
    s_progress = (uint8_t)(el >= GYRO_MS ? 100u : el * 100u / GYRO_MS);
    if (el >= GYRO_MS) {
        double var = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double m = G.s[k] / G.n;
            G.bias[k] = (float)m;
            const double v = G.s2[k] / G.n - m*m;
            var += v > 0 ? v : 0;
            s_res.gyro.bias[k] = (float)m;
        }
        G.noise = (float)sqrt(var / 3.0);
        const float q = 100.0f - 100.0f * (G.noise * R2D);   /* 0.03 deg/s rms -> 97 */
        s_quality = (uint8_t)(q < 0.0f ? 0.0f : q);
        done(2);
    }
}

/* ================================================================== */
/* COMPASS                                                             */
/* ================================================================== */
static void bin(int ring, float a, float b)
{
    if (sqrtf(a*a + b*b) < 0.3f) return;      /* direction too close to the ring axis */
    int k = (int)((atan2f(b, a) + 3.14159265f) * (12.0f / 6.2831853f));
    if (k < 0) k = 0;
    if (k > 11) k = 11;
    const int i = ring * 12 + k;
    C.cov[i >> 3] |= (uint8_t)(1u << (i & 7));
}

static uint8_t coverage_count(void)
{
    uint8_t n = 0;
    for (int i = 0; i < 36; ++i) n += (C.cov[i >> 3] >> (i & 7)) & 1u;
    return n;
}

/* Gaussian elimination with partial pivoting, 6x6 */
static int solve6(double A6[6][6], double b6[6], double x[6])
{
    double M[6][7];
    for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) M[i][j] = A6[i][j]; M[i][6] = b6[i]; }
    for (int c = 0; c < 6; ++c) {
        int p = c;
        for (int r = c + 1; r < 6; ++r) if (fabs(M[r][c]) > fabs(M[p][c])) p = r;
        if (fabs(M[p][c]) < 1e-12) return -1;
        if (p != c) for (int j = 0; j < 7; ++j) { const double t = M[c][j]; M[c][j] = M[p][j]; M[p][j] = t; }
        for (int r = c + 1; r < 6; ++r) {
            const double f = M[r][c] / M[c][c];
            for (int j = c; j < 7; ++j) M[r][j] -= f * M[c][j];
        }
    }
    for (int i = 5; i >= 0; --i) {
        double s = M[i][6];
        for (int j = i + 1; j < 6; ++j) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return 0;
}

static void compass_fit(void)
{
    double p[6];
    if (solve6(C.N, C.b, p) != 0 || p[0] <= 0.0 || p[1] <= 0.0 || p[2] <= 0.0) { fail(PV2_R_OUT_OF_RANGE); return; }
    /* p0 x^2 + p1 y^2 + p2 z^2 + p3 x + p4 y + p5 z = 1
     * -> centre c_i = -p_{3+i} / (2 p_i), radius r_i = sqrt(G / p_i), G = 1 + sum p_i c_i^2 */
    double c[3], r[3], Gs = 1.0;
    for (int i = 0; i < 3; ++i) { c[i] = -p[3 + i] / (2.0 * p[i]); Gs += p[i] * c[i] * c[i]; }
    if (Gs <= 0.0) { fail(PV2_R_OUT_OF_RANGE); return; }
    double rm = 0.0;
    for (int i = 0; i < 3; ++i) { r[i] = sqrt(Gs / p[i]); rm += r[i] / 3.0; }
    for (int i = 0; i < 3; ++i) {
        const double sc = rm / r[i];
        if (r[i] < 15.0 || r[i] > 150.0 || sc < 0.7 || sc > 1.4) { fail(PV2_R_OUT_OF_RANGE); return; }
        s_res.compass.hi[i] = (float)c[i];
        s_res.compass.scale[i] = (float)sc;
    }
    /* residual: spread of the corrected field magnitude over the kept samples */
    double s1 = 0.0, s2 = 0.0;
    for (uint16_t k = 0; k < C.nkeep; ++k) {
        double n2 = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double v = s_res.compass.scale[i] * (C.keep[k][i] - c[i]);
            n2 += v * v;
        }
        const double n = sqrt(n2);
        s1 += n; s2 += n * n;
    }
    const double mean = s1 / C.nkeep, var = s2 / C.nkeep - mean * mean;
    const double rel = (var > 0 ? sqrt(var) : 0.0) / (mean > 1e-6 ? mean : 1.0);
    const double q = 100.0 - 1000.0 * rel;                 /* 1 % spread -> 90 */
    C.qual   = (uint8_t)(q < 0.0 ? 0.0 : (q > 100.0 ? 100.0 : q));
    C.interf = (uint8_t)(rel * 100.0 > 100.0 ? 100.0 : rel * 100.0 + 0.5);
    s_quality = C.qual;
    done(3);
}

static void compass_feed(const cal_sample_t *s)
{
    if (!s->mag_new) return;
    const float *m = s->mag;
    /* coverage: direction of the field corrected with the calibration in effect */
    float u[3], n = 0.0f;
    for (int i = 0; i < 3; ++i) { u[i] = s_prev.compass.scale[i] * (m[i] - s_prev.compass.hi[i]); n += u[i]*u[i]; }
    n = sqrtf(n);
    if (n < 1e-3f) return;
    for (int i = 0; i < 3; ++i) u[i] /= n;
    bin(0, u[0], u[1]);       /* ring 0: rotation about the vertical (spin flat)  */
    bin(1, u[0], u[2]);       /* ring 1: nose up / down                           */
    bin(2, u[1], u[2]);       /* ring 2: roll left / right                        */

    const double phi[6] = { (double)m[0]*m[0], (double)m[1]*m[1], (double)m[2]*m[2], m[0], m[1], m[2] };
    for (int i = 0; i < 6; ++i) {
        C.b[i] += phi[i];
        for (int j = 0; j < 6; ++j) C.N[i][j] += phi[i] * phi[j];
    }
    C.n++;
    memcpy(C.keep[C.kidx], m, sizeof C.keep[0]);
    C.kidx = (uint16_t)((C.kidx + 1u) % COMPASS_KEEP);
    if (C.nkeep < COMPASS_KEEP) C.nkeep++;

    const uint8_t cnt = coverage_count();
    s_progress = (uint8_t)(cnt * 100u / 36u);
    if (cnt == 36u && C.n >= COMPASS_MIN_SAMPLES) compass_fit();
}

/* ================================================================== */
void cal_feed(const cal_sample_t *s, uint32_t now)
{
    s_now = now;
    if (!cal_wants_samples()) return;
    switch (s_type) {
    case PV2_CAL_ACCEL:   accel_feed(s, now); break;
    case PV2_CAL_LEVEL:   level_feed(s, now); break;
    case PV2_CAL_GYRO:    gyro_feed(s, now); break;
    case PV2_CAL_COMPASS: compass_feed(s); break;
    default: break;
    }
}

void cal_tick(uint32_t now) { s_now = now; }

uint8_t cal_capture(uint32_t now, uint8_t *reason)
{
    (void)now;
    *reason = PV2_R_NONE;
    if (!cal_active()) { *reason = PV2_R_NOT_CALIBRATING; return PV2_ACK_REJECTED; }
    if (s_type == PV2_CAL_ACCEL && s_state == PV2_CS_WAITING_USER) {
        const int pos = accel_position(A.last_a);
        if (pos < 0 || A.last_g >= 2.0f * STILL_GYRO) { *reason = PV2_R_NOT_STILL; return PV2_ACK_REJECTED; }
        if (A.pos != (uint8_t)pos) { A.sum[0] = A.sum[1] = A.sum[2] = 0.0f; A.n = 0; }
        accel_capture_pos(pos);
        return PV2_ACK_OK;
    }
    return PV2_ACK_INVALID;                    /* no capture step in this calibration/state */
}

uint8_t cal_set_ref(uint8_t type, uint8_t index, float v, uint8_t *reason)
{
    *reason = PV2_R_NONE;
    if (!cal_active() || type != s_type) { *reason = PV2_R_NOT_CALIBRATING; return PV2_ACK_REJECTED; }
    if (type == PV2_CAL_LEVEL && s_state == PV2_CS_DONE && index <= 1u) {
        if (!isfinite(v) || fabsf(v) > 5.0f) { *reason = PV2_R_OUT_OF_RANGE; return PV2_ACK_INVALID; }
        if (index == 0u) s_res.level.roll_deg = v; else s_res.level.pitch_deg = v;
        apply(&s_res);
        s_applied = 1;
        s_changed = 1;
        return PV2_ACK_OK;
    }
    return PV2_ACK_INVALID;
}

/* ================================================================== */
static int16_t  cl16(float v)  { return (int16_t)(v > 32767.f ? 32767 : (v < -32768.f ? -32768 : lrintf(v))); }
static uint16_t clu16(float v) { return (uint16_t)(v > 65535.f ? 65535 : (v < 0.f ? 0 : lrintf(v))); }

size_t cal_frame(uint8_t *out, size_t cap)
{
    if (!cal_active()) return 0;
    pv2_calib_hdr_t h = { s_type, s_state, s_step, s_progress, s_quality, s_reason };
    union {
        pv2_cal_accel_t a; pv2_cal_level_t l; pv2_cal_gyro_t g; pv2_cal_compass_t c;
    } d;
    uint8_t n = 0;
    memset(&d, 0, sizeof d);
    switch (s_type) {
    case PV2_CAL_ACCEL: {
        const cal_accel_t *r = (s_state == PV2_CS_DONE || s_state == PV2_CS_SAVED) ? &s_res.accel : &s_active->accel;
        d.a.done_mask = A.done_mask; d.a.current = A.current; d.a.still_pct = A.still_pct;
        int16_t  o[3]; uint16_t sc[3];
        for (int k = 0; k < 3; ++k) { o[k] = cl16(r->off[k] * 1000.0f); sc[k] = clu16(r->scale[k] * 1e4f); }
        memcpy(d.a.offset_mg, o, sizeof o);
        memcpy(d.a.scale_e4, sc, sizeof sc);
        const uint16_t fe = clu16(A.fit_err_pct * 100.0f);
        memcpy(&d.a.fit_err_x100, &fe, 2);
        n = sizeof d.a;
        break;
    }
    case PV2_CAL_LEVEL: {
        const int16_t v[2] = { cl16(L.roll * 100.0f), cl16(L.pitch * 100.0f) };
        const int16_t t[2] = { cl16(s_res.level.roll_deg * 100.0f), cl16(s_res.level.pitch_deg * 100.0f) };
        memcpy(&d.l.roll_cdeg, &v[0], 2); memcpy(&d.l.pitch_cdeg, &v[1], 2);
        memcpy(&d.l.remaining_ms, &L.remaining, 2);
        memcpy(&d.l.trim_roll_cdeg, &t[0], 2); memcpy(&d.l.trim_pitch_cdeg, &t[1], 2);
        n = sizeof d.l;
        break;
    }
    case PV2_CAL_GYRO: {
        int16_t b[3];
        for (int k = 0; k < 3; ++k) b[k] = cl16(G.bias[k] * R2D * 1000.0f);
        const uint16_t nz = clu16(G.noise * R2D * 1000.0f);
        const int16_t  tc = 0;                 /* IMU temperature not read yet */
        memcpy(&d.g.remaining_ms, &G.remaining, 2);
        memcpy(d.g.bias_mdps, b, sizeof b);
        memcpy(&d.g.noise_mdps, &nz, 2);
        memcpy(&d.g.temp_cc, &tc, 2);
        n = sizeof d.g;
        break;
    }
    case PV2_CAL_COMPASS: {
        memcpy(d.c.coverage, C.cov, 5);
        const uint8_t fin = (s_state == PV2_CS_DONE || s_state == PV2_CS_SAVED);
        d.c.interf_fc   = fin ? C.interf : 0u;
        d.c.interf_gps  = 0xFFu;
        d.c.quality_fc  = fin ? C.qual : 0xFFu;
        d.c.quality_gps = 0xFFu;
        n = sizeof d.c;
        break;
    }
    default:
        break;
    }
    const size_t len = pv2_calib(&h, &d, n, out, cap);
    if (s_state == PV2_CS_SAVED) { s_state = PV2_CS_IDLE; s_type = 0; }   /* SAVED is sent once */
    return len;
}
