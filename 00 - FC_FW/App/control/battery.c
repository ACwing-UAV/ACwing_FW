/**
 * @file  battery.c
 * @brief Battery monitor (see battery.h).
 */
#include "battery.h"
#include "proto_v2.h"
#include "spsc.h"
#include <math.h>
#include <string.h>

#define DETECT_MS     500u      /* battery present and stable before counting cells */
#define R_WIN_MS      100u      /* window of the internal resistance estimate      */
#define R_MIN_DI      0.5f      /* A: load step needed for an estimate              */
#define REST_I        0.3f      /* A: below this the pack is "resting"              */
#define SOC_REST_TAU  2.0f      /* s */
#define SOC_V_TAU     10.0f     /* s: voltage-only state of charge                  */
#define SOC_CORR_TAU  600.0f    /* s: pull of the voltage on the coulomb counter    */
#define I_SLOW_TAU    10.0f     /* s: current used for the time left                */

static batt_cfg_t   s_cfg;
static batt_state_t s;
static uint32_t     s_present_ms, s_absent_ms;
static uint32_t     s_below_ms[4], s_recover_ms;
static uint8_t      s_have_anchor, s_have_f;
static float        s_soc_anchor, s_used_anchor, s_i_slow;
/* internal resistance windows */
static float        s_wv, s_wi;
static uint32_t     s_wn, s_wms;
static float        s_pv, s_pi;
static uint8_t      s_have_pw;

static volatile uint32_t s_seq;
static batt_state_t      s_pub;

/* LiPo open-circuit voltage per cell, 0…100 % in 5 % steps */
static const float k_ocv[21] = {
    3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
    3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f
};

float batt_soc_from_ocv(float v)
{
    if (!(v > k_ocv[0])) return 0.0f;
    if (v >= k_ocv[20]) return 1.0f;
    for (int k = 1; k <= 20; ++k) {
        if (v < k_ocv[k]) {
            const float f = (v - k_ocv[k - 1]) / (k_ocv[k] - k_ocv[k - 1]);
            return 0.05f * ((float)(k - 1) + f);
        }
    }
    return 1.0f;
}

static float lpf(float y, float x, float dt, float tau) { return y + (x - y) * (dt / (tau + dt)); }
static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

void batt_init(const batt_cfg_t *cfg)
{
    s_cfg = *cfg;
    if (!(s_cfg.scale_v > 0.0f)) s_cfg.scale_v = 110.0f;
    if (!(s_cfg.scale_i > 0.0f)) s_cfg.scale_i = 1.0f;
    if (!(s_cfg.sens_v_per_a > 0.0f)) s_cfg.sens_v_per_a = 0.1f;
    memset(&s, 0, sizeof s);
    s.r_cell = BATT_R_CELL_DEFAULT;
    s.remain_s = -1.0f;
    s_present_ms = s_absent_ms = s_recover_ms = 0u;
    memset(s_below_ms, 0, sizeof s_below_ms);
    s_have_anchor = s_have_f = 0u;
    s_soc_anchor = s_used_anchor = s_i_slow = 0.0f;
    s_wv = s_wi = 0.0f; s_wn = s_wms = 0u; s_have_pw = 0u;
    s_seq = 0u;
    s_pub = s;
}

void batt_set_scales(float sv, float si)
{
    if (sv > 0.0f && isfinite(sv)) s_cfg.scale_v = sv;
    if (si > 0.0f && isfinite(si)) s_cfg.scale_i = si;
}

void batt_get_cfg(batt_cfg_t *out) { *out = s_cfg; }
const batt_state_t *batt_state(void) { return &s; }

void batt_read(batt_state_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_seq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        batt_state_t v = s_pub;
        SPSC_BARRIER();
        if (s_seq == s0) { *out = v; return; }
    }
}

static void publish(void)
{
    s_seq++;
    SPSC_BARRIER();
    s_pub = s;
    SPSC_BARRIER();
    s_seq++;
}

static void lost_battery(void)
{
    s.present = 0u;
    s.cells = 0u;
    s.level = PV2_BATT_OK;
    s.cell_v = s.cell_rest_v = 0.0f;
    s.soc = 0.0f; s.pct = 0u;
    s.remain_s = -1.0f;
    s_present_ms = 0u;
    memset(s_below_ms, 0, sizeof s_below_ms);
    s_recover_ms = 0u;
    s_have_anchor = 0u;
    s_have_pw = 0u;
}

/* internal resistance from load steps: two consecutive 100 ms means */
static void r_estimate(float v, float i, uint32_t dt)
{
    s_wv += v; s_wi += i; s_wn++; s_wms += dt;
    if (s_wms < R_WIN_MS) return;
    const float mv = s_wv / (float)s_wn, mi = s_wi / (float)s_wn;
    s_wv = s_wi = 0.0f; s_wn = 0u; s_wms = 0u;
    if (s_have_pw && s.cells) {
        const float di = mi - s_pi;
        if (fabsf(di) > R_MIN_DI) {
            const float r = -(mv - s_pv) / di / (float)s.cells;
            if (r > 0.002f && r < 0.5f) s.r_cell = 0.9f * s.r_cell + 0.1f * r;
        }
    }
    s_pv = mv; s_pi = mi; s_have_pw = 1u;
}

static uint8_t level_of(float cv, const batt_limits_t *L, float hyst)
{
    if (cv < L->emerg + hyst) return PV2_BATT_EMERGENCY;
    if (cv < L->crit + hyst)  return PV2_BATT_CRITICAL;
    if (cv < L->warn + hyst)  return PV2_BATT_LOW;
    return PV2_BATT_OK;
}

void batt_step(float pin_v, float cur_pin_v, uint32_t dt_ms, uint8_t armed, const batt_limits_t *L)
{
    const float dt = (float)dt_ms * 1e-3f;
    const float v = pin_v * s_cfg.scale_v / 10.0f;
    const float i = s_cfg.has_current ? cur_pin_v / s_cfg.sens_v_per_a * s_cfg.scale_i : 0.0f;
    s.vbat_raw = v;

    if (!s_have_f) { s.vbat = v; s.current = i; s_have_f = 1u; }
    else { s.vbat = lpf(s.vbat, v, dt, 0.1f); s.current = lpf(s.current, i, dt, 0.05f); }
    if (s.current > 0.0f) s.used_mah += s.current * dt / 3.6f;      /* A·s → mAh */
    s_i_slow = lpf(s_i_slow, s.current, dt, I_SLOW_TAU);

    /* ---- presence ---- */
    if (!s.present) {
        s_absent_ms = 0u;
        if (s.vbat > BATT_PRESENT_V) { s.present = 1u; s_present_ms = 0u; }
    } else if (s.vbat < BATT_ABSENT_V) {
        if ((s_absent_ms += dt_ms) > 200u) lost_battery();
    } else s_absent_ms = 0u;
    if (!s.present) { s.remain_s = -1.0f; publish(); return; }
    if (s_present_ms < 0xFFFF0000u) s_present_ms += dt_ms;

    /* ---- cells ---- */
    if (L->cells >= 1u && L->cells <= 6u) s.cells = L->cells;
    else if (!s.cells && !armed && s_present_ms >= DETECT_MS) {
        int n = (int)ceilf(s.vbat / BATT_CELL_DETECT_V);
        s.cells = (uint8_t)(n < 1 ? 1 : (n > 6 ? 6 : n));
    }
    if (!s.cells) { publish(); return; }

    const float n = (float)s.cells;
    const float cell = s.vbat / n;
    s.cell_v = (s.cell_v > 0.0f) ? lpf(s.cell_v, cell, dt, 0.5f) : cell;
    if (s_cfg.has_current) r_estimate(v, i, dt_ms);
    s.cell_rest_v = cell + s.current * s.r_cell;

    /* ---- warning level: enter after 2 s below, leave only disarmed ---- */
    const float thr[4] = { 0.0f, L->warn, L->crit, L->emerg };
    for (uint8_t k = 1u; k <= 3u; ++k) {
        s_below_ms[k] = (s.cell_v < thr[k]) ? s_below_ms[k] + dt_ms : 0u;
        if (s_below_ms[k] >= BATT_LEVEL_MS && s.level < k) s.level = k;
    }
    if (!armed && s.level > PV2_BATT_OK) {
        const uint8_t now_lvl = level_of(s.cell_v, L, BATT_RECOVER_HYST);
        if (now_lvl < s.level) {
            if ((s_recover_ms += dt_ms) >= BATT_RECOVER_MS) { s.level = now_lvl; s_recover_ms = 0u; }
        } else s_recover_ms = 0u;
    } else s_recover_ms = 0u;

    /* ---- state of charge ---- */
    const float soc_v = batt_soc_from_ocv(s.cell_rest_v);
    const uint8_t counting = (uint8_t)(s_cfg.has_current && L->capacity_mah > 0.0f);
    const uint8_t resting  = (uint8_t)(!armed && (!s_cfg.has_current || fabsf(s.current) < REST_I));
    if (!s_have_anchor) { s.soc = soc_v; s_have_anchor = 1u; s_soc_anchor = soc_v; s_used_anchor = s.used_mah; }
    else if (resting) {
        s.soc = lpf(s.soc, soc_v, dt, SOC_REST_TAU);
        s_soc_anchor = s.soc; s_used_anchor = s.used_mah;
    } else if (counting) {
        s.soc = s_soc_anchor - (s.used_mah - s_used_anchor) / L->capacity_mah;
        s_soc_anchor += (soc_v - s.soc) * (dt / SOC_CORR_TAU);
    } else {
        s.soc = lpf(s.soc, soc_v, dt, SOC_V_TAU);
    }
    s.soc = clamp01(s.soc);
    s.pct = (uint8_t)(s.soc * 100.0f + 0.5f);

    /* ---- flight time left ---- */
    if (armed && counting && s_i_slow > 0.2f) {
        const float usable = (s.soc - BATT_RESERVE_SOC) * L->capacity_mah;      /* mAh */
        s.remain_s = usable > 0.0f ? usable / (s_i_slow * 1000.0f) * 3600.0f : 0.0f;
    } else s.remain_s = -1.0f;

    publish();
}
