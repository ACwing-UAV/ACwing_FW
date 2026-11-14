/**
 * @file  hover_track.c
 * @brief Hover thrust for the flight core (see hover_track.h).
 */
#include "hover_track.h"
#include <math.h>
#include <string.h>

#define RING 16u
_Static_assert(HT_LAG_MS / HT_STEP_MS < RING, "lag ring too short");

static hover_est_t s_he;
static float       s_learned;
static uint8_t     s_have_learned, s_warned;
static float       s_ring[RING];       /* vertical collective of the last steps */
static uint8_t     s_ri;
static uint32_t    s_last_ms;
static uint8_t     s_have_last;

static float seed_from(float param)
{
    if (s_have_learned) return s_learned;
    return (param >= HOVER_EST_MIN && param <= HOVER_EST_MAX) ? param : HT_SEED_DEFAULT;
}

static void ring_push(float t_c)
{
    s_ring[s_ri] = t_c;
    s_ri = (uint8_t)((s_ri + 1u) % RING);
}

/* the collective HT_LAG_MS ago */
static float ring_delayed(void)
{
    const uint32_t back = HT_LAG_MS / HT_STEP_MS;
    return s_ring[(s_ri + RING - 1u - back) % RING];
}

/* 1 when an update is due (every HT_UPDATE_MS); *dt = time since the last one */
static uint8_t update_due(uint32_t now, float *dt)
{
    if (!s_have_last) { s_last_ms = now; s_have_last = 1u; return 0u; }
    const uint32_t el = now - s_last_ms;
    if (el < HT_UPDATE_MS) return 0u;
    s_last_ms = now;
    *dt = (float)el * 1e-3f;
    return 1u;
}

/* the model a_up = g (t_c / t_h - 1) leaves out drag and the tilt beyond 30 deg */
static uint8_t model_holds(const hover_gate_t *g)
{
    return (uint8_t)(g->airborne && !g->saturated && g->cos_tilt > HT_MIN_COS && fabsf(g->vz) < HT_MAX_VZ);
}

/* the plausibility warning: once per flight, after the estimate has settled */
static uint8_t warn_once(void)
{
    if (s_warned || s_he.n_upd < HT_MIN_UPDATES || !hover_est_abnormal(&s_he)) return 0u;
    s_warned = 1u;
    return 1u;
}

void hover_track_init(void)
{
    s_have_learned = 0u; s_learned = 0.0f;
    hover_track_arm(-1.0f);
}

void hover_track_arm(float param_seed)
{
    hover_est_init(&s_he, seed_from(param_seed), 0.1f);
    memset(s_ring, 0, sizeof s_ring);
    s_ri = 0u; s_have_last = 0u; s_warned = 0u;
}

void hover_track_disarm(uint32_t flight_ms)
{
    if (flight_ms >= HT_KEEP_MS && s_he.n_upd > HT_MIN_UPDATES) { s_learned = s_he.th; s_have_learned = 1u; }
}

uint8_t hover_track_step(uint32_t now, uint8_t armed, const hover_gate_t *g)
{
    ring_push(g->thrust * (g->cos_tilt > 0.0f ? g->cos_tilt : 0.0f));
    float dt = 0.0f;
    if (!armed || !update_due(now, &dt)) return 0u;
    hover_est_predict(&s_he, dt);
    const float t_c = ring_delayed();
    if (!model_holds(g) || t_c <= 0.02f) return 0u;
    (void)hover_est_update(&s_he, g->a_up, t_c, dt);
    return warn_once();
}

float hover_track_value(void) { return s_he.th; }
float hover_track_sd(void)    { return sqrtf(s_he.P); }
float hover_track_nis(void)   { return s_he.nis; }
