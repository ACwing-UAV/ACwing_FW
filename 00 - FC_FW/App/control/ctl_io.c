/**
 * @file  ctl_io.c
 * @brief Vehicle state, reference, command and law registry behind ctl_api.h (ADR 0013).
 */
#include "ctl_io.h"
#include "params.h"
#include <math.h>
#include <string.h>

extern const ctl_law_t g_ctl_law_cascade;        /* ctl_law_cascade.c: the default law, index 0 */

static const ctl_law_t *s_laws[CTL_LAWS_MAX];
static unsigned         s_n_laws;
static int              s_active = -1;

static ctl_ref_t     s_ref;
static ctl_vehicle_t s_veh;
static float         s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float         s_w[3];
static float         s_hover = 0.5f, s_vcell, s_trim[2];
static ctl_status_t  s_status;
static ctl_cmd_t     s_cmd;

/* ------------------------------------------------------------------ registry */
__attribute__((weak)) void ctl_user_laws(void) { }
int ctl_register(const ctl_law_t *law)
{
    if (!law || !law->outer || !law->attitude || !law->rate || s_n_laws >= CTL_LAWS_MAX) return -1;
    for (unsigned i = 0; i < s_n_laws; ++i) if (s_laws[i] == law) return (int)i;
    s_laws[s_n_laws] = law;
    return (int)s_n_laws++;
}

int ctl_active(void) { return s_active; }
const char *ctl_active_name(void) { return (s_active >= 0) ? s_laws[s_active]->name : ""; }

static void activate(int idx)
{
    if (idx < 0 || (unsigned)idx >= s_n_laws) idx = 0;
    if (idx == s_active) return;
    s_active = idx;
    if (s_laws[idx]->init) s_laws[idx]->init();
    if (s_laws[idx]->reset) s_laws[idx]->reset();
}

static int wanted(void)
{
    const int i = params_index(105u);                 /* ctlLaw */
    return (i >= 0) ? (int)(param_get(i) + 0.5f) : 0;
}

void ctl_io_init(void)
{
    s_n_laws = 0; s_active = -1;
    memset(&s_ref, 0, sizeof s_ref);
    memset(&s_veh, 0, sizeof s_veh);
    memset(&s_status, 0, sizeof s_status);
    memset(&s_cmd, 0, sizeof s_cmd);
    s_q[0] = 1.0f; s_q[1] = s_q[2] = s_q[3] = 0.0f;
    s_w[0] = s_w[1] = s_w[2] = 0.0f;
    s_hover = 0.5f; s_vcell = 0.0f; s_trim[0] = s_trim[1] = 0.0f;
    s_ref.thrust_max = 1.0f; s_ref.vz_ceiling = CTL_NO_LIMIT;
    (void)ctl_register(&g_ctl_law_cascade);
    ctl_user_laws();                                  /* the laws of the user, indices 1, 2, ... */
    activate(wanted());
}

/* ------------------------------------------------------------------ producers */
void ctl_io_feed_vehicle(const ctl_vehicle_t *v)  { s_veh = *v; }
void ctl_io_feed_hover(float hover)               { s_hover = hover; }
void ctl_io_feed_attitude(const float q[4])       { memcpy(s_q, q, sizeof s_q); }
void ctl_io_feed_rates(const float w[3])          { memcpy(s_w, w, sizeof s_w); }
void ctl_io_feed_cell_voltage(float v_cell)       { s_vcell = v_cell; }
void ctl_io_set_level_trim(float r, float p)      { s_trim[0] = r; s_trim[1] = p; }
void ctl_io_set_ref(const ctl_ref_t *ref)         { s_ref = *ref; }

/* ------------------------------------------------------------------ steps */
void ctl_io_outer(float dt)
{
    if (s_active < 0) return;
    if (!s_ref.armed) {
        const int w = wanted();
        if (w != s_active && w >= 0 && (unsigned)w < s_n_laws) activate(w);   /* only while disarmed */
    }
    s_laws[s_active]->outer(dt);
}

void ctl_io_attitude(float dt) { if (s_active >= 0) s_laws[s_active]->attitude(dt); }

void ctl_io_rate(float dt)
{
    s_cmd.kind = CTL_CMD_NONE;
    if (s_active >= 0) s_laws[s_active]->rate(dt);
}

void ctl_io_take_cmd(ctl_cmd_t *cmd) { *cmd = s_cmd; }

/* ------------------------------------------------------------------ ctl_api.h: getters */
void    ctl_get_ref(ctl_ref_t *ref)      { *ref = s_ref; }
void    ctl_get_attitude(float q[4])     { memcpy(q, s_q, sizeof s_q); }
void    ctl_get_rates(float w[3])        { memcpy(w, s_w, sizeof s_w); }
float   ctl_get_yaw(void)                { return s_veh.yaw; }
float   ctl_get_cos_tilt(void)           { return s_veh.cos_tilt; }
uint8_t ctl_height_valid(void)           { return s_veh.h_valid; }
float   ctl_get_height(void)             { return s_veh.h; }
float   ctl_get_vz(void)                 { return s_veh.vz; }
float   ctl_get_accel_up(void)           { return s_veh.a_up; }
uint8_t ctl_vel_valid(void)              { return s_veh.vel_valid; }
void    ctl_get_vel_body(float v[2])     { v[0] = s_veh.vel_b[0]; v[1] = s_veh.vel_b[1]; }
float   ctl_get_hover_thrust(void)       { return s_hover; }
float   ctl_get_cell_voltage(void)       { return s_vcell; }
void    ctl_get_level_trim(float t[2])   { t[0] = s_trim[0]; t[1] = s_trim[1]; }

float ctl_get_param(uint16_t id)
{
    const int i = params_index(id);
    return (i >= 0) ? param_get(i) : NAN;
}

/* ------------------------------------------------------------------ ctl_api.h: setters */
static float clamp01(float v) { return (v > 0.0f) ? ((v < 1.0f) ? v : 1.0f) : 0.0f; }

void ctl_set_thrust_torque(float collective, const float torque[3])
{
    s_cmd.kind = CTL_CMD_THRUST_TORQUE;
    s_cmd.collective = clamp01(collective);
    memcpy(s_cmd.torque, torque, sizeof s_cmd.torque);
}

void ctl_set_motor_thrust(const float thrust[4])
{
    s_cmd.kind = CTL_CMD_MOTORS;
    for (int i = 0; i < 4; ++i) s_cmd.motor[i] = clamp01(thrust[i]);
}

void ctl_set_status(const ctl_status_t *st) { s_status = *st; }
void ctl_get_status(ctl_status_t *st)       { *st = s_status; }
