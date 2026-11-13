/**
 * @file  pos_ctrl.c
 * @brief Horizontal position hold (see pos_ctrl.h).
 */
#include "pos_ctrl.h"
#include <math.h>
#include <string.h>

#define G0 9.80665f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void pc_gains_default(pc_gains_t *g)
{
    g->kp_pos = 0.8f;
    g->kp_vel = 1.8f;
    g->ki_vel = 0.9f;
    g->i_lim  = 2.0f;           /* ~ 11 deg of trim or wind */
}

void pc_init(pos_ctrl_t *c, const pc_gains_t *g)
{
    memset(c, 0, sizeof *c);
    c->g = *g;
    c->state = PC_BRAKE;
}

void pc_reset(pos_ctrl_t *c)
{
    c->p[0] = c->p[1] = c->p_sp[0] = c->p_sp[1] = 0.0f;
    c->i[0] = c->i[1] = 0.0f;
    c->v_sp[0] = c->v_sp[1] = 0.0f;
    c->brake_s = 0.0f;
    c->state = PC_BRAKE;
}

/* world (NW) <-> heading frame (x forward, y left) */
static void to_world(float yaw, const float h[2], float w[2])
{
    const float c = cosf(yaw), s = sinf(yaw);
    w[0] = c * h[0] - s * h[1];
    w[1] = s * h[0] + c * h[1];
}
static void to_heading(float yaw, const float w[2], float h[2])
{
    const float c = cosf(yaw), s = sinf(yaw);
    h[0] =  c * w[0] + s * w[1];
    h[1] = -s * w[0] + c * w[1];
}

/* velocity setpoint (heading frame) from the sticks or the held point; updates the state */
static void velocity_sp(pos_ctrl_t *c, const pc_in_t *in, float v_sp[2])
{
    const uint8_t moving = (uint8_t)(fabsf(in->stick_fwd) > PC_DEADBAND || fabsf(in->stick_left) > PC_DEADBAND);
    if (moving) {
        c->state = PC_MOVE;
        v_sp[0] = clampf(in->stick_fwd, -1.0f, 1.0f) * in->v_max;
        v_sp[1] = clampf(in->stick_left, -1.0f, 1.0f) * in->v_max;
        return;
    }
    if (c->state == PC_MOVE) { c->state = PC_BRAKE; c->brake_s = 0.0f; }
    if (c->state == PC_BRAKE) {
        c->brake_s += in->dt;
        const float sp = sqrtf(in->vel_b[0] * in->vel_b[0] + in->vel_b[1] * in->vel_b[1]);
        if (sp < PC_LATCH_V || c->brake_s > PC_BRAKE_MAX_S) {
            c->state = PC_HOLD;
            c->p_sp[0] = c->p[0]; c->p_sp[1] = c->p[1];
        }
        v_sp[0] = v_sp[1] = 0.0f;
        return;
    }
    /* holding: back to the point, at most PC_HOLD_VMAX */
    const float ew[2] = { c->p_sp[0] - c->p[0], c->p_sp[1] - c->p[1] };
    float eh[2];
    to_heading(in->yaw, ew, eh);
    v_sp[0] = c->g.kp_pos * eh[0];
    v_sp[1] = c->g.kp_pos * eh[1];
    const float n = sqrtf(v_sp[0] * v_sp[0] + v_sp[1] * v_sp[1]);
    const float vmax = (in->v_max < PC_HOLD_VMAX) ? in->v_max : PC_HOLD_VMAX;
    if (n > vmax && n > 1e-6f) { v_sp[0] *= vmax / n; v_sp[1] *= vmax / n; }
}

void pc_step(pos_ctrl_t *c, const pc_in_t *in, float *roll, float *pitch)
{
    const float dt = (in->dt > 0.0f && in->dt < 0.1f) ? in->dt : 0.0f;
    /* dead reckoning in the world plane */
    float vw[2];
    to_world(in->yaw, in->vel_b, vw);
    c->p[0] += vw[0] * dt; c->p[1] += vw[1] * dt;

    float v_sp[2];
    velocity_sp(c, in, v_sp);
    c->v_sp[0] = v_sp[0]; c->v_sp[1] = v_sp[1];

    /* velocity loop in the world frame: the wind is fixed in the world, so the integrator does not have to follow
     * the heading while the drone yaws; the acceleration is turned back to the heading frame for the tilt */
    const float tmax = clampf(in->tilt_max, 0.05f, 0.8f);
    const float amax = G0 * tanf(tmax);
    float vspw[2], aw[2], a[2];
    to_world(in->yaw, v_sp, vspw);
    for (int k = 0; k < 2; ++k) {
        const float e = vspw[k] - vw[k];
        const float u = c->g.kp_vel * e + c->i[k];
        /* integrator not further into the limit */
        if (!((u >= amax && e > 0.0f) || (u <= -amax && e < 0.0f)))
            c->i[k] = clampf(c->i[k] + c->g.ki_vel * e * dt, -c->g.i_lim, c->g.i_lim);
        aw[k] = c->g.kp_vel * e + c->i[k];
    }
    to_heading(in->yaw, aw, a);
    /* tilt of the total acceleration, within maxTilt, split along its direction (so |(roll, pitch)| <= maxTilt) */
    const float an = sqrtf(a[0] * a[0] + a[1] * a[1]);
    if (an < 1e-6f) { *pitch = 0.0f; *roll = 0.0f; return; }
    const float tilt = atanf((an < amax ? an : amax) / G0);
    *pitch = tilt * a[0] / an;
    *roll  = -tilt * a[1] / an;
}
