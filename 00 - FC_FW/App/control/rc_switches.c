/**
 * @file  rc_switches.c
 * @brief RC radio switches (see rc_switches.h).
 */
#include "rc_switches.h"
#include "proto_v2.h"

#define RSW_UNKNOWN (-2)          /* no valid frame since the last link loss */

static int s_arm  = RSW_UNKNOWN;  /* last arm-switch state: 0 off, 1 on       */
static int s_mpos = RSW_UNKNOWN;  /* last mode-switch position                */
static uint8_t s_arm_seen_off;    /* the arm switch was off at least once      */

void rsw_init(void)
{
    s_arm = RSW_UNKNOWN; s_mpos = RSW_UNKNOWN; s_arm_seen_off = 0u;
}

int rsw_position(uint16_t us)
{
    if (us < 800u) return -1;
    if (us < 1300u) return 0;
    return (us > 1700u) ? 2 : 1;
}

int rsw_mode_of(int pos, uint8_t map)
{
    static const int8_t k_map[2][3] = {
        { PV2_MODE_ANGLE, PV2_MODE_ALT_HOLD, PV2_MODE_POS_HOLD },
        { PV2_MODE_ACRO,  PV2_MODE_ANGLE,    PV2_MODE_ALT_HOLD },
    };
    if (pos < 0 || pos > 2 || map < 1u || map > 2u) return -1;
    return k_map[map - 1u][pos];
}

/* arm switch: edge -> action */
static uint8_t arm_edge(const rsw_in_t *in)
{
    const int pos = rsw_position(in->aux_arm_us);
    if (pos < 0) { s_arm = RSW_UNKNOWN; return RSW_NONE; }       /* channel not used */
    const int on = (pos == 2);
    const int prev = s_arm;
    s_arm = on;
    if (!on) s_arm_seen_off = 1u;
    if (prev == RSW_UNKNOWN || prev == on || !in->radio_ctrl) return RSW_NONE;
    if (on) return (!in->armed && s_arm_seen_off) ? RSW_ARM : RSW_NONE;
    if (!in->armed) return RSW_NONE;
    return (in->in_air && in->kill_in_air) ? RSW_KILL : RSW_DISARM;
}

/* mode switch: position change (or the radio taking control) -> mode */
static int8_t mode_change(const rsw_in_t *in)
{
    const int pos = rsw_position(in->aux_mode_us);
    const int prev = s_mpos;
    s_mpos = (pos < 0) ? RSW_UNKNOWN : pos;
    if (pos < 0 || !in->radio_ctrl) return -1;
    if (pos == prev && !in->took_control) return -1;
    return (int8_t)rsw_mode_of(pos, in->mode_map);
}

void rsw_step(const rsw_in_t *in, rsw_out_t *out)
{
    out->action = RSW_NONE; out->mode = -1;
    if (!in->link) { s_arm = RSW_UNKNOWN; s_mpos = RSW_UNKNOWN; return; }   /* failsafe values are no switch */
    out->action = arm_edge(in);
    out->mode = mode_change(in);
}
