/**
 * @file  input_arb.c
 * @brief Input arbitration (see input_arb.h).
 */
#include "input_arb.h"
#include <string.h>

static uint8_t  s_radio;            /* the radio has control */
static uint32_t s_idle_since;
static uint8_t  s_have_idle;

void ia_init(void) { s_radio = 0u; s_have_idle = 0u; s_idle_since = 0u; }

static int16_t iabs16(int16_t v) { return (int16_t)(v < 0 ? -v : v); }

/* the pilot is using the radio now */
static uint8_t radio_used(const rc_state_t *r)
{
    return (uint8_t)(r->aux_us[0] > IA_ARM_SW_US ||
                     iabs16(r->roll) > IA_STICK_ACTIVE || iabs16(r->pitch) > IA_STICK_ACTIVE ||
                     iabs16(r->yaw) > IA_STICK_ACTIVE || r->thr > IA_THR_ACTIVE);
}

static void radio_sticks(const rc_state_t *r, ia_sticks_t *s)
{
    s->have = 1u; s->thr = r->thr; s->yaw = r->yaw; s->pitch = r->pitch; s->roll = r->roll; s->t_ms = r->t_ms;
}

/* disarmed: idle radio (arm switch off, sticks at rest) for IA_RELEASE_MS gives control back */
static uint8_t release_due(uint32_t now, const rc_state_t *r)
{
    if (r->valid && radio_used(r)) { s_have_idle = 0u; return 0u; }
    if (!s_have_idle) { s_idle_since = now; s_have_idle = 1u; }
    return (uint8_t)(!r->valid || (now - s_idle_since) >= IA_RELEASE_MS);
}

void ia_step(uint32_t now, uint8_t armed, const ia_sticks_t *app, const rc_state_t *radio, ia_out_t *o)
{
    memset(o, 0, sizeof *o);
    const uint8_t up = (uint8_t)(radio->ever && radio->valid);
    if (!s_radio && up && radio_used(radio)) { s_radio = 1u; s_have_idle = 0u; o->took_control = 1u; }
    else if (s_radio && !armed && release_due(now, radio)) s_radio = 0u;
    o->radio = s_radio;
    o->arm_sw = (uint8_t)(up && radio->aux_us[0] > IA_ARM_SW_US);
    if (s_radio) radio_sticks(radio, &o->sticks);
    else o->sticks = *app;
}
