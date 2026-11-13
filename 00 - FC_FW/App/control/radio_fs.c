/**
 * @file  radio_fs.c
 * @brief Radio failsafe decisions (see radio_fs.h).
 */
#include "radio_fs.h"

static uint8_t s_on;

void    rfs_init(void)   { s_on = 0u; }
uint8_t rfs_active(void) { return s_on; }

uint8_t rfs_step(uint8_t radio_ctrl, uint32_t silence, uint8_t armed, uint8_t in_air)
{
    if (!radio_ctrl || !armed) { s_on = 0u; return RFS_NONE; }
    if (!in_air) {
        s_on = 0u;
        return (silence > RFS_GROUND_MS) ? RFS_DISARM : RFS_NONE;
    }
    if (!s_on && silence > RFS_TRIGGER_MS) { s_on = 1u; return RFS_TRIGGER; }
    if (s_on && silence < RFS_BACK_MS)     { s_on = 0u; return RFS_BACK; }
    return RFS_NONE;
}
