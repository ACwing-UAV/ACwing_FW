/**
 * @file  control_hooks.c
 * @brief Empty implementations (phase 1: attitude estimation only).
 */
#include "control_hooks.h"

__attribute__((weak)) void ctrl_init(void) {}
__attribute__((weak)) void ctrl_rate_step(const ctrl_rate_in_t *in)     { (void)in; }
__attribute__((weak)) void ctrl_attitude_step(const ctrl_att_in_t *in)  { (void)in; }
__attribute__((weak)) void ctrl_set_level_trim(float r, float p)         { (void)r; (void)p; }
__attribute__((weak)) uint8_t ctrl_motor_pct(uint8_t m)                  { (void)m; return 0u; }
__attribute__((weak)) uint8_t ctrl_service(uint32_t now)                 { (void)now; return 0u; }
