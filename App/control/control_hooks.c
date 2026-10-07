/**
 * @file  control_hooks.c
 * @brief Empty implementations (phase 1: attitude estimation only).
 */
#include "control_hooks.h"

__attribute__((weak)) void ctrl_init(void) {}
__attribute__((weak)) void ctrl_rate_step(const ctrl_rate_in_t *in)     { (void)in; }
__attribute__((weak)) void ctrl_attitude_step(const ctrl_att_in_t *in)  { (void)in; }
