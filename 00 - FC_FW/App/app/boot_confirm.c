/**
 * @file  boot_confirm.c
 * @brief Confirmation of a test-booted firmware (see boot_confirm.h).
 */
#include "boot_confirm.h"
#include <string.h>

void bc_init(bc_t *b) { memset(b, 0, sizeof *b); }

uint8_t bc_step(bc_t *b, const bc_in_t *in, uint32_t now)
{
    if (b->done) return 0u;
    if (!(in->imu_ok && in->est_running)) { b->healthy = 0u; return 0u; }
    if (!b->healthy) { b->healthy = 1u; b->healthy_since = now; }
    if ((uint32_t)(now - b->healthy_since) < BC_HEALTHY_MS) return 0u;
    if (in->armed) return 0u;                                   /* no flash write while armed */
    if (b->retry && (int32_t)(now - b->retry_at) < 0) return 0u;
    return 1u;
}

void bc_result(bc_t *b, int rc, uint32_t now)
{
    if (rc == 0) { b->done = 1u; return; }
    b->retry = 1u;
    b->retry_at = now + BC_RETRY_MS;
}
