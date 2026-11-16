/**
 * @file  cal_motors.c
 * @brief MOTORS calibration procedure (see cal_motors.h).
 */
#include "cal_motors.h"
#include <string.h>

void cm_start(cm_proc_t *p)
{
    memset(p, 0, sizeof *p);
}

uint8_t cm_sample(cm_proc_t *p, uint8_t test_motor)
{
    const uint8_t spinning = (uint8_t)(test_motor != 0u);
    if (spinning == p->spinning && (!spinning || test_motor == p->motor)) return 0u;
    p->spinning = spinning;
    if (spinning) {
        p->motor = test_motor;
        if (test_motor <= 8u) p->ok_mask |= (uint8_t)(1u << (test_motor - 1u));
    }
    return 1u;
}
