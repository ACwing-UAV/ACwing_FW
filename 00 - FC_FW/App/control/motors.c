/**
 * @file  motors.c
 */
#include "motors.h"
#if MOTORS_ENABLE
#include "motor_pwm.h"                 /* board binding: bsp/motor_pwm.c */
_Static_assert(MOTOR_PWM_OUTPUTS == MOTORS_N, "one PWM output per motor: add the outputs of this board in bsp/motor_pwm.c");
static uint32_t cmp_of(float d) { return (uint32_t)(d * (float)MOTORS_PWM_TOP + 0.5f); }
#endif

static volatile uint8_t s_armed;

void motors_init(void)
{
    s_armed = 0;
#if MOTORS_ENABLE
    motor_pwm_start();
#endif
}

void motors_arm(uint8_t armed)
{
    s_armed = armed ? 1u : 0u;
    if (!s_armed) { const float z[MOTORS_N] = { 0 }; motors_set(z); }
}

void motors_set(const float duty[MOTORS_N])
{
#if MOTORS_ENABLE
    for (unsigned i = 0; i < MOTORS_N; ++i) {
        float d = s_armed ? duty[i] : 0.0f;
        if (d < 0.0f) d = 0.0f;
        if (d > 1.0f) d = 1.0f;
        motor_pwm_set(i, cmp_of(d));
    }
#else
    (void)duty;
#endif
}

void motors_test(uint8_t motor, float duty)
{
#if MOTORS_ENABLE
    if (s_armed) return;                         /* never while armed */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 0.2f) duty = 0.2f;                /* MOTOR_TEST limit: 20 % */
    for (unsigned i = 0; i < MOTORS_N; ++i) {
        const float d = (motor == (uint8_t)(i + 1u)) ? duty : 0.0f;
        motor_pwm_set(i, cmp_of(d));
    }
#else
    (void)motor; (void)duty;
#endif
}
