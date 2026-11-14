/**
 * @file  motor_pwm.c
 * @brief HAL binding of the motor PWM outputs (see motor_pwm.h).
 */
#include "motor_pwm.h"
#include "motors.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

static const uint32_t k_ch[MOTOR_PWM_OUTPUTS] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

void motor_pwm_start(void)
{
    for (unsigned i = 0; i < MOTOR_PWM_OUTPUTS; ++i) {
        __HAL_TIM_SET_COMPARE(&htim2, k_ch[i], 0u);
        (void)HAL_TIM_PWM_Start(&htim2, k_ch[i]);
    }
}

void motor_pwm_set(unsigned motor, uint32_t cmp)
{
    if (motor >= MOTOR_PWM_OUTPUTS) return;
    if (cmp > MOTORS_PWM_TOP) cmp = MOTORS_PWM_TOP;
    __HAL_TIM_SET_COMPARE(&htim2, k_ch[motor], cmp);
}
