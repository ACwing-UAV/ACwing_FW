/**
 * @file  motors.c
 */
#include "motors.h"
#if MOTORS_ENABLE
#include "main.h"
extern TIM_HandleTypeDef htim1;
static const uint32_t k_ch[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
#endif

static volatile uint8_t s_armed;

void motors_init(void)
{
    s_armed = 0;
#if MOTORS_ENABLE
    for (int i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COMPARE(&htim1, k_ch[i], 0);
        (void)HAL_TIM_PWM_Start(&htim1, k_ch[i]);
    }
#endif
}

void motors_arm(uint8_t armed)
{
    s_armed = armed ? 1u : 0u;
    if (!s_armed) { const float z[4] = { 0, 0, 0, 0 }; motors_set(z); }
}

void motors_set(const float duty[4])
{
#if MOTORS_ENABLE
    for (int i = 0; i < 4; ++i) {
        float d = s_armed ? duty[i] : 0.0f;
        if (d < 0.0f) d = 0.0f;
        if (d > 1.0f) d = 1.0f;
        __HAL_TIM_SET_COMPARE(&htim1, k_ch[i], (uint32_t)(d * (float)MOTORS_PWM_TOP + 0.5f));
    }
#else
    (void)duty;
#endif
}
