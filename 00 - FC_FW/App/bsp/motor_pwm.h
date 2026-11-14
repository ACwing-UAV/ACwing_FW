/**
 * @file  motor_pwm.h
 * @brief Board binding of the motor PWM outputs (used by control/motors.c when MOTORS_ENABLE = 1).
 *
 * Bench (NUCLEO-WB55) and ACwing FC rev A use the same timer for M1..M4:
 *   TIM2 CH1..CH4 on PA0, PA1, PA2, PA3, 64 MHz timer clock, ARR = MOTORS_PWM_TOP (1999) -> 32 kHz, 2000 levels.
 * TIM1 cannot serve 4 motors on the WB55: its CH4 exists only on PA11, which is USB_DM. On rev A, M5..M7 are on
 * TIM1 CH1..CH3 (PA8..PA10) and M8 on TIM16 CH1 (PA6): they are added here with the rev A port (MOTORS_N 8).
 * The timer itself is configured by CubeMX (MX_TIM2_Init in main.c); this file only starts it and writes the
 * compare registers.
 */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

#define MOTOR_PWM_OUTPUTS 4u      /**< outputs bound on this board */

void motor_pwm_start(void);                       /**< every output at 0, then PWM on */
void motor_pwm_set(unsigned motor, uint32_t cmp); /**< motor 0..MOTOR_PWM_OUTPUTS-1, compare 0..MOTORS_PWM_TOP */

#endif /* MOTOR_PWM_H */
