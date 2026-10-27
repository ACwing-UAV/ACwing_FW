/**
 * @file  motors.h
 * @brief PWM for 4 brushed coreless motors (8520, 1S) - PREPARED, unused in phase 1.
 *
 * TIM1 CH1..CH4, 64 MHz timer clock, ARR = 1999 -> 32 kHz, 2000 levels.
 * The WB55 pins can NOT drive the motors: a power stage is required
 * (low-side logic-level N-MOSFET + freewheeling diode per motor), see
 * README.md. Check in CubeMX that the TIM1 pins do not clash with those used
 * by the IKS4A1 on the Arduino connector.
 */
#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#define MOTORS_ENABLE    0        /* 1 only with the power stage connected */
#define MOTORS_PWM_TOP   1999u

void motors_init(void);
void motors_arm(uint8_t armed);              /* disarmed: duty forced to 0   */
void motors_set(const float duty[4]);        /* 0..1, saturated              */
/* MOTOR_TEST (props off, disarmed): motor 1..4 at duty, the others off;
 * motor 0 = all off. Does nothing with MOTORS_ENABLE = 0. */
void motors_test(uint8_t motor, float duty);

#endif /* MOTORS_H */
