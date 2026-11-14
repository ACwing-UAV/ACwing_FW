/**
 * @file  input_arb.h
 * @brief Input arbitration (HANDOVER_FW §6): who flies, the app or the RC radio, and with which sticks.
 *
 * The radio takes control when its link is up and the pilot uses it: arm switch (AUX1) on, or a stick moved
 * (roll / pitch / yaw beyond 15 %, or the throttle above the bottom). Once in control it keeps it while armed,
 * also with the link lost (the radio failsafe, radio_fs.h, decides then). Disarmed, it gives control back to the
 * app after 1 s with the arm switch off and the sticks idle, or when its link is lost.
 *
 * Sticks in the app RC scale (-1000..1000, throttle -1000 at the bottom, pitch + forward, yaw + right).
 * Flight-control context only. Host-testable.
 */
#ifndef INPUT_ARB_H
#define INPUT_ARB_H

#include <stdint.h>
#include "rc_input.h"

#define IA_STICK_ACTIVE   150     /**< 15 %: a roll / pitch / yaw stick moved     */
#define IA_THR_ACTIVE     (-850)  /**< throttle above the bottom                  */
#define IA_ARM_SW_US      1700u   /**< AUX1 above this: arm switch on             */
#define IA_RELEASE_MS     1000u   /**< disarmed and idle this long: back to the app */

/** one source of sticks */
typedef struct {
    uint8_t  have;               /**< frames have been received                     */
    int16_t  thr, yaw, pitch, roll;
    uint32_t t_ms;               /**< time of these sticks                          */
} ia_sticks_t;

typedef struct {
    uint8_t     radio;           /**< 1: the radio has control                       */
    uint8_t     took_control;    /**< 1 on the step the radio took control (EVENT 43) */
    uint8_t     arm_sw;          /**< radio arm switch on (and its link up)          */
    ia_sticks_t sticks;          /**< the sticks of the source in control            */
} ia_out_t;

void ia_init(void);
/** app: last app RC (have = 0: never); radio: rc_input state (ever = 0: no receiver) */
void ia_step(uint32_t now_ms, uint8_t armed, const ia_sticks_t *app, const rc_state_t *radio, ia_out_t *out);

#endif /* INPUT_ARB_H */
