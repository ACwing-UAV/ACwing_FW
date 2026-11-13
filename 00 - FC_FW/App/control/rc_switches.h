/**
 * @file  rc_switches.h
 * @brief RC radio switches (HANDOVER_FW §6): arm switch on AUX1, flight-mode switch on AUX2.
 *
 * Edges, not levels. The switches are tracked whenever the radio link is up, and they act only while the radio
 * has control (input_arb.h):
 *   - AUX1 off -> on: ARM, only if the switch was seen off before (a switch left on at power-up never arms).
 *   - AUX1 on -> off, armed: DISARM on the ground; in the air KILL if rcArmSwKill = 1 (as Betaflight), otherwise
 *     DISARM (refused in the air by the flight core).
 *   - AUX2 position change, or the radio taking control: SET_MODE from the map (rcModeMap):
 *       map 0: off (default)   map 1: low ANGLE, mid ALT_HOLD, high POS_HOLD   map 2: low ACRO, mid ANGLE, high ALT_HOLD
 *     A two-position switch gives low and high. Off by default because receivers send 1500 us (mid) on the
 *     channels without a switch: a map always on would change the mode whenever the radio takes control.
 * Levels are never acted on: a radio taking control with the arm switch off does not disarm a drone the app
 * armed. After a link loss the first valid frame sets the positions again without an edge.
 *
 * Positions: low < 1300 us, mid, high > 1700 us; below 800 us no signal (channel not used).
 * Pure logic, flight-control context. Host-testable.
 */
#ifndef RC_SWITCHES_H
#define RC_SWITCHES_H

#include <stdint.h>

enum { RSW_NONE = 0, RSW_ARM = 1, RSW_DISARM = 2, RSW_KILL = 3 };

typedef struct {
    uint8_t  link;            /**< radio frames valid (rc_state_t.valid)         */
    uint8_t  radio_ctrl;      /**< the radio has control                         */
    uint8_t  took_control;    /**< it took control on this step                  */
    uint16_t aux_arm_us;      /**< AUX1                                          */
    uint16_t aux_mode_us;     /**< AUX2                                          */
    uint8_t  armed, in_air;
    uint8_t  kill_in_air;     /**< param rcArmSwKill                             */
    uint8_t  mode_map;        /**< param rcModeMap: 0 off, 1, 2                  */
} rsw_in_t;

typedef struct {
    uint8_t action;           /**< RSW_NONE / ARM / DISARM / KILL                */
    int8_t  mode;             /**< PV2_MODE_* to set, -1 none                    */
} rsw_out_t;

void rsw_init(void);
void rsw_step(const rsw_in_t *in, rsw_out_t *out);
/** switch position of a channel: 0 low, 1 mid, 2 high, -1 no signal */
int  rsw_position(uint16_t us);
/** flight mode of a mode-switch position for a map (PV2_MODE_*), -1 none (map 0 = off) */
int  rsw_mode_of(int position, uint8_t map);

#endif /* RC_SWITCHES_H */
