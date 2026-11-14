/**
 * @file  radio_fs.h
 * @brief Radio failsafe (HANDOVER_FW §6, fsRadio): decides when the lost radio link triggers it and when it ends.
 *
 * Only while the radio has control. Silence = time since the last valid radio frame (rc_input).
 *   in air, silence > 1 s       -> RFS_TRIGGER: the flight core applies fsRadio (hover / land / RTH), EVENT 21
 *   in air, link back (< 0.5 s) -> RFS_BACK:    the flight core gives a HOLD back to the pilot
 *   on the ground armed, > 3 s  -> RFS_DISARM
 * Flight-control context only. Host-testable.
 */
#ifndef RADIO_FS_H
#define RADIO_FS_H

#include <stdint.h>

#define RFS_TRIGGER_MS 1000u
#define RFS_BACK_MS    500u
#define RFS_GROUND_MS  3000u

enum { RFS_NONE = 0, RFS_TRIGGER, RFS_BACK, RFS_DISARM };

void    rfs_init(void);
uint8_t rfs_active(void);                 /**< the failsafe is on (FLIGHT.failsafe bit RADIO) */
uint8_t rfs_step(uint8_t radio_ctrl, uint32_t silence_ms, uint8_t armed, uint8_t in_air);

#endif /* RADIO_FS_H */
