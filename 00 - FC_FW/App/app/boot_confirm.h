/**
 * @file  boot_confirm.h
 * @brief When a firmware swapped in by MCUboot for a test boot may be confirmed (ADR 0015 §1). Pure logic,
 *        host-tested; the write is bsp/boot_slot.c (MCUboot's own bootutil_public.c).
 *
 * MCUboot reverts an unconfirmed image at the next reset (crash, IWDG, power). The image is confirmed only after it
 * has shown it can fly: the IMU read and the attitude estimate running, without a break, for BC_HEALTHY_MS. The flash
 * write (one double word, a few ms with the CPU2 handshake) is done only while disarmed. A build without MCUboot has
 * no confirm callback: nothing happens.
 */
#ifndef BOOT_CONFIRM_H
#define BOOT_CONFIRM_H

#include <stdint.h>

#define BC_HEALTHY_MS   60000u
#define BC_RETRY_MS     5000u          /**< a failed write is tried again after this */

typedef struct {
    uint8_t imu_ok;                    /**< IMU initialised and delivering samples      */
    uint8_t est_running;               /**< attitude estimate converged and running     */
    uint8_t armed;
} bc_in_t;

typedef struct {
    uint32_t healthy_since, retry_at;
    uint8_t  healthy, done, retry;
} bc_t;

void    bc_init(bc_t *b);
/** 1: confirm now (call the writer, then bc_result) */
uint8_t bc_step(bc_t *b, const bc_in_t *in, uint32_t now_ms);
void    bc_result(bc_t *b, int write_rc, uint32_t now_ms);   /**< 0 = written: done for this boot */

#endif /* BOOT_CONFIRM_H */
