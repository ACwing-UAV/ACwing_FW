/**
 * @file  imu_watch.h
 * @brief IMU stall watch: detection, recovery attempts, events and the watchdog policy (#106).
 *
 * On the bench (24/11) the IMU stopped delivering samples after a long session (29 I2C errors): no STATE, a frozen
 * horizon, and nothing told the user. The flight core already stops using the estimate after 50 ms without gyro
 * (IMU_STALE_MS) and the rate-loop watchdog cuts the motors when armed (CTRL_STALL_MS), but nobody tried to bring
 * the sensor back and the IWDG was fed regardless.
 *
 * This module is pure logic (host-testable); the ImuTask calls it:
 * - imw_sample() whenever gyro samples were read;
 * - imw_step() at every loop: it says when to attempt a recovery (I2C bus recovery + IMU re-init) and which event
 *   to raise (IMU_LOST once per outage, IMU_RECOVERED with the outage length);
 * - imw_feed_wdg(): 0 when the board is disarmed and the IMU has been lost for IMW_RESET_MS, so the IWDG resets the
 *   MCU as a last resort. Armed, the watchdog is always fed (a reset would not bring the motors back sooner).
 */
#ifndef IMU_WATCH_H
#define IMU_WATCH_H

#include <stdint.h>

#define IMW_STALL_MS   50u     /**< no gyro sample for this long: lost (= IMU_STALE_MS, CTRL_STALL_MS)           */
#define IMW_RETRY_MS   250u    /**< between two recovery attempts                                                 */
#define IMW_OK_MS      100u    /**< samples flowing again for this long after an attempt: recovered               */
#define IMW_FLOW_MS    20u     /**< "flowing": the last sample at most this old (one is read every ~2 ms)          */
#define IMW_RESET_MS   5000u   /**< disarmed and still lost: stop feeding the IWDG (MCU reset)                    */

typedef enum { IMW_OK = 0, IMW_LOST = 1 } imw_state_t;

typedef struct {
    uint32_t last_ms;          /**< last gyro sample                         */
    uint32_t since_ms;         /**< the outage started (detection time)      */
    uint32_t next_try_ms;      /**< next recovery attempt                    */
    uint32_t back_ms;          /**< samples flowing again since              */
    uint32_t outages, recoveries;
    uint16_t tries;            /**< attempts in this outage                  */
    uint8_t  state;            /**< imw_state_t                              */
    uint8_t  back;             /**< samples seen again after an attempt      */
} imw_t;

typedef struct {
    uint8_t  recover;          /**< 1: attempt a recovery now                */
    uint8_t  event;            /**< 1: raise level/code/arg                  */
    uint8_t  level;
    uint16_t code;
    int32_t  arg;
} imw_out_t;

void    imw_init(imw_t *w, uint32_t now_ms);
void    imw_sample(imw_t *w, uint32_t now_ms);
void    imw_step(imw_t *w, uint32_t now_ms, imw_out_t *out);
uint8_t imw_ok(const imw_t *w);
uint8_t imw_feed_wdg(const imw_t *w, uint32_t now_ms, uint8_t armed);

#endif /* IMU_WATCH_H */
