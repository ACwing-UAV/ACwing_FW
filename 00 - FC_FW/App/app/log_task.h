/**
 * @file  log_task.h
 * @brief LogTask: owns the log flash, writes the flight log and serves the
 *        log download over USB.
 *
 * - **Recording.** Producers (ImuTask, BattTask) put records in their own
 *   queue (log/log_queue.h); the LogTask drains them into the store
 *   (log/log_store.h). Parameter logMode: 0 off, 1 from ARM to 1 s after
 *   DISARM (one log per flight), 2 always (one log per power-on).
 * - **Download.** USB CDC carries protocol v2 frames (telemetry/usb_link.h):
 *   LOG_LIST (0x50) → LOG_ENTRY frames, LOG_READ (0x51) → LOG_DATA frames,
 *   LOG_ERASE (0x52, "WIPE"); every command is answered with an ACK, sent
 *   after its data (so the ACK means "complete"). Recording pauses (the open
 *   log is closed) while the PC talks to the drone and for 5 s after;
 *   LOG_READ and LOG_ERASE are refused when armed.
 *
 * log_task_step() is plain C (host test); the RTOS loop is at the end of
 * log_task.c.
 */
#ifndef LOG_TASK_H
#define LOG_TASK_H

#include <stdint.h>
#include "log_store.h"

#define LOG_POSTROLL_MS   1000u    /**< keep logging after DISARM            */
#define LOG_USB_PAUSE_MS  5000u    /**< no recording after a USB command      */

typedef struct {
    const log_flash_t *flash;      /**< NULL: no log flash on this board       */
    char     hw_rev;
    uint8_t  fw[3];
    uint16_t (*sensors)(void);     /**< CAPS bits for LR_INFO; may be NULL     */
    /** board binding: move queued USB TX bytes to the endpoint; may be NULL */
    void     (*usb_pump)(void);
    /** optional whole-chip erase (faster than sector by sector); may be NULL */
    int      (*erase_chip)(void);
} log_task_cfg_t;

void    log_task_init(const log_task_cfg_t *cfg);
void    log_task_step(uint32_t now_ms);
/** flash used %, 0xFF = no log flash (SYSTEM.log); any task */
uint8_t log_task_pct(void);
/** 1 while a USB transfer is in progress (the task runs faster) */
uint8_t log_task_busy(void);

#ifndef HOST_TEST
void log_task_start(const log_task_cfg_t *cfg);
#endif

#endif /* LOG_TASK_H */
