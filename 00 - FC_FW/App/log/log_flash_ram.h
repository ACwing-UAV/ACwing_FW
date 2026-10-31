/**
 * @file  log_flash_ram.h
 * @brief log_flash_t on a RAM buffer with NOR semantics (erase sets 0xFF,
 *        program can only clear bits). Used on the bench, which has no log
 *        flash, and by the host tests. The log is lost at power-off.
 */
#ifndef LOG_FLASH_RAM_H
#define LOG_FLASH_RAM_H

#include "log_store.h"

/** buf: size bytes (rounded down to whole sectors) */
void log_flash_ram_init(log_flash_t *f, uint8_t *buf, uint32_t size);

#endif /* LOG_FLASH_RAM_H */
