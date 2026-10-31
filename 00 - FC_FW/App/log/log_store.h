/**
 * @file  log_store.h
 * @brief Flight-log store: ring of sectors on a NOR flash (format in
 *        log_format.h).
 *
 * The store works on any flash described by log_flash_t: the W25Q128JV of
 * the ACwing FC (drivers/w25q) or a RAM area on the bench
 * (bsp/log_flash_ram.c). It is used by one task only (the LogTask), so it
 * has no locks. Host-testable.
 *
 * - log_mount() reads every sector header once and rebuilds the list of
 *   logs and the write position (newest sector serial + 1).
 * - log_open() starts a new log at a fresh sector; log_append() packs
 *   records into a page buffer and programs whole pages; log_close() flushes
 *   the last page. A power loss loses at most the page in RAM.
 * - When the ring wraps, the oldest sector is erased: the oldest log becomes
 *   "truncated" or disappears.
 */
#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stdint.h>
#include "log_format.h"

#define LOG_LIST_MAX 64u

/** flash backend; addresses in bytes from the start of the log area */
typedef struct {
    void    *ctx;
    uint32_t size;                                                     /**< multiple of LOG_SECTOR */
    int    (*read)(void *ctx, uint32_t addr, void *buf, uint32_t n);
    int    (*prog)(void *ctx, uint32_t addr, const void *buf, uint32_t n); /**< inside one 256-byte page */
    int    (*erase)(void *ctx, uint32_t addr);                         /**< one 4 KB sector, blocking */
} log_flash_t;

typedef struct {
    uint16_t id;
    uint16_t first;          /**< sector of the lowest seq still present  */
    uint16_t nsect;
    uint8_t  truncated;      /**< seq 0 overwritten                       */
    uint8_t  active;         /**< being written now                       */
    uint32_t t0_ms;
} log_entry_t;

int      log_mount(const log_flash_t *f);          /**< 0 = ok (empty flash is ok) */
uint8_t  log_count(void);
const log_entry_t *log_entry(uint8_t index);       /**< oldest first */
const log_entry_t *log_find(uint16_t id);
uint8_t  log_used_pct(void);

int      log_open(uint32_t t0_ms);                 /**< 0 = ok */
int      log_append(uint8_t type, const void *payload, uint8_t len);
int      log_close(void);
uint8_t  log_is_open(void);

/** bytes of a closed log 'id' from 'offset' (sectors concatenated, headers included) */
int      log_read(uint16_t id, uint32_t offset, void *buf, uint32_t n);
uint32_t log_size(uint16_t id);                    /**< nsect × LOG_SECTOR, 0 unknown */

/** erases every sector that holds data; logs restart from id 1 */
int      log_erase_all(void);

#endif /* LOG_STORE_H */
