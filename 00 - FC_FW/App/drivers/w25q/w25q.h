/**
 * @file  w25q.h
 * @brief Winbond W25Q serial NOR flash (W25Q128JV on the ACwing FC: 16 MB,
 *        4 KB sectors, 256-byte pages, SPI mode 0/3).
 *
 * Portable: the bus is one function that runs a transaction with chip select
 * held low for the whole of it (command and address in `tx`, then `rx`
 * bytes clocked in). The ACwing FC shares SPI1 with the IMU, so the binding
 * takes the bus lock around each transaction; a sector erase is one short
 * command followed by status polls, never a long bus hold.
 *
 * Provides the log_flash_t callbacks of log/log_store.h.
 */
#ifndef W25Q_H
#define W25Q_H

#include <stdint.h>

typedef struct {
    /** CS low, write tx (ntx bytes), read rx (nrx bytes), CS high; 0 = ok */
    int    (*xfer)(void *ctx, const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx);
    /** same, with a second write buffer after tx (page program data) */
    int    (*xfer_w)(void *ctx, const uint8_t *tx, uint32_t ntx, const uint8_t *data, uint32_t n);
    void   (*delay_ms)(uint32_t ms);           /**< yields the task while the chip is busy */
    uint32_t (*now_ms)(void);
    void    *ctx;
} w25q_bus_t;

typedef struct {
    w25q_bus_t bus;
    uint32_t   size;           /**< bytes, from the JEDEC id                */
    uint8_t    mfr, type, cap; /**< JEDEC id                                */
    uint32_t   base;           /**< start of the log area                   */
} w25q_t;

/** reads the JEDEC id, releases power-down; 0 = a W25Q-compatible part */
int w25q_init(w25q_t *d, const w25q_bus_t *bus);

int w25q_read(w25q_t *d, uint32_t addr, void *buf, uint32_t n);
int w25q_prog(w25q_t *d, uint32_t addr, const void *buf, uint32_t n);   /**< inside one page */
int w25q_erase_sector(w25q_t *d, uint32_t addr);                       /**< 4 KB, waits (≤ 400 ms) */
int w25q_erase_chip(w25q_t *d);                                        /**< waits (≤ 200 s) */

/* log_flash_t adapters (ctx = w25q_t*) */
int w25q_lf_read(void *ctx, uint32_t addr, void *buf, uint32_t n);
int w25q_lf_prog(void *ctx, uint32_t addr, const void *buf, uint32_t n);
int w25q_lf_erase(void *ctx, uint32_t addr);

#endif /* W25Q_H */
