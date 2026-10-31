/**
 * @file  log_flash_ram.c
 * @brief RAM log flash (see log_flash_ram.h).
 */
#include "log_flash_ram.h"
#include <string.h>

typedef struct { uint8_t *b; uint32_t size; } ram_t;
static ram_t s_ram;

static int rd(void *c, uint32_t a, void *buf, uint32_t n)
{
    const ram_t *r = (const ram_t *)c;
    if (a + n > r->size) return -1;
    memcpy(buf, r->b + a, n);
    return 0;
}

static int pg(void *c, uint32_t a, const void *buf, uint32_t n)
{
    const ram_t *r = (const ram_t *)c;
    if (a + n > r->size || (a % LOG_PAGE) + n > LOG_PAGE) return -1;
    const uint8_t *p = (const uint8_t *)buf;
    for (uint32_t k = 0; k < n; ++k) r->b[a + k] &= p[k];        /* NOR: 1 -> 0 only */
    return 0;
}

static int er(void *c, uint32_t a)
{
    const ram_t *r = (const ram_t *)c;
    if (a % LOG_SECTOR || a + LOG_SECTOR > r->size) return -1;
    memset(r->b + a, 0xFF, LOG_SECTOR);
    return 0;
}

void log_flash_ram_init(log_flash_t *f, uint8_t *buf, uint32_t size)
{
    s_ram.b = buf; s_ram.size = size - size % LOG_SECTOR;
    f->ctx = &s_ram; f->size = s_ram.size;
    f->read = rd; f->prog = pg; f->erase = er;
}
