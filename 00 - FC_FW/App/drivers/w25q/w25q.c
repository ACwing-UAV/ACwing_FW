/**
 * @file  w25q.c
 * @brief W25Q serial NOR flash driver (see w25q.h). Commands from the
 *        Winbond W25Q128JV datasheet (rev. F), 3-byte addressing.
 */
#include "w25q.h"
#include <string.h>

#define CMD_WREN       0x06u
#define CMD_RDSR1      0x05u
#define CMD_READ       0x03u   /* up to 50 MHz; the binding runs below that */
#define CMD_PP         0x02u
#define CMD_SE         0x20u   /* 4 KB */
#define CMD_CE         0xC7u
#define CMD_JEDEC      0x9Fu
#define CMD_RELEASE_PD 0xABu
#define SR1_BUSY       0x01u
#define SR1_WEL        0x02u

#define T_PP_MS        3u      /* page program max */
#define T_SE_MS        400u    /* sector erase max */
#define T_CE_MS        200000u /* chip erase max */

static int cmd1(w25q_t *d, uint8_t c) { return d->bus.xfer(d->bus.ctx, &c, 1, NULL, 0); }

static int status(w25q_t *d, uint8_t *sr)
{
    const uint8_t c = CMD_RDSR1;
    return d->bus.xfer(d->bus.ctx, &c, 1, sr, 1);
}

/* waits for BUSY = 0; polls fast first (page program), then yields */
static int wait_ready(w25q_t *d, uint32_t max_ms)
{
    const uint32_t t0 = d->bus.now_ms ? d->bus.now_ms() : 0u;
    for (uint32_t k = 0;; ++k) {
        uint8_t sr = 0xFFu;
        if (status(d, &sr) != 0) return -1;
        if (!(sr & SR1_BUSY)) return 0;
        if (d->bus.now_ms && (d->bus.now_ms() - t0) > max_ms + 10u) return -2;
        if (!d->bus.now_ms && k > max_ms * 100u) return -2;          /* host / no clock */
        if (k > 20u && d->bus.delay_ms) d->bus.delay_ms(1);
    }
}

static int write_enable(w25q_t *d)
{
    if (cmd1(d, CMD_WREN) != 0) return -1;
    uint8_t sr = 0;
    if (status(d, &sr) != 0 || !(sr & SR1_WEL)) return -1;
    return 0;
}

static void a24(uint8_t *b, uint8_t cmd, uint32_t a)
{
    b[0] = cmd; b[1] = (uint8_t)(a >> 16); b[2] = (uint8_t)(a >> 8); b[3] = (uint8_t)a;
}

int w25q_init(w25q_t *d, const w25q_bus_t *bus)
{
    memset(d, 0, sizeof *d);
    d->bus = *bus;
    (void)cmd1(d, CMD_RELEASE_PD);
    if (d->bus.delay_ms) d->bus.delay_ms(1);                       /* tRES1 = 3 us */
    uint8_t id[3] = { 0, 0, 0 };
    const uint8_t c = CMD_JEDEC;
    if (d->bus.xfer(d->bus.ctx, &c, 1, id, 3) != 0) return -1;
    d->mfr = id[0]; d->type = id[1]; d->cap = id[2];
    /* Winbond 0xEF, 0x40 (SPI) / 0x70 (QPI-capable JV-IM); capacity 2^cap bytes */
    if (d->mfr != 0xEFu || (d->type != 0x40u && d->type != 0x70u) || d->cap < 0x14u || d->cap > 0x18u) return -2;
    d->size = 1u << d->cap;                                         /* 0x18 = 16 MB: 3-byte addresses */
    return 0;
}

int w25q_read(w25q_t *d, uint32_t addr, void *buf, uint32_t n)
{
    if (addr + n > d->size) return -1;
    uint8_t c[4];
    a24(c, CMD_READ, d->base + addr);
    return d->bus.xfer(d->bus.ctx, c, 4, (uint8_t *)buf, n);
}

int w25q_prog(w25q_t *d, uint32_t addr, const void *buf, uint32_t n)
{
    if (n == 0u) return 0;
    if (addr + n > d->size || (addr & 0xFFu) + n > 256u) return -1;   /* one page */
    if (write_enable(d) != 0) return -1;
    uint8_t c[4];
    a24(c, CMD_PP, d->base + addr);
    if (d->bus.xfer_w(d->bus.ctx, c, 4, (const uint8_t *)buf, n) != 0) return -1;
    return wait_ready(d, T_PP_MS);
}

int w25q_erase_sector(w25q_t *d, uint32_t addr)
{
    if (addr >= d->size || (addr & 0xFFFu)) return -1;
    if (write_enable(d) != 0) return -1;
    uint8_t c[4];
    a24(c, CMD_SE, d->base + addr);
    if (d->bus.xfer(d->bus.ctx, c, 4, NULL, 0) != 0) return -1;
    return wait_ready(d, T_SE_MS);
}

int w25q_erase_chip(w25q_t *d)
{
    if (write_enable(d) != 0) return -1;
    if (cmd1(d, CMD_CE) != 0) return -1;
    return wait_ready(d, T_CE_MS);
}

int w25q_lf_read(void *ctx, uint32_t a, void *b, uint32_t n)       { return w25q_read((w25q_t *)ctx, a, b, n); }
int w25q_lf_prog(void *ctx, uint32_t a, const void *b, uint32_t n) { return w25q_prog((w25q_t *)ctx, a, b, n); }
int w25q_lf_erase(void *ctx, uint32_t a)                           { return w25q_erase_sector((w25q_t *)ctx, a); }
