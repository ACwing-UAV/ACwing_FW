/**
 * @file  log_store.c
 * @brief Flight-log store (see log_store.h).
 */
#include "log_store.h"
#include <string.h>

static const log_flash_t *F;
static uint16_t  s_nsect;
static uint16_t  s_wp;            /* next sector to write                 */
static uint32_t  s_serial;        /* last serial written                  */
static uint16_t  s_last_id;
static uint8_t   s_used[4096 / 8];   /* bit per sector holding a header (16 MB max) */

static log_entry_t s_list[LOG_LIST_MAX];
static uint8_t     s_n;

/* writer */
static uint8_t  s_open;
static uint16_t s_cur;            /* sector being written                  */
static uint16_t s_seq;
static uint32_t s_t0;
static uint32_t s_pos;            /* byte offset in the current sector     */
static uint8_t  s_page[LOG_PAGE];
static uint32_t s_page_base;      /* offset in the sector of s_page[0]     */

/* ------------------------------------------------------------------ CRCs */
uint8_t log_crc8(const uint8_t *p, uint32_t n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; ++k) c = (uint8_t)((c & 0x80u) ? ((unsigned)c << 1) ^ 0x07u : ((unsigned)c << 1));
    }
    return c;
}

uint16_t log_crc16(const uint8_t *p, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)(*p++) << 8;
        for (int k = 0; k < 8; ++k) c = (uint16_t)((c & 0x8000u) ? ((unsigned)c << 1) ^ 0x1021u : ((unsigned)c << 1));
    }
    return c;
}

/* ------------------------------------------------------------------ helpers */
static uint32_t addr(uint16_t sect) { return (uint32_t)sect * LOG_SECTOR; }
static uint16_t next(uint16_t s)    { return (uint16_t)((s + 1u) % s_nsect); }
static void     set_used(uint16_t s, uint8_t on)
{
    if (on) s_used[s >> 3] |= (uint8_t)(1u << (s & 7u)); else s_used[s >> 3] &= (uint8_t)~(1u << (s & 7u));
}
static uint8_t  is_used(uint16_t s) { return (uint8_t)((s_used[s >> 3] >> (s & 7u)) & 1u); }

static int hdr_ok(const log_sector_hdr_t *h)
{
    return h->magic == LOG_MAGIC && h->format == LOG_FORMAT &&
           h->crc == log_crc16((const uint8_t *)h, LOG_HDR_SIZE - 2u);
}

static int read_hdr(uint16_t s, log_sector_hdr_t *h)
{
    if (F->read(F->ctx, addr(s), h, sizeof *h) != 0) return -1;
    return hdr_ok(h) ? 0 : -1;
}

/* the oldest log loses sector 'sect' (about to be erased) */
static void forget_sector(uint16_t sect)
{
    if (!is_used(sect)) return;
    set_used(sect, 0);
    if (s_n && s_list[0].first == sect && !s_list[0].active) {
        s_list[0].first = next(sect);
        s_list[0].truncated = 1u;
        if (--s_list[0].nsect == 0u) { memmove(&s_list[0], &s_list[1], (s_n - 1u) * sizeof s_list[0]); s_n--; }
    }
}

/* ------------------------------------------------------------------ mount */
int log_mount(const log_flash_t *f)
{
    F = f;
    s_nsect = (uint16_t)(f->size / LOG_SECTOR);
    if (s_nsect == 0u || s_nsect > sizeof s_used * 8u) return -1;
    memset(s_used, 0, sizeof s_used);
    s_n = 0; s_open = 0; s_serial = 0; s_last_id = 0; s_wp = 0;

    /* newest sector = highest serial */
    uint8_t any = 0;
    uint16_t newest = 0;
    for (uint16_t s = 0; s < s_nsect; ++s) {
        log_sector_hdr_t h;
        if (read_hdr(s, &h) != 0) continue;
        set_used(s, 1);
        if (!any || (int32_t)(h.serial - s_serial) > 0) { s_serial = h.serial; newest = s; s_last_id = h.log_id; }
        any = 1;
    }
    if (!any) return 0;
    s_wp = next(newest);

    /* logs in ring order, oldest first: start after the newest sector */
    uint16_t s = s_wp;
    for (uint16_t k = 0; k < s_nsect; ++k, s = next(s)) {
        if (!is_used(s)) continue;
        log_sector_hdr_t h;
        if (read_hdr(s, &h) != 0) continue;
        log_entry_t *e = s_n ? &s_list[s_n - 1u] : NULL;
        if (e && e->id == h.log_id && (uint16_t)(e->first + e->nsect) % s_nsect == s) { e->nsect++; continue; }
        if (s_n == LOG_LIST_MAX) { memmove(&s_list[0], &s_list[1], (LOG_LIST_MAX - 1u) * sizeof s_list[0]); s_n--; }
        e = &s_list[s_n++];
        memset(e, 0, sizeof *e);
        e->id = h.log_id; e->first = s; e->nsect = 1; e->t0_ms = h.t0_ms;
        e->truncated = (uint8_t)(h.seq != 0u);
    }
    return 0;
}

uint8_t log_count(void) { return s_n; }
const log_entry_t *log_entry(uint8_t i) { return i < s_n ? &s_list[i] : NULL; }
uint8_t log_is_open(void) { return s_open; }

const log_entry_t *log_find(uint16_t id)
{
    for (uint8_t i = 0; i < s_n; ++i) if (s_list[i].id == id) return &s_list[i];
    return NULL;
}

uint8_t log_used_pct(void)
{
    if (!s_nsect) return 0;
    uint32_t n = 0;
    for (uint16_t s = 0; s < s_nsect; ++s) n += is_used(s);
    return (uint8_t)((n * 100u + s_nsect / 2u) / s_nsect);
}

/* ------------------------------------------------------------------ writer */
static int flush_page(void)
{
    const uint32_t used = s_pos - s_page_base;
    if (used == 0u) return 0;
    const int r = F->prog(F->ctx, addr(s_cur) + s_page_base, s_page, used);
    memset(s_page, 0xFF, sizeof s_page);
    s_page_base = s_pos;
    return r;
}

/* new sector for the open log: erase it, header in the page buffer */
static int start_sector(void)
{
    if (s_n && s_list[s_n - 1u].active) {
        /* the open log would overwrite its own first sector: stop it */
        if (s_list[s_n - 1u].nsect >= s_nsect) return -1;
    }
    s_cur = s_wp;
    forget_sector(s_cur);
    if (F->erase(F->ctx, addr(s_cur)) != 0) return -1;
    s_wp = next(s_cur);

    log_sector_hdr_t h;
    memset(&h, 0, sizeof h);
    h.magic = LOG_MAGIC; h.serial = ++s_serial; h.log_id = s_last_id; h.seq = s_seq;
    h.t0_ms = s_t0; h.format = LOG_FORMAT;
    h.crc = log_crc16((const uint8_t *)&h, LOG_HDR_SIZE - 2u);
    memset(s_page, 0xFF, sizeof s_page);
    memcpy(s_page, &h, sizeof h);
    s_page_base = 0; s_pos = LOG_HDR_SIZE;
    set_used(s_cur, 1);
    if (s_seq == 0u) {
        if (s_n == LOG_LIST_MAX) { memmove(&s_list[0], &s_list[1], (LOG_LIST_MAX - 1u) * sizeof s_list[0]); s_n--; }
        log_entry_t *e = &s_list[s_n++];
        memset(e, 0, sizeof *e);
        e->id = s_last_id; e->first = s_cur; e->nsect = 1; e->t0_ms = s_t0; e->active = 1;
    } else {
        s_list[s_n - 1u].nsect++;
    }
    s_seq++;
    return 0;
}

int log_open(uint32_t t0)
{
    if (!F || s_open) return -1;
    s_last_id = (uint16_t)(s_last_id + 1u);
    if (s_last_id == 0u) s_last_id = 1u;
    s_seq = 0; s_t0 = t0;
    if (start_sector() != 0) return -1;
    s_open = 1;
    return 0;
}

int log_append(uint8_t type, const void *payload, uint8_t len)
{
    if (!s_open || len > LOG_REC_MAX || type == 0xFFu) return -1;
    uint8_t rec[LOG_REC_MAX + LOG_REC_OVERHEAD];
    rec[0] = type; rec[1] = len;
    memcpy(&rec[2], payload, len);
    rec[2u + len] = log_crc8(rec, 2u + len);
    const uint32_t n = (uint32_t)len + LOG_REC_OVERHEAD;

    if (s_pos + n > LOG_SECTOR) {
        if (flush_page() != 0) return -1;
        if (start_sector() != 0) { s_open = 0; s_list[s_n - 1u].active = 0; return -1; }
    }
    for (uint32_t k = 0; k < n; ++k) {
        s_page[s_pos - s_page_base] = rec[k];
        s_pos++;
        if (s_pos - s_page_base == LOG_PAGE && flush_page() != 0) return -1;
    }
    return 0;
}

int log_close(void)
{
    if (!s_open) return 0;
    const int r = flush_page();
    s_open = 0;
    if (s_n) s_list[s_n - 1u].active = 0;
    return r;
}

/* ------------------------------------------------------------------ reader */
uint32_t log_size(uint16_t id)
{
    const log_entry_t *e = log_find(id);
    return e ? (uint32_t)e->nsect * LOG_SECTOR : 0u;
}

int log_read(uint16_t id, uint32_t off, void *buf, uint32_t n)
{
    const log_entry_t *e = log_find(id);
    if (!e || e->active || off + n > (uint32_t)e->nsect * LOG_SECTOR) return -1;   /* closed logs only */
    uint8_t *b = (uint8_t *)buf;
    while (n) {
        const uint16_t sect = (uint16_t)((e->first + off / LOG_SECTOR) % s_nsect);
        const uint32_t in = off % LOG_SECTOR;
        uint32_t k = LOG_SECTOR - in;
        if (k > n) k = n;
        if (F->read(F->ctx, addr(sect) + in, b, k) != 0) return -1;
        b += k; off += k; n -= k;
    }
    return 0;
}

int log_erase_all(void)
{
    if (!F || s_open) return -1;
    for (uint16_t s = 0; s < s_nsect; ++s) {
        if (!is_used(s)) continue;
        if (F->erase(F->ctx, addr(s)) != 0) return -1;
        set_used(s, 0);
    }
    s_n = 0; s_wp = 0; s_last_id = 0;
    return 0;
}
