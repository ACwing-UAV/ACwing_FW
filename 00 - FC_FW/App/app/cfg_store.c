/**
 * @file  cfg_store.c
 * @brief A/B image selection, CRC (see cfg_store.h).
 */
#include "cfg_store.h"
#include <string.h>
#include <stddef.h>

uint32_t cfg_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t c = 0xFFFFFFFFu;
    while (len--) {
        c ^= *p++;
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static int valid(const cfg_image_t *im)
{
    return im->magic == CFG_MAGIC && im->version == CFG_VERSION && im->size == sizeof *im &&
           im->crc == cfg_crc32(im, offsetof(cfg_image_t, crc));
}

static int latest_page(void)
{
    int best = -1;
    uint32_t seq = 0;
    for (int p = 0; p < CFG_PAGES; ++p) {
        const cfg_image_t *im = (const cfg_image_t *)cfg_flash_page(p);
        if (!im || !valid(im)) continue;
        if (best < 0 || (int32_t)(im->seq - seq) > 0) { best = p; seq = im->seq; }
    }
    return best;
}

int cfg_load(cfg_image_t *out)
{
    const int p = latest_page();
    if (p < 0) return -1;
    memcpy(out, cfg_flash_page(p), sizeof *out);
    return 0;
}

int cfg_save(cfg_image_t *img)
{
    const int cur = latest_page();
    uint32_t seq = 1;
    int page = 0;
    if (cur >= 0) {
        seq  = ((const cfg_image_t *)cfg_flash_page(cur))->seq + 1u;
        page = (cur + 1) % CFG_PAGES;
    }
    img->magic = CFG_MAGIC; img->version = CFG_VERSION; img->size = sizeof *img;
    img->seq = seq;
    img->crc = cfg_crc32(img, offsetof(cfg_image_t, crc));
    if (cfg_flash_write(page, img, sizeof *img) != 0) return -1;
    return valid((const cfg_image_t *)cfg_flash_page(page)) ? 0 : -1;   /* read back */
}
