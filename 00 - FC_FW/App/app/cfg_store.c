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

/* ---- version 1 layout (firmware 0.4.x), kept to migrate it ---- */
#define V1_PARAMS 24                       /* P_TAKEOFF_ALT .. P_COMPASS_PRIMARY */
typedef struct {
    cal_accel_t   accel;
    cal_level_t   level;
    cal_gyro_t    gyro;
    cal_compass_t compass;
    cal_motors_t  motors;
    uint32_t      done_mask;
} cal_set_v1_t;
typedef struct {
    uint32_t     magic;
    uint16_t     version;
    uint16_t     size;
    uint32_t     seq;
    float        params[V1_PARAMS];
    char         name[16];
    cal_set_v1_t cal, bak;
    uint32_t     bak_mask;
    uint32_t     crc;
} cfg_image_v1_t;

/* 0 = not valid, else the version */
static int valid_version(const void *page)
{
    const cfg_image_t *im = (const cfg_image_t *)page;
    if (!im || im->magic != CFG_MAGIC) return 0;
    if (im->version == CFG_VERSION && im->size == sizeof(cfg_image_t) &&
        im->crc == cfg_crc32(im, offsetof(cfg_image_t, crc))) return CFG_VERSION;
    const cfg_image_v1_t *v1 = (const cfg_image_v1_t *)page;
    if (v1->version == 1u && v1->size == sizeof(cfg_image_v1_t) &&
        v1->crc == cfg_crc32(v1, offsetof(cfg_image_v1_t, crc))) return 1;
    return 0;
}
static int valid(const cfg_image_t *im) { return valid_version(im) == CFG_VERSION; }

static int latest_page(void)
{
    int best = -1;
    uint32_t seq = 0;
    for (int p = 0; p < CFG_PAGES; ++p) {
        const cfg_image_t *im = (const cfg_image_t *)cfg_flash_page(p);
        if (!valid_version(im)) continue;
        if (best < 0 || (int32_t)(im->seq - seq) > 0) { best = p; seq = im->seq; }
    }
    return best;
}

static void cal_from_v1(cal_set_t *d, const cal_set_v1_t *s)
{
    cal_set_defaults(d, s->compass.hi);
    d->accel = s->accel; d->level = s->level; d->gyro = s->gyro;
    d->compass = s->compass; d->motors = s->motors;
    d->done_mask = s->done_mask;
}

void cfg_params_put(cfg_image_t *img, const float v[PARAM_COUNT])
{
    memset(img->params, 0, sizeof img->params);
    for (int i = 0; i < PARAM_COUNT && i < (int)CFG_PARAM_SLOTS; ++i) {
        img->params[i].id = g_param_def[i].id;
        img->params[i].value = v[i];
    }
}

void cfg_params_get(const cfg_image_t *img, float v[PARAM_COUNT])
{
    for (int i = 0; i < PARAM_COUNT; ++i) v[i] = g_param_def[i].def;
    for (unsigned k = 0; k < CFG_PARAM_SLOTS; ++k) {
        if (!img->params[k].id) continue;
        const int i = params_index(img->params[k].id);
        if (i >= 0) v[i] = img->params[k].value;
    }
}

int cfg_load(cfg_image_t *out)
{
    const int p = latest_page();
    if (p < 0) return -1;
    const void *pg = cfg_flash_page(p);
    if (valid_version(pg) == CFG_VERSION) { memcpy(out, pg, sizeof *out); return 0; }

    /* version 1 -> 2 */
    const cfg_image_v1_t *v1 = (const cfg_image_v1_t *)pg;
    memset(out, 0, sizeof *out);
    out->magic = CFG_MAGIC; out->version = 1u; out->size = sizeof *out; out->seq = v1->seq;
    float v[PARAM_COUNT];
    for (int i = 0; i < PARAM_COUNT; ++i) v[i] = (i < V1_PARAMS) ? v1->params[i] : g_param_def[i].def;
    cfg_params_put(out, v);
    memcpy(out->name, v1->name, sizeof out->name);
    cal_from_v1(&out->cal, &v1->cal);
    cal_from_v1(&out->bak, &v1->bak);
    out->bak_mask = v1->bak_mask;
    out->batt_used_scale_i = 1.0f;
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

_Static_assert(sizeof(cfg_image_t) <= 4096u, "configuration image must fit one flash page");
_Static_assert(P_COMPASS_PRIMARY == V1_PARAMS - 1, "version 1 parameters are the first 24 of the table");
