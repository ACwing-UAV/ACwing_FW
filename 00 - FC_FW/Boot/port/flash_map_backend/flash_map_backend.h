/**
 * @file  flash_map_backend.h
 * @brief MCUboot flash-area API on the ACwing flash map (acwing_flash_layout.h). The functions are in
 *        flash_map.c; the low-level program / erase is a backend (acw_flash_ops_t) so that the same map serves
 *        MCUboot (bare registers, CPU2 stopped), the application (HSEM, CPU2 running) and the host tests (RAM).
 */
#ifndef ACW_FLASH_MAP_BACKEND_H
#define ACW_FLASH_MAP_BACKEND_H

#include <stdint.h>

struct flash_area {
    uint8_t  fa_id;
    uint8_t  fa_device_id;
    uint16_t pad16;
    uint32_t fa_off;
    uint32_t fa_size;
};

struct flash_sector {
    uint32_t fs_off;
    uint32_t fs_size;
};

/** low level, offsets from ACW_FLASH_BASE; 0 = ok */
typedef struct {
    int (*read)(uint32_t off, void *dst, uint32_t len);
    int (*program)(uint32_t off, const void *src, uint32_t len);   /**< len multiple of 8, off 8-aligned */
    int (*erase_page)(uint32_t off);                               /**< off page-aligned */
} acw_flash_ops_t;
void acw_flash_set_ops(const acw_flash_ops_t *ops);

static inline uint8_t  flash_area_get_device_id(const struct flash_area *fa) { return fa->fa_device_id; }
static inline uint32_t flash_area_get_off(const struct flash_area *fa)       { return fa->fa_off; }
static inline uint32_t flash_area_get_size(const struct flash_area *fa)      { return fa->fa_size; }
static inline uint8_t  flash_area_get_id(const struct flash_area *fa)        { return fa->fa_id; }
static inline uint32_t flash_sector_get_off(const struct flash_sector *fs)   { return fs->fs_off; }
static inline uint32_t flash_sector_get_size(const struct flash_sector *fs)  { return fs->fs_size; }

int      flash_area_open(uint8_t id, const struct flash_area **area);
void     flash_area_close(const struct flash_area *fa);
int      flash_area_read(const struct flash_area *fa, uint32_t off, void *dst, uint32_t len);
int      flash_area_write(const struct flash_area *fa, uint32_t off, const void *src, uint32_t len);
int      flash_area_erase(const struct flash_area *fa, uint32_t off, uint32_t len);
uint32_t flash_area_align(const struct flash_area *fa);
uint8_t  flash_area_erased_val(const struct flash_area *fa);
int      flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors);
int      flash_area_get_sector(const struct flash_area *fa, uint32_t off, struct flash_sector *sector);
int      flash_area_id_from_multi_image_slot(int image_index, int slot);
int      flash_area_id_from_image_slot(int slot);
int      flash_area_id_to_multi_image_slot(int image_index, int area_id);

#endif /* ACW_FLASH_MAP_BACKEND_H */
