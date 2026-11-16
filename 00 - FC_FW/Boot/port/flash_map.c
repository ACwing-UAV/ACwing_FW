/**
 * @file  flash_map.c
 * @brief MCUboot flash-area API on the ACwing flash map (see flash_map_backend.h, acwing_flash_layout.h).
 *        Portable: the program / erase primitives come from acw_flash_set_ops(). Host-tested.
 */
#include "flash_map_backend/flash_map_backend.h"
#include "sysflash/sysflash.h"
#include "acwing_flash_layout.h"
#include <stddef.h>

static const struct flash_area s_areas[] = {
    { FLASH_AREA_BOOTLOADER,        FLASH_DEVICE_INTERNAL_FLASH, 0, ACW_BOOT_OFF,  ACW_BOOT_SIZE  },
    { FLASH_AREA_IMAGE_0_PRIMARY,   FLASH_DEVICE_INTERNAL_FLASH, 0, ACW_SLOT0_OFF, ACW_SLOT0_SIZE },
    { FLASH_AREA_IMAGE_0_SECONDARY, FLASH_DEVICE_INTERNAL_FLASH, 0, ACW_SLOT1_OFF, ACW_SLOT1_SIZE },
};
#define N_AREAS (sizeof s_areas / sizeof s_areas[0])

_Static_assert(ACW_SLOT0_OFF == ACW_BOOT_OFF + ACW_BOOT_SIZE, "slot 0 follows MCUboot");
_Static_assert(ACW_SLOT1_OFF == ACW_SLOT0_OFF + ACW_SLOT0_SIZE, "slot 1 follows slot 0");
_Static_assert(ACW_SLOT1_OFF + ACW_SLOT1_SIZE <= ACW_CFG_OFF, "slots end before the configuration pages");
_Static_assert(ACW_SLOT0_SIZE == ACW_SLOT1_SIZE + ACW_FLASH_PAGE, "swap-using-move: slot 0 one sector larger");
_Static_assert(ACW_SLOT0_SIZE % ACW_FLASH_PAGE == 0 && ACW_SLOT1_SIZE % ACW_FLASH_PAGE == 0 &&
               ACW_BOOT_SIZE % ACW_FLASH_PAGE == 0, "areas made of whole pages");

static const acw_flash_ops_t *s_ops;
void acw_flash_set_ops(const acw_flash_ops_t *ops) { s_ops = ops; }

int flash_area_open(uint8_t id, const struct flash_area **area)
{
    for (size_t i = 0; i < N_AREAS; ++i)
        if (s_areas[i].fa_id == id) { *area = &s_areas[i]; return 0; }
    return -1;
}

void flash_area_close(const struct flash_area *fa) { (void)fa; }

static int in_area(const struct flash_area *fa, uint32_t off, uint32_t len)
{
    return off <= fa->fa_size && len <= fa->fa_size - off;
}

int flash_area_read(const struct flash_area *fa, uint32_t off, void *dst, uint32_t len)
{
    if (!s_ops || !in_area(fa, off, len)) return -1;
    return s_ops->read(fa->fa_off + off, dst, len);
}

int flash_area_write(const struct flash_area *fa, uint32_t off, const void *src, uint32_t len)
{
    if (!s_ops || !in_area(fa, off, len)) return -1;
    if (((fa->fa_off + off) % ACW_FLASH_WRITE_ALIGN) || (len % ACW_FLASH_WRITE_ALIGN)) return -1;
    return s_ops->program(fa->fa_off + off, src, len);
}

int flash_area_erase(const struct flash_area *fa, uint32_t off, uint32_t len)
{
    if (!s_ops || !in_area(fa, off, len)) return -1;
    if ((off % ACW_FLASH_PAGE) || (len % ACW_FLASH_PAGE)) return -1;
    for (uint32_t p = 0; p < len; p += ACW_FLASH_PAGE)
        if (s_ops->erase_page(fa->fa_off + off + p)) return -1;
    return 0;
}

uint32_t flash_area_align(const struct flash_area *fa) { (void)fa; return ACW_FLASH_WRITE_ALIGN; }
uint8_t  flash_area_erased_val(const struct flash_area *fa) { (void)fa; return 0xFFu; }

int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors)
{
    const struct flash_area *fa;
    if (flash_area_open((uint8_t)fa_id, &fa)) return -1;
    const uint32_t n = fa->fa_size / ACW_FLASH_PAGE;
    if (n > *count) return -1;
    for (uint32_t i = 0; i < n; ++i) { sectors[i].fs_off = i * ACW_FLASH_PAGE; sectors[i].fs_size = ACW_FLASH_PAGE; }
    *count = n;
    return 0;
}

int flash_area_get_sector(const struct flash_area *fa, uint32_t off, struct flash_sector *sector)
{
    if (off >= fa->fa_size) return -1;
    sector->fs_off = off - (off % ACW_FLASH_PAGE);
    sector->fs_size = ACW_FLASH_PAGE;
    return 0;
}

int flash_area_id_from_multi_image_slot(int image_index, int slot)
{
    if (image_index != 0) return -1;
    return slot == 0 ? FLASH_AREA_IMAGE_0_PRIMARY : (slot == 1 ? FLASH_AREA_IMAGE_0_SECONDARY : -1);
}

int flash_area_id_from_image_slot(int slot) { return flash_area_id_from_multi_image_slot(0, slot); }

int flash_area_id_to_multi_image_slot(int image_index, int area_id)
{
    if (image_index != 0) return -1;
    return area_id == FLASH_AREA_IMAGE_0_PRIMARY ? 0 : (area_id == FLASH_AREA_IMAGE_0_SECONDARY ? 1 : -1);
}
