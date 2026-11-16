/**
 * @file  boot_slot.c
 * @brief MCUboot slots seen from the application (ADR 0015): the confirmation of a test boot.
 *
 * The trailer format is MCUboot's: rather than writing it again, this file compiles MCUboot's own
 * bootutil_public.c and the ACwing flash map (Boot/port/flash_map.c) in the application, on top of the flash_wb.c
 * primitives (HSEM, CPU2 handshake). Only in the MCUboot build (APP_MCUBOOT, Tests/target/build_fw.py --mcuboot);
 * the plain build (linked at 0x08000000, for the debugger) has no slots.
 */
#if defined(APP_MCUBOOT) && APP_MCUBOOT
#include "boot_slot.h"
#include "flash_wb.h"
#include "acwing_flash_layout.h"
#include "flash_map_backend/flash_map_backend.h"
#include "bootutil/bootutil_public.h"
#include <string.h>

/* MCUboot sources, compiled here so that the CubeIDE project needs no extra source folder */
#include "../../Boot/port/flash_map.c"
#include "../../Boot/third_party/mcuboot/boot/bootutil/src/bootutil_public.c"

static int rd(uint32_t off, void *dst, uint32_t len)
{
    memcpy(dst, (const void *)(uintptr_t)(ACW_FLASH_BASE + off), len);
    return 0;
}
static int prog(uint32_t off, const void *src, uint32_t len) { return flash_wb_program(ACW_FLASH_BASE + off, src, len); }
static int erase(uint32_t off) { return flash_wb_erase_page(ACW_FLASH_BASE + off); }
static const acw_flash_ops_t s_app_ops = { rd, prog, erase };

int boot_slot_confirm(void)
{
    acw_flash_set_ops(&s_app_ops);
    flash_wb_begin();
    const int rc = boot_set_confirmed();       /* writes image_ok only after a test swap; 0 otherwise too */
    flash_wb_end();
    return rc;
}
#endif
