/**
 * @file  cfg_flash_wb.c
 * @brief cfg_store flash backend for the STM32WB55, safe with the BLE stack
 *        running on the CPU2 (AN5289, "Flash memory management").
 *
 * Pages: the last two 4 KB pages of the CPU1 application area,
 * 0x0807E000 and 0x0807F000. The linker script keeps them out of FLASH
 * (STM32WB55RGVX_FLASH.ld: FLASH LENGTH = 504K).
 *
 * The erase and the programs go through flash_wb.c (HSEM, CPU2 told about the erase).
 *
 * Called only by the AppTask, only while disarmed: a page erase stalls the
 * CPU1 for ~22 ms (the IMU FIFO absorbs it, the IWDG window is 500 ms).
 */
#include "cfg_store.h"
#include "flash_wb.h"
#include <stdint.h>

#define CFG_PAGE0_ADDR   0x0807E000u
#define CFG_PAGE_SIZE    0x1000u

const void *cfg_flash_page(int page)
{
    if (page < 0 || page >= CFG_PAGES) return NULL;
    return (const void *)(uintptr_t)(CFG_PAGE0_ADDR + (uint32_t)page * CFG_PAGE_SIZE);
}

int cfg_flash_write(int page, const void *data, uint32_t len)
{
    if (page < 0 || page >= CFG_PAGES || len > CFG_PAGE_SIZE) return -1;
    const uint32_t base = (uint32_t)(uintptr_t)cfg_flash_page(page);
    flash_wb_begin();
    int rc = flash_wb_erase_page(base);
    if (rc == 0) rc = flash_wb_program(base, data, len);
    flash_wb_end();
    return rc;
}
