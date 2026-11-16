/**
 * @file  wb_flash.c
 * @brief Bare-register flash backend of MCUboot on the STM32WB55. MCUboot runs before the CPU2 is started (C2BOOT
 *        clear after a reset), so the flash controller is CPU1's alone: no HSEM, no SHCI (unlike the application,
 *        bsp/cfg_flash_wb.c). Double-word programming, 4 KB page erase.
 */
#include "flash_map_backend/flash_map_backend.h"
#include "acwing_flash_layout.h"
#include "stm32wbxx.h"
#include <string.h>

#define FLASH_SR_ERRORS (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | \
                         FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR)

static void unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu; }
}
static void lock(void) { FLASH->CR |= FLASH_CR_LOCK; }
static int wait_idle(void)
{
    while (FLASH->SR & (FLASH_SR_BSY | FLASH_SR_CFGBSY)) { }
    const uint32_t err = FLASH->SR & FLASH_SR_ERRORS;
    FLASH->SR = err | FLASH_SR_EOP;                      /* write 1 to clear */
    return err ? -1 : 0;
}

static int rd(uint32_t off, void *dst, uint32_t len)
{
    memcpy(dst, (const void *)(uintptr_t)(ACW_FLASH_BASE + off), len);
    return 0;
}

static int prog(uint32_t off, const void *src, uint32_t len)
{
    const uint8_t *s = (const uint8_t *)src;
    int rc = 0;
    unlock();
    (void)wait_idle();
    for (uint32_t i = 0; rc == 0 && i < len; i += 8u) {
        uint32_t w[2];
        memcpy(w, s + i, 8u);
        volatile uint32_t *dst = (volatile uint32_t *)(uintptr_t)(ACW_FLASH_BASE + off + i);
        FLASH->CR |= FLASH_CR_PG;
        dst[0] = w[0];
        __ISB();
        dst[1] = w[1];
        rc = wait_idle();
        FLASH->CR &= ~FLASH_CR_PG;
        if (rc == 0 && (dst[0] != w[0] || dst[1] != w[1])) rc = -1;
    }
    lock();
    return rc;
}

static int erase(uint32_t off)
{
    unlock();
    (void)wait_idle();
    const uint32_t page = off / ACW_FLASH_PAGE;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    const int rc = wait_idle();
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB_Msk);
    lock();
    return rc;
}

const acw_flash_ops_t g_wb_flash_ops = { rd, prog, erase };
