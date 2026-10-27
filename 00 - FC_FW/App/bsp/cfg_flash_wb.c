/**
 * @file  cfg_flash_wb.c
 * @brief cfg_store flash backend for the STM32WB55, safe with the BLE stack
 *        running on the CPU2 (AN5289, "Flash memory management").
 *
 * Pages: the last two 4 KB pages of the CPU1 application area,
 * 0x0807E000 and 0x0807F000. The linker script keeps them out of FLASH
 * (STM32WB55RGVX_FLASH.ld: FLASH LENGTH = 504K).
 *
 * Every erase and every double-word program is one "single flash operation":
 *   1. take HSEM CFG_HW_FLASH_SEMID (flash controller shared by the 2 cores);
 *   2. with interrupts off, start the operation only if the CPU2 does not hold
 *      CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID (radio timing-critical window);
 *      otherwise retry 1 ms later;
 *   3. release the HSEM.
 * The CPU2 is told about the erase (SHCI_C2_FLASH_EraseActivity) and to use
 * semaphore 7 (SHCI_C2_SetFlashActivityControl), only once the wireless stack
 * is running (ble_tlm_svc_init() marks it).
 *
 * Called only by the AppTask, only while disarmed: a page erase stalls the
 * CPU1 for ~22 ms (the IMU FIFO absorbs it, the IWDG window is 500 ms).
 */
#include "cfg_store.h"
#include "main.h"
#include "cmsis_os2.h"
#include "hw_conf.h"
#include <string.h>

#if defined(__has_include)
#if __has_include("shci.h")
#include "shci.h"
#define CFG_HAVE_SHCI 1
#endif
#endif

#define CFG_PAGE0_ADDR   0x0807E000u
#define CFG_PAGE_SIZE    0x1000u

volatile uint8_t g_cpu2_ready;          /* set when the wireless stack is running */

const void *cfg_flash_page(int page)
{
    if (page < 0 || page >= CFG_PAGES) return NULL;
    return (const void *)(uintptr_t)(CFG_PAGE0_ADDR + (uint32_t)page * CFG_PAGE_SIZE);
}

typedef enum { OP_ERASE, OP_PROGRAM } op_t;

static int single_op(op_t op, uint32_t addr, uint64_t dw)
{
    for (int attempt = 0; attempt < 2000; ++attempt) {          /* ~2 s worst case */
        while (HAL_HSEM_FastTake(CFG_HW_FLASH_SEMID) != HAL_OK) osDelay(1);
        HAL_FLASH_Unlock();
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        HAL_StatusTypeDef st = HAL_BUSY;
        const uint32_t pm = __get_PRIMASK();
        __disable_irq();
        if (!HAL_HSEM_IsSemTaken(CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID)) {
            if (op == OP_ERASE) {
                FLASH_EraseInitTypeDef e = { .TypeErase = FLASH_TYPEERASE_PAGES,
                                             .Page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE,
                                             .NbPages = 1 };
                uint32_t bad = 0;
                st = HAL_FLASHEx_Erase(&e, &bad);
            } else {
                st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dw);
            }
        }
        if (!pm) __enable_irq();
        HAL_FLASH_Lock();
        HAL_HSEM_Release(CFG_HW_FLASH_SEMID, 0);
        if (st == HAL_OK) return 0;
        if (st != HAL_BUSY) return -1;                           /* real flash error */
        osDelay(1);                                              /* CPU2 busy: retry */
    }
    return -1;
}

int cfg_flash_write(int page, const void *data, uint32_t len)
{
    if (page < 0 || page >= CFG_PAGES || len > CFG_PAGE_SIZE) return -1;
    const uint32_t base = (uint32_t)(uintptr_t)cfg_flash_page(page);
    int rc = 0;
#ifdef CFG_HAVE_SHCI
    if (g_cpu2_ready) {
        (void)SHCI_C2_SetFlashActivityControl(FLASH_ACTIVITY_CONTROL_SEM7);
        (void)SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_ON);
    }
#endif
    rc = single_op(OP_ERASE, base, 0);
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t off = 0; rc == 0 && off < len; off += 8u) {
        uint64_t dw = 0xFFFFFFFFFFFFFFFFull;
        memcpy(&dw, p + off, (len - off) < 8u ? (len - off) : 8u);
        rc = single_op(OP_PROGRAM, base + off, dw);
    }
#ifdef CFG_HAVE_SHCI
    if (g_cpu2_ready) (void)SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF);
#endif
    return rc;
}
