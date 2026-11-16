/**
 * @file  flash_wb.c
 * @brief CPU1 flash erase / program safe with the BLE stack on the CPU2 (see flash_wb.h).
 */
#include "flash_wb.h"
#include "main.h"
#include "cmsis_os2.h"
#include "hw_conf.h"
#include <string.h>

#if defined(__has_include)
#if __has_include("shci.h")
#include "shci.h"
#define FLASH_WB_HAVE_SHCI 1
#endif
#endif

volatile uint8_t g_cpu2_ready;

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

void flash_wb_begin(void)
{
#ifdef FLASH_WB_HAVE_SHCI
    if (g_cpu2_ready) {
        (void)SHCI_C2_SetFlashActivityControl(FLASH_ACTIVITY_CONTROL_SEM7);
        (void)SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_ON);
    }
#endif
}

void flash_wb_end(void)
{
#ifdef FLASH_WB_HAVE_SHCI
    if (g_cpu2_ready) (void)SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF);
#endif
}

int flash_wb_erase_page(uint32_t addr) { return single_op(OP_ERASE, addr, 0); }

int flash_wb_program(uint32_t addr, const void *data, uint32_t len)
{
    if (addr % 8u) return -1;
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t off = 0; off < len; off += 8u) {
        uint64_t dw = 0xFFFFFFFFFFFFFFFFull;
        memcpy(&dw, p + off, (len - off) < 8u ? (len - off) : 8u);
        if (single_op(OP_PROGRAM, addr + off, dw)) return -1;
    }
    return 0;
}
