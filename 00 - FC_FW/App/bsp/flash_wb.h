/**
 * @file  flash_wb.h
 * @brief CPU1 flash writes on the STM32WB55 while the BLE stack runs on the CPU2 (AN5289, "Flash memory
 *        management"). Shared by the configuration store (cfg_flash_wb.c) and the MCUboot slots (boot_slot.c).
 *
 * Every erase and every double-word program is one "single flash operation":
 *   1. take HSEM CFG_HW_FLASH_SEMID (flash controller shared by the 2 cores);
 *   2. with interrupts off, start the operation only if the CPU2 does not hold
 *      CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID (radio timing-critical window); otherwise retry 1 ms later;
 *   3. release the HSEM.
 * flash_wb_begin() / flash_wb_end() bracket a group of operations: the CPU2 is told about the erase
 * (SHCI_C2_FLASH_EraseActivity) and to use semaphore 7, once the wireless stack is running (g_cpu2_ready).
 * RTOS context only (osDelay); a page erase stalls the CPU1 for ~22 ms.
 */
#ifndef FLASH_WB_H
#define FLASH_WB_H

#include <stdint.h>

extern volatile uint8_t g_cpu2_ready;          /**< set by ble_tlm_svc.c when the wireless stack runs */

void flash_wb_begin(void);
void flash_wb_end(void);
int  flash_wb_erase_page(uint32_t addr);                         /**< 4 KB page at addr; 0 = ok */
int  flash_wb_program(uint32_t addr, const void *data, uint32_t len); /**< 8-byte aligned, padded with 0xFF */

#endif /* FLASH_WB_H */
