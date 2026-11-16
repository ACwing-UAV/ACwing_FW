/**
 * @file  acwing_flash_layout.h
 * @brief Flash map of the CPU1 application area with MCUboot (ADR 0015 §1). The single source of the addresses:
 *        MCUboot (Boot/), the application (STM32WB55RGVX_FLASH_MCUBOOT.ld, bsp/boot_slot.c) and the CI check
 *        (Tests/target/check_layout.py) read them from here.
 *
 *   0x08000000  MCUboot                         28 KB   ( 7 pages; the build is ~14 KB)
 *   0x08007000  slot 0: running image           240 KB   (60 pages; MCUboot header 0x400, then the application)
 *   0x08043000  slot 1: new image (bench)       236 KB   (59 pages; on the ACwing FC: the W25Q, ADR 0015 phase 6)
 *   0x0807E000  configuration pages (cfg_store)    8 KB  (untouched by MCUboot)
 *   0x08080000  end of the CPU1 area (the CPU2 BLE stack above)
 *
 * swap-using-move needs slot 0 one sector larger than slot 1. Sectors are the 4 KB flash pages.
 */
#ifndef ACWING_FLASH_LAYOUT_H
#define ACWING_FLASH_LAYOUT_H

#define ACW_FLASH_BASE        0x08000000u
#define ACW_FLASH_PAGE        0x1000u          /* 4 KB erase unit */
#define ACW_FLASH_WRITE_ALIGN 8u               /* double-word program */

#define ACW_BOOT_OFF          0x00000000u
#define ACW_BOOT_SIZE         0x00007000u      /* 28 KB, 7 pages */
#define ACW_SLOT0_OFF         0x00007000u
#define ACW_SLOT0_SIZE        0x0003C000u      /* 240 KB, 60 pages */
#define ACW_SLOT1_OFF         0x00043000u
#define ACW_SLOT1_SIZE        0x0003B000u      /* 236 KB, 59 pages */
#define ACW_CFG_OFF           0x0007E000u
#define ACW_AREA_END          0x00080000u

#define ACW_IMAGE_HEADER_SIZE 0x400u           /* imgtool --header-size; keeps the vector table 1 KB aligned */
#define ACW_APP_VECTORS       (ACW_FLASH_BASE + ACW_SLOT0_OFF + ACW_IMAGE_HEADER_SIZE)

#endif /* ACWING_FLASH_LAYOUT_H */
