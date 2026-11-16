/**
 * @file  mcuboot_config.h
 * @brief MCUboot configuration for the ACwing FC (STM32WB55, bare metal), ADR 0015 §1.
 */
#ifndef __MCUBOOT_CONFIG_H__
#define __MCUBOOT_CONFIG_H__

#define MCUBOOT_SIGN_EC256                  /* ECDSA P-256, imgtool -k <key.pem> */
#define MCUBOOT_USE_TINYCRYPT
#define MCUBOOT_SWAP_USING_MOVE 1           /* test boot, confirm, revert; no scratch area */
#define MCUBOOT_VALIDATE_PRIMARY_SLOT       /* the running image is checked at every boot */
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS
#define MCUBOOT_MAX_IMG_SECTORS 64          /* 240 KB / 4 KB = 60 */
#define MCUBOOT_IMAGE_NUMBER    1

/* no logging (no UART in the bootloader); assert stops in a loop (mcuboot_assert.h) */
#define MCUBOOT_HAVE_ASSERT_H
#define MCUBOOT_WATCHDOG_FEED() do { } while (0)
#define MCUBOOT_CPU_IDLE()      do { } while (0)

#endif /* __MCUBOOT_CONFIG_H__ */
