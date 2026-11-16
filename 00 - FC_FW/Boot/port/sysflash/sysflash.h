/* Flash area ids of the ACwing FC (one image, internal flash). */
#ifndef ACW_SYSFLASH_H
#define ACW_SYSFLASH_H
#include "mcuboot_config/mcuboot_config.h"

#define FLASH_DEVICE_INTERNAL_FLASH 0
#define FLASH_SLOT_DOES_NOT_EXIST   255
#define FLASH_AREA_BOOTLOADER       0
#define FLASH_AREA_IMAGE_0_PRIMARY  1
#define FLASH_AREA_IMAGE_0_SECONDARY 2
#define FLASH_AREA_IMAGE_SCRATCH    3          /* not used: swap-using-move */

#define FLASH_AREA_IMAGE_PRIMARY(x)   (((x) == 0) ? FLASH_AREA_IMAGE_0_PRIMARY : FLASH_SLOT_DOES_NOT_EXIST)
#define FLASH_AREA_IMAGE_SECONDARY(x) (((x) == 0) ? FLASH_AREA_IMAGE_0_SECONDARY : FLASH_SLOT_DOES_NOT_EXIST)
#endif
