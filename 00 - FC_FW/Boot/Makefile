# ACwing FC bootloader: MCUboot for the STM32WB55 (ADR 0015).
#   make                       build/acwing_boot.elf/.bin with the development key
#   make KEY=path/key_pub.c    another public key (releases: the production key)
#   make check-size            fail above the 28 KB of the boot area
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
FC      = ..
MCB     = third_party/mcuboot
KEY    ?= keys/dev_pubkey.c
OUT     = build

CPU     = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
INC     = -Iport -I$(MCB)/boot/bootutil/include -I$(MCB)/boot/bootutil/src \
          -I$(MCB)/ext/tinycrypt/lib/include -I$(MCB)/ext/mbedtls-asn1/include \
          -I"$(FC)/Drivers/CMSIS/Device/ST/STM32WBxx/Include" -I"$(FC)/Drivers/CMSIS/Include"
DEFS    = -DSTM32WB55xx -DKEY_C='"$(abspath $(KEY))"' -DMBEDTLS_CONFIG_FILE='"mbedtls/mbedtls_config.h"'
CFLAGS  = $(CPU) -std=gnu11 -Os -g -ffunction-sections -fdata-sections -Wall -Wno-unused-function $(DEFS) $(INC)

BOOTUTIL = boot_record.c bootutil_area.c bootutil_find_key.c bootutil_img_hash.c bootutil_img_security_cnt.c \
           bootutil_loader.c bootutil_misc.c bootutil_public.c caps.c fault_injection_hardening.c \
           image_ecdsa.c image_validate.c loader.c swap_misc.c swap_move.c tlv.c
SRCS = src/main.c src/startup.c src/wb_flash.c src/keys.c port/flash_map.c \
       $(addprefix $(MCB)/boot/bootutil/src/,$(BOOTUTIL)) \
       $(addprefix $(MCB)/ext/tinycrypt/lib/source/,ecc.c ecc_dsa.c sha256.c utils.c ecc_platform_specific.c) \
       $(MCB)/ext/mbedtls-asn1/src/asn1parse.c $(MCB)/ext/mbedtls-asn1/src/platform_util.c
OBJS = $(patsubst %.c,$(OUT)/%.o,$(SRCS))

all: $(OUT)/acwing_boot.bin check-size

$(OUT)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/acwing_boot.elf: $(OBJS) src/boot.ld
	$(CC) $(CPU) -T src/boot.ld -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections \
	  -Wl,-Map=$(OUT)/acwing_boot.map -Wl,--print-memory-usage -o $@ $(OBJS)

$(OUT)/acwing_boot.bin: $(OUT)/acwing_boot.elf
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

check-size: $(OUT)/acwing_boot.bin
	@s=$$(stat -c %s $<); echo "bootloader $$s B of 28672"; test $$s -le 28672

clean:
	rm -rf build
.PHONY: all clean check-size
