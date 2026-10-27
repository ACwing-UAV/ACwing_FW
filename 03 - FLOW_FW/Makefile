# ACwing Flow bridge firmware (STM32C011F4U6)
#   make            build/ACwing_Flow_FW.elf, .bin, .hex
#   make flash      STM32CubeProgrammer CLI over SWD (test pads TP1-TP5)
#   make test       host tests (register map, I2C protocol, PMW3901 sequence)
TARGET  = ACwing_Flow_FW
BUILD   = build
PREFIX ?= arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

SRC = Core/Src/main.c Core/Src/bridge.c Core/Src/pmw3901.c Core/Src/system_stm32c0xx.c
ASM = Core/Startup/startup_stm32c011xx.s
INC = -ICore/Inc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32C0xx/Include

MCU    = -mcpu=cortex-m0plus -mthumb
CFLAGS = $(MCU) -DSTM32C011xx -std=gnu11 -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections $(INC)
LDFLAGS = $(MCU) -TSTM32C011F4UX_FLASH.ld --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections \
          -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

OBJ = $(addprefix $(BUILD)/,$(notdir $(SRC:.c=.o))) $(BUILD)/startup.o

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: Core/Src/%.c Core/Inc/*.h | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@
$(BUILD)/startup.o: $(ASM) | $(BUILD)
	$(CC) $(MCU) -c $< -o $@
$(BUILD)/$(TARGET).elf: $(OBJ) STM32C011F4UX_FLASH.ld
	$(CC) $(OBJ) $(LDFLAGS) -o $@
	$(SIZE) $@
$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@
$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@
$(BUILD):
	mkdir -p $@

flash: $(BUILD)/$(TARGET).elf
	STM32_Programmer_CLI -c port=SWD mode=UR -w $< -v -rst

test:
	$(MAKE) -C Tests/host

clean:
	rm -rf $(BUILD)
	$(MAKE) -C Tests/host clean

.PHONY: all flash test clean
