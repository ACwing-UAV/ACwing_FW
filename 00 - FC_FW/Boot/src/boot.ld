/* ACwing FC bootloader (MCUboot): the first 28 KB of the flash (Boot/port/acwing_flash_layout.h). */
ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 28K
  RAM   (xrw) : ORIGIN = 0x20000008, LENGTH = 0x2FFF8
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
  .text : { . = ALIGN(4); *(.text) *(.text*) *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH
  .ARM.exidx : { *(.ARM.exidx*) } >FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } >RAM AT> FLASH
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >RAM
  /* heap for MCUboot's sector tables and the ASN.1 parser (newlib malloc / sbrk): from here up to the stack */
  end = _ebss;
  _end = _ebss;
  /DISCARD/ : { *(.ARM.attributes) }
}
