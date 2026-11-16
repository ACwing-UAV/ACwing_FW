#!/usr/bin/env python3
"""The flash map of ADR 0015 is written in several places; they must agree with Boot/port/acwing_flash_layout.h.

- Boot/src/boot.ld: the bootloader fits its area;
- STM32WB55RGVX_FLASH_MCUBOOT.ld: the application starts after the MCUboot header in slot 0 and is no longer than
  slot 1 minus the header and 4 KB for the trailer (an image that links can be received in slot 1);
- STM32WB55RGVX_FLASH.ld (plain build, for the debugger): it stops at the configuration pages;
- App/bsp/cfg_flash_wb.c: the configuration pages where the map puts them.

Run from 00 - FC_FW:  python3 Tests/target/check_layout.py
"""
import os, re, sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


def size(v):
    v = v.strip()
    m = re.fullmatch(r"(\d+)\s*K", v)
    return int(m.group(1)) * 1024 if m else int(v, 0)


def flash_region(ld):
    m = re.search(r"^\s*FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*(\w+)\s*,\s*LENGTH\s*=\s*(\w+)", ld, re.M)
    return int(m.group(1), 0), size(m.group(2))


def main():
    lay = {k: int(v, 0) for k, v in re.findall(r"#define\s+(ACW_\w+)\s+(0x[0-9A-Fa-f]+|\d+)u\b",
                                                read("Boot/port/acwing_flash_layout.h"))}
    base, errs = lay["ACW_FLASH_BASE"], []

    o, l = flash_region(read("Boot/src/boot.ld"))
    if o != base + lay["ACW_BOOT_OFF"] or l != lay["ACW_BOOT_SIZE"]:
        errs.append(f"Boot/src/boot.ld FLASH {o:#x}+{l:#x}, the map says {base:#x}+{lay['ACW_BOOT_SIZE']:#x}")

    o, l = flash_region(read("STM32WB55RGVX_FLASH_MCUBOOT.ld"))
    want_o = base + lay["ACW_SLOT0_OFF"] + lay["ACW_IMAGE_HEADER_SIZE"]
    max_l = lay["ACW_SLOT1_SIZE"] - lay["ACW_IMAGE_HEADER_SIZE"] - 0x1000
    if o != want_o:
        errs.append(f"MCUboot linker script ORIGIN {o:#x}, expected {want_o:#x} (slot 0 + header)")
    if l > max_l:
        errs.append(f"MCUboot linker script LENGTH {l:#x} > {max_l:#x} (slot 1 - header - 4 KB trailer)")

    o, l = flash_region(read("STM32WB55RGVX_FLASH.ld"))
    if o != base or o + l > base + lay["ACW_CFG_OFF"]:
        errs.append(f"plain linker script FLASH {o:#x}+{l:#x} overlaps the configuration pages")

    m = re.search(r"#define\s+CFG_PAGE0_ADDR\s+(0x[0-9A-Fa-f]+)u", read("App/bsp/cfg_flash_wb.c"))
    if not m or int(m.group(1), 16) != base + lay["ACW_CFG_OFF"]:
        errs.append("App/bsp/cfg_flash_wb.c CFG_PAGE0_ADDR differs from ACW_CFG_OFF")

    if errs:
        print("flash layout check FAILED:")
        for e in errs:
            print("  -", e)
        return 1
    print(f"flash layout OK: boot {lay['ACW_BOOT_SIZE'] // 1024} KB, slot 0 {lay['ACW_SLOT0_SIZE'] // 1024} KB, "
          f"slot 1 {lay['ACW_SLOT1_SIZE'] // 1024} KB, configuration at {base + lay['ACW_CFG_OFF']:#x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
