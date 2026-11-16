# ACwing FC bootloader (MCUboot)

[ADR 0015](../../03%20-%20Docs/decisions/0015-firmware-and-app-updates.md): MCUboot on the STM32WB55. It checks the
ECDSA P-256 signature of the image and swaps a new one in for a test boot. If the firmware does not confirm that
image, the previous one comes back at the next reset.

| Folder | Content |
|---|---|
| `port/` | the ACwing port: `acwing_flash_layout.h` (the flash map, the single source of the addresses), `flash_map.c` (MCUboot flash-area API, portable), `mcuboot_config/`, `sysflash/` |
| `src/` | `main.c` (64 MHz, `boot_go`, jump), `wb_flash.c` (register-level flash driver, the CPU2 is not running yet), `startup.c`, `boot.ld`, `keys.c` |
| `keys/` | the **development** key pair. Its private half is **not a secret**: it signs local and CI builds. Release images use the production key, a GitHub Actions secret, and a bootloader built with `KEY=` its public half |
| `third_party/mcuboot/` | the MCUboot sources used (bootutil, tinycrypt, the mbedtls ASN.1 parser), Apache-2.0, commit in `MCUBOOT_COMMIT` |

```bash
make
```

`make` builds `build/acwing_boot.bin` (about 12.5 KB of the 28 KB area).

```bash
make KEY=path/to/release_pub.c
```

That command builds the bootloader with another public key (`imgtool getpub -k key.pem`).

**Tests:** `Tests/host/boot_test` runs the real `boot_go()` on a flash simulated in RAM: test boot, revert,
confirmation, wrong key, corrupted image, power cut during the swap. The board steps are in
`03 - Docs/firmware/build.md` §7.

**Updating MCUboot itself:** only with SWD or the ROM USB DFU (BOOT0), never from the field.
