#!/usr/bin/env python3
"""Invariants of the STM32CubeMX project that a regeneration can break without failing the build.

CubeMX owns Core/, the .ioc and the HAL configuration; the firmware relies on a few of their settings. A
regeneration that changes one of them must fail here, with the reason, instead of on the board:

- USB belongs to App/bsp/usb (ST USB device library, HSI48 + CRS, the CPU2 clock semaphore). The USB peripheral must
  NOT be enabled in CubeMX: it would generate a second hpcd_USB_FS, HAL_PCD_MspInit and MX_USB_PCD_Init (PLLSAI1
  at 48 MHz, which also moves the ADC clock).
- The HAL time base (TIM17) keeps priority 15, the lowest: FreeRTOS and the IMU interrupt must preempt it.
- The motor PWM (bsp/motor_pwm.c) needs TIM2 CH1..CH4 at prescaler 0 and period 1999 (32 kHz with a 64 MHz clock).
- Text files keep LF line endings (.gitattributes): a CRLF regeneration turns every diff into a full-file change.

Run from 00 - FC_FW:  python3 Tests/target/check_cubemx.py
"""
import os, re, subprocess, sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))   # 00 - FC_FW


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8", errors="replace") as f:
        return f.read()


def main():
    errors = []
    ioc = read("ACwing_FC_FW.ioc")
    main_c = read("Core/Src/main.c")
    hal_conf = read("Core/Inc/stm32wbxx_hal_conf.h")

    # USB owned by the application
    if re.search(r"^\s*#define\s+HAL_PCD_MODULE_ENABLED", hal_conf, re.M):
        errors.append("HAL_PCD_MODULE_ENABLED is on: disable the USB peripheral in CubeMX (USB is App/bsp/usb)")
    if "MX_USB_PCD_Init" in main_c or "PCD_HandleTypeDef" in main_c:
        errors.append("main.c initialises USB (MX_USB_PCD_Init): disable the USB peripheral in CubeMX")
    if re.search(r"^Mcu\.IP\d+=USB\s*$", ioc, re.M):
        errors.append(".ioc: the USB IP is enabled (Connectivity > USB: Disable; keep PA11/PA12 as USB_DM/USB_DP signals)")

    # HAL time base priority
    m = re.search(r"#define\s+TICK_INT_PRIORITY\s+(\d+)U", hal_conf)
    if not m or int(m.group(1)) != 15:
        errors.append(f"TICK_INT_PRIORITY is {m.group(1) if m else '?'}, expected 15 (NVIC > TIM1_TRG_COM_TIM17: priority 15)")

    # motor PWM timer
    t = re.search(r"htim2\.Instance = TIM2;(.*?)HAL_TIM_MspPostInit\(&htim2\);", main_c, re.S)
    if not t:
        errors.append("MX_TIM2_Init missing: TIM2 CH1..CH4 PWM is the motor output (bsp/motor_pwm.c)")
    else:
        body = t.group(1)
        if not re.search(r"Init\.Prescaler = 0;", body):
            errors.append("TIM2 prescaler must be 0")
        if not re.search(r"Init\.Period = 1999;", body):
            errors.append("TIM2 period must be 1999 (MOTORS_PWM_TOP, 32 kHz)")
        for ch in range(1, 5):
            if f"TIM_CHANNEL_{ch})" not in body:
                errors.append(f"TIM2 channel {ch} is not configured for PWM")
    if "MX_TIM2_Init();" not in main_c:
        errors.append("MX_TIM2_Init() is not called in main()")

    # line endings of the tracked text files of the project
    files = subprocess.run(["git", "ls-files", "--eol", "--", "."], cwd=ROOT, capture_output=True, text=True).stdout
    crlf = [ln.split("\t")[-1] for ln in files.splitlines() if ln.startswith("i/crlf")]
    if crlf:
        errors.append(f"{len(crlf)} file(s) committed with CRLF, e.g. {crlf[0]} (git add --renormalize .)")

    if errors:
        print("CubeMX project check FAILED:")
        for e in errors:
            print("  -", e)
        return 1
    print("CubeMX project check: OK (USB owned by the app, tick priority 15, TIM2 motor PWM, LF endings)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
