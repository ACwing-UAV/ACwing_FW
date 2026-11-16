# 0015. Firmware and app updates: MCUboot on the FC, the FC as gateway for the flow board, a release manifest on GitHub Pages

- **Status:** Proposed (2026-11-16)
- **Date:** 2026-11-16

## Context

Today the firmware is loaded only with a debugger (ST-LINK / SWD). The flow-bridge firmware is loaded the same way,
on its own board. The app has no idea whether a newer firmware or a newer app exists.

What is asked:

- the firmware of the FC is updated from the app, **over USB and over BLE**;
- at start-up the app checks whether a newer **app** and a newer **firmware** exist, and offers to update both;
- the **FC acts as gateway** for the firmware of the ACwing Flow board (STM32C011 bridge): the app talks only to the FC.

Constraints found in the project:

| Item | Value | Consequence |
|---|---|---|
| FC application area (CPU1) | 512 KB at 0x08000000. The CPU2 BLE stack sits above it. The last 8 KB hold the configuration pages | on the bench a bootloader, the running image and the new image must all fit here. Every ACwing FC (rev A and later) also has the W25Q128 (16 MB) |
| FC firmware today | about 165 KB (bench Debug) | two copies plus a bootloader fit in 512 KB, with a margin of about 65 KB per copy |
| bench (NUCLEO) | no external flash | the new image goes in the internal flash |
| flow bridge | STM32C011F4: **16 KB of flash**, 6 KB of RAM | no room for a bootloader of our own |
| C011 system bootloader (ROM, AN2606) | **I2C1 on PB6/PB7**, 7-bit address **0x64** | **the same pins as the Qwiic bus** (the bridge is a target at 0x3B on I2C1 PB6/PB7). The FC can program the bridge through its ROM bootloader over the existing cable, with no new wires; 0x64 is free on the bus |
| app | PWA (service worker) and native iOS/Android (Capacitor) | the PWA can update itself; the native apps update only through the stores |
| BLE | ATT MTU 247 configured (`CFG_BLE_MAX_ATT_MTU`) | about 240 B per write. The rate depends on the phone; to be measured |
| ADR 0009 | USB is a ground link: no flight commands, no arming with a PC | an update is a ground operation: allowed over USB, only while disarmed |

## Decision

### 1. FC: MCUboot, signed images, test-and-confirm

**MCUboot** (open source, bare-metal port for the STM32WB55) runs first at reset and starts the application.

- **Signature:** ECDSA P-256, made by `imgtool` in the CI. The private key is a GitHub Actions secret. The public key
  is compiled into MCUboot. A **development key** (in the repository, marked as such) signs local builds for the
  bench; a board with the production MCUboot refuses them.
- **Swap with test and confirm** (`swap-using-move`): the new image is swapped in for one boot. The firmware
  **confirms** it only when it is healthy: IMU running, estimator converged, no reset by the watchdog, disarmed,
  60 s up. A reset before the confirmation (crash, watchdog, power) brings back the old image at the next boot.
  A firmware that cannot fly can never become the confirmed one.
- **Downgrade:** allowed, with an explicit confirmation in the app (useful for testing). There is no security counter
  for now. Revisit it if the drone is sold.
- MCUboot itself is never updated in the field: only with SWD, or with the ROM USB DFU (recovery, §6).

**Flash map**

| Area | Bench (NUCLEO: internal flash only) | ACwing FC (rev A and later, W25Q fitted) |
|---|---|---|
| MCUboot | 0x08000000, 40 KB | same |
| slot 0 (running image, MCUboot header + application) | 234 KB | 464 KB |
| slot 1 (new image) | 230 KB, internal | W25Q, the first 512 KB (the flight log starts after them) |
| configuration | 0x0807E000, 8 KB (unchanged) | same |

- On the bench the **largest image is about 228 KB**. Today's 165 KB leaves about 63 KB. The CI must fail above 90 %
  of the slot, as it already does for RAM and flash.
- On the ACwing FC the limit is the whole area (464 KB): slot 1 is on the W25Q, which every FC revision keeps.
- The application is linked after the MCUboot header (vector table at slot 0 + 0x400, VTOR set in `SystemInit`).
  The CubeMX project and the linker scripts change together with this. The CubeMX check of the CI (`check_cubemx.py`)
  gets a new rule for it.
- MCUboot runs before CPU2 is started, so it writes the flash without the CPU1/CPU2 semaphores. The application writes
  slot 1 with the same protocol as the configuration pages (`cfg_flash_wb.c`): HSEM, CPU2 notified.

### 2. Transfer protocol: protocol v2 over USB and BLE

The same commands on both links, handled by the AppTask and by a new low-priority **FwTask** that owns the flash
writes. Allowed **only while disarmed and on the ground**; ARM is refused during a transfer.

| Item | Content |
|---|---|
| `FW_INFO` (request → frame) | per target: version, build, maximum size, state (`target` 0 FC, 1 FLOW) |
| `FW_BEGIN` | target, size, SHA-256 of the image → slot 1 erased in the FwTask (bench: 58 pages, about 1.3 s) |
| `FW_DATA` | offset u32 + data (up to 240 B over BLE, 1 KB over USB), sent in a window |
| `FW_STATUS` frame (5 Hz) | phase, next expected offset, progress, error. The app resends from the next offset: no acknowledge per packet, robust to losses and reconnections |
| `FW_END` | SHA-256 and the signature checked **in the application** before anything is marked (the same verification code as MCUboot) |
| `FW_APPLY` | FC: mark slot 1 for test, reboot. FLOW: start the gateway (§3) |
| `FW_ABORT` | stop, slot 1 left invalid |

- Expected time for 165 KB: **USB** a few seconds; **BLE** about 10–40 s, depending on the phone's connection
  interval. To be measured; if it is too slow, request a short connection interval during the transfer.
- During the erase of each internal page CPU1 stalls for about 20 ms. While disarmed this is acceptable: the IMU
  FIFO absorbs it and the attitude estimate keeps running.

### 3. The FC as gateway for the flow board

The app sends the flow image to the FC (`target` FLOW) with the same commands. The FC stores it (≤ 16 KB: on the
bench in slot 1, on the ACwing FC in the W25Q) and checks its signature. Then:

1. the ExtTask stops polling the Qwiic bus;
2. the FC writes a **bootloader-entry command** to the bridge (new CTRL value with a magic byte, so that no stray
   write can trigger it). The bridge jumps to its ROM bootloader;
3. the FC talks AN4221 (the STM32 I²C bootloader protocol) at 0x64:
   - Get ID: it must be the C011;
   - erase;
   - write in 256-byte blocks (the no-stretch commands);
   - read back and compare;
   - Go to 0x08000000;
4. the bridge restarts. The FC reads the new VERSION, and the ExtTask probes again.

Recovery:
- If it fails half way, the bridge flash is empty or partial. The C011 starts its ROM bootloader by itself when its
  flash is empty (empty check, to be confirmed in RM0490), so the FC can retry.
- The flow board can always be reflashed with SWD.

Flow firmware changes:
- the bootloader-entry command;
- a **full version register** (major.minor.patch: today VERSION is 4 + 4 bits);
- the first time, the new firmware is loaded with SWD, because the bridge in the field today cannot enter its
  bootloader on command.

### 4. Releases: manifest and binaries on GitHub Pages

On a tag `vX.Y.Z` a release workflow:

1. builds and signs the FC firmware (bench, and every FC revision: one image per `hwRev`, Release), the flow firmware
   and the PWA;
2. attaches the binaries to the **GitHub Release** (permanent history, also for downgrades);
3. publishes on **GitHub Pages**, with the documentation, `releases/manifest.json` and all the release binaries.
   GitHub Pages sends the CORS header, while the Release assets do not, so the PWA could not download them directly.

The documentation workflow and the release workflow share the one Pages deployment. The site build includes the
releases folder, downloaded from the GitHub Releases, so a docs deploy does not remove them.

```json
{
  "schema": 1,
  "app":  { "version": "0.2.0", "minFw": "0.15.0",
            "ios": "https://apps.apple.com/…", "android": "https://play.google.com/…" },
  "fc":   { "N": { "version": "0.16.0", "file": "fc-N-0.16.0.signed.bin", "size": 171520,
                   "sha256": "…", "minApp": "0.2.0", "proto": 2, "notes": "…" },
            "A": { "…": "…" } },
  "flow": { "version": "1.1.0", "file": "flow-1.1.0.signed.bin", "size": 9216, "sha256": "…", "minFc": "0.16.0" }
}
```

`fc` has one entry per `hwRev` (`N` bench, `A` the ACwing FC rev A): a later FC revision adds its own key.

### 5. Version check and updates in the app

At start-up and on every connection, if online:
- the app downloads the manifest. When offline it skips silently and shows the time of the last check;
- the downloaded firmware is **kept on the device**, so that the update can be done in the field without a network.

It compares:
- its own version (`package.json`);
- the FC firmware (CAPS `fw[]` and `hwRev`);
- the flow firmware (`FW_INFO`, through the FC).

The compatibility fields (`minFw`, `minApp`, `minFc`, the protocol version of CAPS) decide the order:
1. the app first, when the new firmware needs it;
2. then the FC;
3. then the flow board.

**App:**
- **PWA:** the service worker finds the new version, and the app offers "reload to update".
- **iOS / Android:** a notice with the link to the App Store / Play Store (or TestFlight). No code is downloaded
  outside the stores.

**Firmware:**
- an update screen, as for the calibrations;
- the preconditions are shown and checked: disarmed, on the ground, battery ≥ 50 % or USB power, no RC link, motors
  off;
- then the progress, the reboot, and the confirmation that the new firmware is running and confirmed.

The PC tool `acwing-tools fw flash` uses the same protocol over USB, for development and for recovery from a PC.

### 6. Recovery

If the board cannot start at all (MCUboot damaged):
- ROM USB DFU: BOOT0 (jumper on the NUCLEO; pogo pad TP1 on the ACwing FC rev A), then STM32CubeProgrammer or
  `dfu-util`;
- or SWD.

This is documented in a guide and not offered by the app.

### 7. Out of scope

- The CPU2 BLE stack (ST FUS): it is updated rarely, and an error leaves the BLE unusable. It stays with
  STM32CubeProgrammer over USB, documented.
- The GPS board firmware (u-blox M10): it is a ROM part with its own tools.

## Consequences

- The flash available to the application on the bench drops to about 228 KB. The flight log and the configuration
  do not change.
- Every firmware build becomes a signed MCUboot image. The CI builds MCUboot, signs with the development key, and
  checks the image size against the slot.
- A board flashed directly from STM32CubeIDE without MCUboot keeps working as today, during development: a debug
  configuration links at 0x08000000 without MCUboot. Field updates need MCUboot to be loaded once with SWD.
- The protocol grows by 7 commands and 2 frames: HANDOVER_FW §5 and the app are updated together.
- The flow board needs one more SWD flash, of the version that can enter its bootloader.
- The release key becomes a secret that must not be lost. If it is lost, boards with the production MCUboot can only
  be updated with SWD or ROM DFU.

## Alternatives considered

- **Our own minimal bootloader** (about 12–16 KB, PKA for the signature, copy from staging with power-fail resume).
  Less flash, but every corner case (power loss in the middle of a copy, rollback) would have to be written and
  validated by us. MCUboot already has them and is widely used. Rejected by the owner in favour of MCUboot.
- **ST BLE_Ota** (ST's OTA application): BLE only, ST's own protocol and app, no signature, no USB. Rejected.
- **ROM bootloader only** (USB DFU, reached by a jump from the firmware): no BLE, no signature, no rollback.
  Kept only as recovery.
- **A bootloader of our own on the C011:** there is no room in 16 KB, and the ROM bootloader already listens on the
  right pins.
- **Binaries as GitHub Release assets read by the app:** no CORS, so the PWA cannot download them. Pages is used
  instead.
- **Live update of the web bundle inside the native apps:** fast, but against the spirit of the store rules, and a
  broken bundle would block the app in the field. Rejected by the owner.

## Plan (one PR each, every one with its CI)

1. **MCUboot on the bench:**
   - flash map, linker scripts, VTOR;
   - MCUboot built in the CI;
   - images signed with the development key;
   - the confirmation logic, host-tested;
   - slot size limit in the CI.
2. **Transfer protocol:**
   - the FwTask, the commands, `FW_STATUS`;
   - host tests with a simulated flash, including losses, reconnections, abort and a wrong signature;
   - `acwing-tools fw flash` over USB;
   - the simulator in the app.
3. **App:**
   - the update screen over USB and BLE;
   - downloaded firmware kept on the device;
   - preconditions and progress.
4. **Releases:**
   - the tag workflow, the signing secret, the manifest on Pages;
   - the version check at start-up;
   - the PWA reload, the store links.
5. **Flow gateway:**
   - the bridge firmware (bootloader entry, full version);
   - the AN4221 client in the FC, host-tested against a simulated C011 bootloader;
   - one SWD flash of the bridge.
6. **ACwing FC:** slot 1 on the W25Q (the same for every revision), together with the port to rev A of #18.

Each phase opens its issue of checks on the board.
