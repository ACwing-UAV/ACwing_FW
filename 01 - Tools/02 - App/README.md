# ACwing Pilot

App to fly ACwing drones, read their telemetry, change their settings and
calibrate their sensors. One interface for **PC, Mac, tablets and phones**,
delivered as:

- a **web app / PWA** (Chrome or Edge on Windows, macOS, Linux, Android: they
  support Web Bluetooth), installable from the browser;
- a **native iOS / iPadOS app** and a **native Android app** (Capacitor
  shells around the same code; iOS needs the native app because Safari has no
  Web Bluetooth).

The app talks to the drone **directly over Bluetooth LE**: no server, no
account. The drone ↔ app protocol (v2) and what the firmware has to implement
are in **[HANDOVER_FW.md](HANDOVER_FW.md)**.

UI languages: English (default), Italiano, Français, Deutsch, Español
(Settings › General).

## Sections

| Route | Section |
|---|---|
| `#/connect` | Connect (Bluetooth, simulator), pre-flight checks, slide to arm |
| `#/cockpit` | Flight: HUD, virtual sticks (Mode 1/2), flight modes, arm, auto takeoff (1 m default), auto land, RTH (GPS models), pause, motor kill |
| `#/telemetry` | All telemetry, charts, messages, CSV export of the flight log |
| `#/settings/<general\|controls\|safety\|calibration\|connection\|alerts\|about>` | Settings |
| `#/calib/<1…8>` | Calibration wizards: accelerometer, level trim, gyroscope, compass, optical flow + ToF, battery, motors, ELRS sticks |
| `#/help` | Flight controls reference |

Every screen has a phone layout and a tablet/PC layout. Inputs: touch sticks,
keyboard (W/S, A/D, arrows, T takeoff, L land, H RTH, Space pause), gamepad
(Gamepad API); the ELRS radio always overrides the app.

Indoor models (no GPS) are detected from the drone's CAPS frame: every
GPS-only item (RTH, home, distance geofence, GPS failsafe, GPS compass)
disappears and "Return to home" becomes "Land".

## Run

Requirements: Node.js ≥ 20.

```bash
cd "01 - Tools/02 - App"
npm install
npm run dev        # http://localhost:5173 – open in Chrome/Edge
npm test           # protocol, simulator and translation tests
npm run build      # type-check + production build in dist/
```

No drone at hand: **Connect › Simulator › Outdoor drone (GPS)** or
**Indoor drone (no GPS)**. The simulator (`src/sim/`) speaks the real byte
protocol, in 20-byte BLE-sized chunks, and is the behavioural reference for the
firmware.

Web Bluetooth needs a secure context: `localhost` or HTTPS. To use a phone on
the same network during development, serve over HTTPS (e.g. `vite --host`
behind a tunnel) or use the native builds.

## Native apps (Capacitor)

```bash
npx cap add ios        # once
npx cap add android    # once
npm run cap:ios        # build, sync, open Xcode
npm run cap:android    # build, sync, open Android Studio
```

- App id `com.acwing.pilot`, name **ACwing Pilot**; icons in `public/icons/`
  (white ACwing mark on #0E1116).
- iOS: add `NSBluetoothAlwaysUsageDescription` to `Info.plist` ("ACwing Pilot
  connects to your drone over Bluetooth.").
- Android: the `@capacitor-community/bluetooth-le` plugin adds the
  `BLUETOOTH_SCAN`/`BLUETOOTH_CONNECT` permissions; the app scans with
  `neverForLocation`.

## Code map

```
src/
  protocol/   ids.ts (numbers), frame.ts (framing, CRC), messages.ts (frame layouts),
              commands.ts (command layouts), protocol.test.ts
  link/       transport.ts, bleTransport.ts (BLE on web/iOS/Android),
              drone.ts (session store, ACK + retries), flightLog.ts (CSV)
  sim/        simDrone.ts (reference drone), simTransport.ts
  screens/    Connect, Cockpit, Telemetry, Help, settings/*
  calib/      Wizard.tsx (shared shell), wizards.tsx (8 calibrations)
  alerts/     events → toast, voice (speechSynthesis, app language), vibration; callouts
  input/      sticks/keyboard/gamepad mapping, haptics
  i18n/       en (source), it, fr, de, es
  settings/   app-side preferences (this device only)
  ui/         hooks, kit (controls, toasts), icons, formatting, flight widgets
```

Drone-side settings (parameters, calibrations) live on the drone; app-side
preferences (language, units, theme, stick feel, voice) live on the device.

## Status

- App, protocol v2, simulator: complete.
- Firmware side: to be implemented following [HANDOVER_FW.md](HANDOVER_FW.md)
  (today the firmware sends STATE/INIT/STATUS only; the app connects, shows
  the attitude and reports the missing parts as not available).
- Not yet: USB and Wi‑Fi links, firmware update from the app, flight-log
  download from the drone.
