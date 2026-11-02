# 0009. USB is a ground link: no flight commands, no arming while connected

- **Status:** Accepted
- **Date:** 2026-11-02

## Context

The USB CDC port (ADR 0008) carried only the flight log. The ACwing FC has
no ST-LINK virtual COM port, so the PC tools need their telemetry over USB,
and the web app on a PC can talk to the drone through Web Serial. A USB cable
is also a physical tether: a drone that arms or flies with a cable to a
laptop drags the laptop or tears the connector, and a PC program that sends
flight commands next to the pilot's phone is a second pilot.

## Decision

- **USB carries telemetry, parameters, calibrations and the log.** The
  AppTask accepts over USB only PARAM_*, CAL_*, STREAM_RATES, REQUEST and
  SET_NAME; the LogTask keeps LOG_*, SET_TIME and the heartbeat. Every other
  command (ARM, DISARM, KILL, RC, TAKEOFF, LAND, MOTOR_TEST, …) is answered
  `REJECTED / NOT_OVER_USB` (reason 18) and never reaches the flight core. USB
  never feeds the app-link failsafe.
- **No arming while a USB host is connected.** The flight core refuses ARM,
  TAKEOFF and MOTOR_TEST with `REJECTED / USB_CONNECTED` (reason 17) whatever
  link sends them, as long as a host has configured the device; CHECKS item
  12 "USB" is in FAIL, so the app shows a pre-flight blocker. A USB charger
  does not enumerate and does not block. Plugged in while armed, nothing is
  cut (DISARM, LAND, KILL still work).
- **The first link opened has control of the configuration.** With BLE
  (subscribed) and USB (port open: DTR, or a command in the last 3 s) both
  open, configuration changes (PARAM_SET/SAVE/DEFAULTS, CAL_* except ABORT,
  STREAM_RATES, SET_NAME) from the other link are refused with
  `REJECTED / OTHER_LINK` (reason 19); reads and requests are allowed from
  both, and telemetry and events go to both. Control passes to the other
  link when the owner closes. A calibration started over USB is aborted when
  the USB link closes.
- **Transport:** one SPSC TX ring per producing task (LogTask, AppTask,
  TelemetryTask), drained by the LogTask at frame boundaries; ACKs go back
  to the link of the command, with separate ACK-cache entries per link.

## Consequences

- The PC tools get STATE / INIT / STATUS (and REPLAY) over USB on boards
  without a debug probe.
- A drone on the bench cannot be armed from the phone while it is plugged
  into the PC: unplug to fly (the app shows why).
- Two controllers can never fight over the settings, and never over the
  flight.
- RAM: three 4 KB rings and a 2 KB endpoint buffer.

## Alternatives considered

- **USB as a full flight link** (as asked in HANDOVER_APP §10). Rejected for
  the tether risk above.
- **Priority to USB when both are connected.** Rejected: the link a person is
  already using should not lose control because someone plugs in a cable.
