# 0003. The flight core runs in the ImuTask, isolated from BLE

- **Status:** Accepted
- **Date:** 2026-10-27

## Context

The ACwing Pilot app controls the drone over BLE (HANDOVER_FW.md §6). The app
link can stall in many places: the radio, the BLE stack on CPU2, the IPCC,
the HCI thread, the AppTask, a flash write. The failsafe must fire **on time**
whatever stalls. The handover says it explicitly: the BLE path only records
the time, and the check runs in the control loop.

## Decision

- `fc_core.c` (arming, modes, automatic actions, failsafes) runs only in the
  **ImuTask**, at about 250 Hz, next to the estimators.
- The BLE write callback (`app_link.c`) only:
  - stores the arrival time (one atomic store);
  - stores the RC values under a seqlock;
  - queues the other commands.
- Every exchange with the AppTask goes through **lock-free SPSC queues**
  (commands, results, events, calibration samples) or **seqlock snapshots**
  (flight state, estimator state).
- The **IWDG** (0.5 s) is fed only by the ImuTask.

Details: [Firmware architecture](../firmware/architecture.md#safety-isolation),
[Flight core](../firmware/flight-core.md).

## Consequences

- No lock on the flight path, and no priority inversion with BLE.
- If the app, BLE or the AppTask stall, the silence grows and the failsafe
  fires at 1 s and 3 s. If the control loop itself stalls, the watchdog
  resets the MCU.
- The protocol logic (`app_task.c`) and the flight core are both plain C and
  are tested together on the PC, including a BLE stall.
- Cost: a core command waits for the next control cycle (≤ 4 ms) before its
  ACK.

## Alternatives considered

- **Flight logic in the AppTask, under a mutex shared with the control
  loop.** That would put BLE timing on the flight path.
- **A separate flight-manager task.** It would add a scheduling dependency
  between two tasks for every failsafe decision.
