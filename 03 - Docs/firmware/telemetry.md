# Telemetry protocol


Frame (little-endian): `A5 5A | TYPE | LEN | PAYLOAD[LEN] | CRC16 (LSB, MSB)`,
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.

| Type | Name | Size | Rate | Content |
|---|---|---|---|---|
| 0x01 | STATE | 71 B | 240 Hz (30 Hz with REPLAY) | seq, time, q, bias, acc, gyro, mag, flags |
| 0x02 | INIT | 137 B | after calibration, then ~1 Hz | initial state, references, all filter parameters |
| 0x03 | STATUS | 62 B (54 B before phase 2) | 10 Hz, **always** | IMU init result, IRQ/bus/FIFO counters, calibration progress, drops, BLE link state/MTU/drops/last error |
| 0x04 | REPLAY | ≤ 212 B | 240 Hz (only with `APP_TLM_REPLAY = 1`) | exact filter calls of the window + resulting q, b, diag P |

REPLAY operations: `'P' dθ[3] h` (propagation), `'A' acc[3] trusted`
(acc update), `'M' mag[3] extraVar` (mag update), all float32.

Bandwidth: ~19 kB/s normally, ~44 kB/s with REPLAY (921600 baud ≈ 92 kB/s).
The same frames travel over BLE as a byte stream, STATE decimated and no
REPLAY ([BLE telemetry](ble.md)). The decoders read both STATUS lengths, so phase-1 captures
still work.

Frame types `0x05…0x0F` (CAPS, FLIGHT, POWER, NAV, MOTORS, SYSTEM, EVENT,
PARAM, CALIB, ACK, CHECKS) belong to the app protocol v2, specified in
`02 - Tools/02 - App/HANDOVER_FW.md` and implemented in [ACwing Pilot protocol v2](protocol-v2.md). They are sent over
BLE only; the PC tools ignore them.

> **Pitfall.** Telemetry structs are `packed`: float fields are not 4-byte
> aligned. Never pass the address of such a field to a function that writes
> floats: Cortex-M4F uses `VSTR`, which faults on unaligned addresses. Write to a
> local variable and `memcpy`.
