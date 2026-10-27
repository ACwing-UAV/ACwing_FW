# Host tests (protocol v2)

Builds the hardware-independent firmware modules with the host compiler and
runs them against simulated time, a simple vertical plant and RAM flash pages:

```
make          # build with AddressSanitizer/UBSan and run
```

Covered: ACK cache, flight core (arming, TAKEOFF/LAND/HOLD, auto-disarm,
app-link failsafe timings, flow lost, ceiling), vertical estimator, VL53L1X and
flow-bridge drivers, AppTask protocol (subscribe order, rates, parameters,
flash, CHECKS, KILL), BLE HI/LO ring priority.

A second binary, `ext_test`, checks the ExtTask logic (`app/ext_sens.c`):
probing, a board plugged in late, polling and a lost device found again. It
uses a VL53L1X register model and the real flow-bridge firmware logic
(`01 - FLOW_FW/Core/Src/bridge.c`) behind a simulated I²C bus.

Every frame the AppTask sends is written to `frames.bin`. To cross-check it
with the app decoders (`02 - Tools/02 - App/src/protocol`), parse it with the
app's `StreamParser` and `decodeFrame()` and re-encode each frame with the
matching `encode*()`: the bytes must be identical.
