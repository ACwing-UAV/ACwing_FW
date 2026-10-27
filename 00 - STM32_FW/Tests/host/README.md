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

Every frame the AppTask sends is written to `frames.bin`. To cross-check it
with the app decoders (`01 - Tools/02 - App/src/protocol`), parse it with the
app's `StreamParser` and `decodeFrame()` and re-encode each frame with the
matching `encode*()`: the bytes must be identical.
