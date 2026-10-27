# Magnetometer calibration


On the IKS4A1 the LIS2MDL axes are **not** aligned with the LSM6DSV16X axes, and
the board carries a large hard-iron offset. Both are estimated from data by
`calibMagAxes.m`.

**Principle.** The magnetic field is fixed in the world; seen from the rotating
body it must rotate exactly as the integrated gyro says,
$m_j = D_{ij}^\top m_i$. This holds only with the right axis map $M$
($m_{body} = M\,m_{sensor} - h$). All 24 signed-permutation rotations are tried;
for each, the hard-iron $h$ is solved by least squares over pairs of samples
~0.2 s apart:

$$
(I - D^\top) M h = M m_j - D^\top M m_i .
$$

The candidate with the smallest residual wins, and must win by a clear margin.

**Procedure**

1. In `app_config.h` set the identity map and zero offset:
   ```c
   #define MAG_AXIS_MAP   { 1.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f,   0.0f, 0.0f, 1.0f }
   #define MAG_HARD_IRON  { 0.0f, 0.0f, 0.0f }
   ```
   Build with `APP_TLM_REPLAY = 0` and flash.
2. `acwingLiveViewer("serial","COM6")`, reset, keep still 2 s, then ~40 s of slow to
   moderate rotations about **all** axes (tilts included), away from metal objects
   and the USB cable. Close the viewer.
3. Run
   ```matlab
   calibMagAxes('acwing_log_YYYYMMDD_HHMMSS.mat')
   ```
   It prints the residuals of the 4 best hypotheses (warning if the best is not
   clearly separated) and the two lines to paste into `app_config.h`.
4. Rebuild, flash and verify with a new log while rotating in all directions:
   ```matlab
   S = load('acwing_log_....mat'); m = vecnorm(S.L.mag);
   fprintf('|B| mean %.1f uT, variation %.1f%%\n', mean(m), 100*std(m)/mean(m))
   ```
   Expected: |B| ≈ 40–50 µT (local Earth field), variation < 3–5 %. With the
   board level, a 90° turn about the vertical changes yaw by ~90° and tilting
   does not move yaw. A larger variation points to soft-iron effects (ellipsoid
   fit, not yet implemented).

Current calibration (2026-10-07): LIS2MDL rotated 180° about x (y, z flipped),
hard-iron (100.31, −0.56, −166.07) µT. The offset is ~4× the Earth field:
recalibrate whenever anything near the sensor changes.

**Without re-flashing.** A log recorded with an already calibrated firmware
can also be used: pass that firmware's values, and the raw field is
reconstructed. The printed values are always absolute: paste them as they are.

```matlab
calibMagAxes(log, [], [1 0 0, 0 -1 0, 0 0 -1], [100.31 -0.56 -166.07])
```

> **Pitfall.** On a log of a calibrated firmware, calling `calibMagAxes(log)`
> without the firmware values returns ≈ identity and ≈ 0: that is the
> residual *correction*, not the calibration. Pasting it removes the real map
> and offset. The yaw then locks onto a fixed body axis, the magnetometer is
> always de-weighted, and at rest the same axis points the same way whatever
> the orientation.

`fitHardIron.m` (sphere fit) is kept for a quick hard-iron-only check once the
axis map is known.

**Checking the calibration without re-flashing.** `magCheck.m` works on an
ordinary viewer log recorded with the current firmware: board still for 2 s,
then ~30 s of slow rotations about all axes.

```matlab
out = magCheck('acwing_log_YYYYMMDD_HHMMSS.mat', [100.31 -0.56 -166.07])  % current MAG_HARD_IRON
```

It fits a sphere to the logged (already corrected) field and compares how much
the field direction turns with how much the gyro turns. Possible verdicts:

- **consistent**: residual offset ≈ 0, |B| 25–65 µT;
- **`MAG_HARD_IRON` no longer matches**: it prints the corrected value; the axis
  map is unchanged;
- **constant field**: the LIS2MDL is not delivering new data.

**Python**, same algorithms and same numbers (checked on the same log against
the MATLAB functions), with logs from either viewer:

```bash
acwing-tools calib-mag acwing_log_....mat                       # firmware with identity map, zero offset
acwing-tools calib-mag acwing_log_....mat --map 1,0,0,0,-1,0,0,0,-1 --hard-iron 100.31,-0.56,-166.07
acwing-tools fit-hard-iron acwing_log_....mat
acwing-tools mag-check acwing_log_....mat --hard-iron 100.31,-0.56,-166.07
```
